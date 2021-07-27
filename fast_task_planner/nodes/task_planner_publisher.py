#!/usr/bin/env python3
"""This node publishes a set of waypoint message to the 

How to use. First publish a test set of waypoints to "/input_test_waypoints"
>>> rostopic pub /input_test_waypoints std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [741046.191, 3391408.431, 0, 741045.581, 3391504.677, 1, 741055.435, 3391455.232, 0]" 

NOTE: waypoint type
0 = nothing
1 = Checkpoint
2 = NAI

Now echo "/taskplanner_to_av" to see 
>>> rostopic echo /taskplanner_to_av

"""
import rospy
from std_msgs.msg import Float64MultiArray    # AV message (must include noi/checkpoints see above)
from nav_msgs.msg import Path                 # GV message

# add config folder to path
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..',))
print(os.path.join(os.path.dirname(__file__), '..',))

from config.vehicle_data import mission_plans, vehicles_container as vc, rewards # rewards only for plotting
from fastAGC.algorithm_query_single_fixed import AlgorithmFixedSingle
import fastAGC.config as cfg
cfg.DEBUGGING = True
from fast_task_planner.msg import MissionPlanArray

class PlannerAVInterface:
    
    def __init__(self):
        # make sure node is init first
        rospy.init_node('task_planner_node', anonymous=True)
        rospy.loginfo("RUNNING TASK_PLANNING NODE")

        # create algorithm object
        self.alg = AlgorithmFixedSingle(vehicle_container=vc)

        # create pub and subs
        self.pub_av = rospy.Publisher('taskplanner_to_av', Path, queue_size=10, latch=True)
        self.pub_gv = rospy.Publisher('taskplanner_to_gv', Float64MultiArray, queue_size=10, latch=True)
        self.sub_mission = rospy.Subscriber('missionPlanner_to_taskPlanner', MissionPlanArray, self.input_callback)

    def input_callback(self, msg):
      """Receive mission plans, call the task planner

      """
      rospy.loginfo("TASKPLANNER RECEIVED MISSION")
      teams = {None: []}
      for mission_plan in msg.missions:

        # unpack
        vn = mission_plan.vehicle_name
        ti = mission_plan.team_id
        pi = mission_plan.phase_id
        s = mission_plan.start
        e = mission_plan.end
        r = mission_plan.rewards

        #update start/end locations
        if s:
          # stride length 2 
          start_locs_maps = {vn: s}
          self.alg.update(start_locs_map=start_locs_maps)
        if e:
          rendezvous_locs_maps = {vn: [(a,b) for a,b in zip(e[0::2], e[1::2])]}
          self.alg.update(rendezvous_locs_map=rendezvous_locs_maps)
        
        ############################
        # update rewards on the map
        ###########################
        new_weights = {(p.node.position.x, p.node.position.y): p.value for p in r}
        # first zero out all rewards
        zero_weights = {n: 0 for n in vc.get_vehicle_data(vn).reward_mapping.nodes}
        vc.get_vehicle_data(vn).reward_mapping.set_vertex_weight_bulk(zero_weights)
        # now update rewards
        vc.get_vehicle_data(vn).reward_mapping.set_vertex_weight_bulk(new_weights)
        
      # run algorithm after setting up
      self.alg.run()
      rospy.loginfo("TASKPLANNER FINISHED PLANNING")

      # set rendezvous locations in parameter server
      rendez = {}
      for av in self.alg.avs: 
        rendez.update({av: self.alg.veh_results(av).waypoints[-1]})
      for gv in self.alg.gvs:
        rendez.update({gv: self.alg.veh_results(gv).waypoints[-1]})
      rospy.set_param("rendezvous", rendez)

#def talker():
#    rospy.Subscriber('input_waypoints', Path, input_callback)
#    pub = rospy.Publisher('taskplanner_to_av', Float64MultiArray, queue_size=10, latch=True)
#    rospy.init_node('taskplanner_to_av_talker', anonymous=True)
#    rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#        hello_str = "task planner publishing waypoints to av %s" % rospy.get_time()
#        rospy.loginfo(hello_str)
#        pub.publish(test_msg)
#        rate.sleep()

if __name__ == '__main__':
    try:
        pav = PlannerAVInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
