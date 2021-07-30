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
>>> rostopic echo /taskplanner_to_gv

"""
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension    # AV message (must include noi/checkpoints see above)
from geometry_msgs.msg import PoseArray, Pose                      # Gv messages

# add config folder to path
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..',))

from config.vehicle_data import mission_plans, vehicles_container as vc, rewards # rewards only for plotting

# ignore import error if library not installed
try:
  from fastAGC.algorithm_query_single_fixed import AlgorithmFixedSingle
  import fastAGC.config as cfg
  cfg.DEBUGGING = True
except:
  rospy.loginfo("fastAGC library not installed")

from fast_task_planner.msg import MissionPlanArray

class PlannerTaskInterface:
    """Wrapper class around the fastAGC library. Will publish to the
        av and gv topics a set of waypoints, for which a team will meet
        at a rendezvous point

    """
    def __init__(self):
        """Init node, create publishers and subscribers

        """
        # make sure node is init first
        rospy.init_node('task_planner_node', anonymous=True)
        rospy.loginfo("RUNNING TASK_PLANNING NODE")

        # create algorithm object
        try:
          self.alg = AlgorithmFixedSingle(vehicle_container=vc)
        except:
          rospy.loginfo("Skipped initializing FASTAGC")

        # create pub and subs
        self.pub_av = rospy.Publisher('taskplanner_to_av', Float64MultiArray, queue_size=10, latch=True)
        self.pub_gv = rospy.Publisher('taskplanner_to_gv', PoseArray, queue_size=10, latch=True)
        self.sub_mission = rospy.Subscriber('missionPlanner_to_taskPlanner', MissionPlanArray, self.input_callback)

        # TESTING PURPOSES INPUT TOPIC (SEE ABOVE FOR USAGE)
        self.sub_test = rospy.Subscriber('input_test_waypoints', Float64MultiArray, self.test_input_callback)

    def test_input_callback(self, msg):
        
        ####################################
        # Create and populate message for AV
        ####################################
        test_msg = Float64MultiArray()
        test_msg.data = msg.data

        # create 2 dimensions in the dim array
        test_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()] 
        test_msg.layout.dim[0].label = "num_of_waypoints"
        test_msg.layout.dim[0].size = int(len(test_msg.data)/3)
        test_msg.layout.dim[0].stride = len(test_msg.data)   # num_of_waypoints x 2
        test_msg.layout.dim[1].label = "single_waypoint_packet"
        test_msg.layout.dim[1].size = 3     # utm X and utm Y and now type
        test_msg.layout.dim[1].stride = 3   
                
        #####################################
        # Do the same for the GV 
        #####################################

        # create an empty PoseArray message
        pose_array_msg = PoseArray()

        # unpack incoming msg and fill in pose_array_msg
        for x, y in zip(msg.data[0::3], msg.data[1::3]):
          pose = Pose()
          pose.position.x = x
          pose.position.y = y
          pose_array_msg.poses.append(pose)

        hello_str = "task planner publishing waypoints to av %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub_av.publish(test_msg)
        
        hello_str = "task planner publishing waypoints to gv %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub_gv.publish(pose_array_msg)

    def input_callback(self, msg):
      """Receive mission plans, call the task planner (Must be installed)

          Saves rendezvous location into parameter server
          so that it can be used as the starting location next time

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
        pav = PlannerTaskInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
