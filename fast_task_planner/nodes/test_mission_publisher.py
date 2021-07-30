#!/usr/bin/env python3
"""This (test) node will test allow the user to publish a specific mission plan
    to the task_planner

"""
import rospy
from std_msgs.msg import Int64
from geometry_msgs.msg import Pose
from fast_task_planner.msg import MissionPlanArray, MissionPlan, Reward

# add config folder to path
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..',))
print(os.path.join(os.path.dirname(__file__), '..',))

from config.vehicle_data import mission_plans

class TestMissionPlanner:
    
    def __init__(self):
      
        rospy.init_node('mission_planner_node', anonymous=True)
        rospy.loginfo("RUNNING MISSION NODE")

        self.sub = rospy.Subscriber('input_phase', Int64, self.input_callback)
        self.pub = rospy.Publisher('missionPlanner_to_taskPlanner', MissionPlanArray, queue_size=10, latch=True)

    def input_callback(self, msg):
        # get phase_id
        phase_id = msg.data

        # create an empty mission plan array
        mission_plan_array_msg = MissionPlanArray()

        # now get relevant mission plans for each team based on phase_id
        for team_id in mission_plans[phase_id]:
          
          for vehicle in mission_plans[phase_id][team_id]:
            
            # create an empty mission plan message for population
            mission_plan_msg = MissionPlan()

            # extract relevant data
            team_id = mission_plans[phase_id][team_id][vehicle].team_id
            phase_id = mission_plans[phase_id][team_id][vehicle].phase_id
            start = mission_plans[phase_id][team_id][vehicle].start
            end = mission_plans[phase_id][team_id][vehicle].end
            reward_values = mission_plans[phase_id][team_id][vehicle].reward_values

            # populate mission plan message
            mission_plan_msg.team_id = team_id
            mission_plan_msg.vehicle_name = vehicle
            mission_plan_msg.phase_id = phase_id
            # if start is specified
            if start:
                mission_plan_msg.start.append(start[0]) 
                mission_plan_msg.start.append(start[1]) 
            else:
                # reuse last known rendezvous location
                rendez = rospy.get_param("rendezvous")
                pt = rendez[vehicle]
                mission_plan_msg.start.append(pt[0]) 
                mission_plan_msg.start.append(pt[1]) 

            # possible to have multiple rendezvous locations (stride 2)
            for e in end:
                mission_plan_msg.end.append(e[0])
                mission_plan_msg.end.append(e[1])

            # populate rewards nodes (stride 3)
            for rw, val in reward_values.items():
                r = Reward()
                r.node.position.x = rw[0]
                r.node.position.y = rw[1]
                r.value = val

                mission_plan_msg.rewards.append(r)
            
            # now add to mission_plna_array message
            mission_plan_array_msg.missions.append(mission_plan_msg)
        
        # publish mission_plan_array_msg
        self.pub.publish(mission_plan_array_msg)
        rospy.loginfo("PUBLISHED MISSON TO TASKPLANNER")

if __name__ == '__main__':
    try:
        pav = TestMissionPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
