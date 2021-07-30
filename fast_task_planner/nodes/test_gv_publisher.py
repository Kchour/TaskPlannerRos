#!/usr/bin/env python
"""This node publishes a set of waypoint message to the 

How to use. First publish a test set of waypoints to "/input_test_waypoints_gv"
>>> rostopic pub /input_test_waypoints_gv std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [741046.191, 3391408.431, 0, 741045.581, 3391504.677, 1, 741055.435, 3391455.232, 0]" 

NOTE: waypoint type
0 = nothing
1 = checkpoing
2 = NAI

Now echo "/taskplanner_to_gv" to see the result
>>> rostopic echo /taskplanner_to_gv

"""
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseArray, Pose

class PlannerAVInterface:
    
    def __init__(self):
        self.sub = rospy.Subscriber('input_test_waypoints_gv', Float64MultiArray, self.input_callback)
        self.pub = rospy.Publisher('taskplanner_to_gv1', PoseArray, queue_size=10, latch=True)
        rospy.init_node('taskplanner_to_gv_talker', anonymous=True)
        rospy.loginfo("RUNNING NODE")

    def input_callback(self, msg):             

        # create an empty PoseArray message
        pose_array_msg = PoseArray()

        # unpack incoming msg and fill in pose_array_msg
        for x, y in zip(msg.data[0::3], msg.data[1::3]):
          pose = Pose()
          pose.position.x = x
          pose.position.y = y
          pose_array_msg.poses.append(pose)


        hello_str = "task planner publishing waypoints to gv %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(pose_array_msg)

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
        #talker()
    except rospy.ROSInterruptException:
        pass
