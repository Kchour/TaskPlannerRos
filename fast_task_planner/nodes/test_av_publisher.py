#!/usr/bin/env python
"""This node publishes a set of waypoint message to the 

How to use. First publish a test set of waypoints to "/input_test_waypoints_av"
>>> rostopic pub /input_test_waypoints_av std_msgs/Float64MultiArray "layout:
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

Now echo "/taskplanner_to_av" to see the result
>>> rostopic echo /taskplanner_to_av

"""
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Test waypoints for the a
#array = [741046.191, 3391408.431, 741045.581, 3391504.677, 741055.435, 3391455.232]

class PlannerAVInterface:
    
    def __init__(self):
        self.sub = rospy.Subscriber('input_test_waypoints_av', Float64MultiArray, self.input_callback)
        self.pub = rospy.Publisher('taskplanner_to_av', Float64MultiArray, queue_size=10, latch=True)
        rospy.init_node('taskplanner_to_av_talker', anonymous=True)
        rospy.loginfo("RUNNING NODE")

    def input_callback(self, msg):
        
        # Create and populate message
        test_msg = Float64MultiArray()
        test_msg.data = msg.data

        # specify the message layout
        # dim[0].label  = "height"
        # dim[0].size   = 480
        # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
        # dim[1].label  = "width"
        # dim[1].size   = 640
        # dim[1].stride = 3*640 = 1920
        # dim[2].label  = "channel"
        # dim[2].size   = 3
        # dim[2].stride = 3

        # create 2 dimensions in the dim array
        test_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()] 
        test_msg.layout.dim[0].label = "num_of_waypoints"
        test_msg.layout.dim[0].size = int(len(test_msg.data)/3)
        test_msg.layout.dim[0].stride = len(test_msg.data)   # num_of_waypoints x 2
        test_msg.layout.dim[1].label = "single_waypoint_packet"
        test_msg.layout.dim[1].size = 3     # utm X and utm Y and now type
        test_msg.layout.dim[1].stride = 3   
        
        hello_str = "task planner publishing waypoints to av %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(test_msg)

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
