#!/usr/bin/env python3
PKG='fast_task_planner'

import sys
import unittest
import rospy

from std_msgs.msg import Int64

## A sample python unit test
class TestTaskPlannerPublisher(unittest.TestCase):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        
        # create a shared publisher
        self.pub = rospy.Publisher('input_phase', Int64,  queue_size=10, latch=True)

    def setUp(self) -> None:
        """Make sure rendezvous params are empty"""
        if rospy.has_param("rendezvous"):
            rospy.delete_param("rendezvous")

    def test_sequential_mission(self):
        """Runs phases of the mission in sequence from 1,2,3

            Use rostest publishtest to check health of task_planner_publisher

        """
        phase_msg = Int64()

        # first phase
        phase_msg.data = 1
        self.pub.publish(phase_msg)

        # second phase
        phase_msg.data = 2
        self.pub.publish(phase_msg)

        # third phase
        phase_msg.data = 3
        self.pub.publish(phase_msg)

    def test_failure_out_of_order(self):
        """this case will fail!
        """
        # third phase
        phase_msg.data = 3
        self.pub.publish(phase_msg)

if __name__ == '__main__':
    # make sure to init this first
    rospy.init_node('test_task_planner_node', anonymous=True)

    # next two lines must be there
    import rostest
    rostest.rosrun(PKG, 'test_task_planner_publisher', TestTaskPlannerPublisher)

    # shutdown upon finishing tests
    rospy.signal_shutdown("FINISHED WITH: test_task_planner_publisher")