#!/usr/bin/env python

import rospy
import rostest
import time
import os
import sys
import unittest

from rosgraph_msgs.msg import Clock
from ros_monitoring_msgs.msg import MetricList
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags
from robomaker_testing import utils

TEST_NAME = 'simple_test'
OUTPUT_FILE = '/tmp/TEST-{}-OUTPUT.xml'.format(TEST_NAME)


class SimpleTest(unittest.TestCase):

    def setUp(self):
        self.test_name = "simple_test"
        rospy.loginfo("Test Name: %s", self.test_name)
    
    def test_sum(self):
        self.assertEqual(sum([1, 2, 3]), 6, "Should be 6")

    def test_sum_tuple(self):
        self.assertEqual(sum((1, 2, 2)), 6, "Should be 6")
        

if __name__ == "__main__":
    
    def override_shutdown(msg):
        pass
    

    
    # rostest.rosrun will call rospy.signal_shutdown before we can upload the
    # test results - we override the signal_shutdown call and reset it later
    rospy.init_node(TEST_NAME, log_level=rospy.INFO)
    rospy_signal_shutdown = rospy.signal_shutdown
    rospy.signal_shutdown = override_shutdown

    # rostest.rosrun will call sys.exit before we can upload the test results
    # we catch the exception, upload the file and reset and call
    # rospy.signal_shutdown before we exit
    try:
        args = sys.argv
        args.append(rostest.XML_OUTPUT_FLAG+OUTPUT_FILE)
        rostest.rosrun("aws_robomaker_test_framework", TEST_NAME, SimpleTest, sysargs=args)
    except SystemExit:
        utils.upload_file(OUTPUT_FILE)
        rospy.signal_shutdown = rospy_signal_shutdown
        rospy.signal_shutdown("fin")
