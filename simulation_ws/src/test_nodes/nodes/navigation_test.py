#!/usr/bin/env python

import rospy
import rostest
import time
import os
import unittest

from rosgraph_msgs.msg import Clock
from ros_monitoring_msgs.msg import MetricList
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags

METRIC_NAME = "distance_to_goal"


class NavigationTest(unittest.TestCase):
    """
    This test case will send a number of expected goals and monitor their status.
    If the robot reaches all of the destinations, it will mark the test as passed.
    """

    def cancel_job(self):
        rospy.wait_for_service("/robomaker/job/cancel")
        requestCancel = rospy.ServiceProxy("/robomaker/job/cancel", Cancel)
        response = requestCancel()
        if response.success:
            self.is_cancelled = True
            rospy.loginfo("Successfully requested cancel job")
            self.set_tag(
                name=self.test_name +
                "_Time_Elapsed_End",
                value=str(
                    time.time()).split(
                    ".",
                    1)[0])
        else:
            rospy.logerr("Cancel request failed: %s", response.message)

    def set_tag(self, name, value):
        rospy.wait_for_service("/robomaker/job/add_tags")
        requestAddTags = rospy.ServiceProxy("/robomaker/job/add_tags", AddTags)
        tags = ([Tag(key=name, value=value)])
        response = requestAddTags(tags)
        if response.success:
            rospy.loginfo("Successfully added tags: %s", tags)
        else:
            rospy.logerr(
                "Add tags request failed for tags (%s): %s",
                tags,
                response.message)

    def setUp(self):
        self.latch = False
        self.successful_navigations = 0
        self.test_name = "Robot_Monitoring_Tests_" + \
            str(time.time()).split(".", 1)[0]
        self.is_completed = False
        rospy.loginfo("Test Name: %s", self.test_name)
        self.navigation_success_count = rospy.get_param(
            'NAVIGATION_SUCCESS_COUNT')
        self.timeout = rospy.get_param("SIM_TIMEOUT_SECONDS")

    def set_latched(self):
        self.latch = True

    def set_unlatched(self):
        self.latch = False

    def increment_navigations(self):
        self.successful_navigations = self.successful_navigations + 1

    def is_complete(self):
        return self.successful_navigations >= self.navigation_success_count

    def check_timeout(self, msg):
        """
            Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).
        """
        if msg.clock.secs > self.timeout and not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            self.set_tag(
                name=self.test_name +
                "_Timed_Out",
                value=str(
                    self.timeout))
            self.cancel_job()

    def check_complete(self, msgs):
        for msg in msgs.metrics:
            if msg.metric_name == METRIC_NAME:
                rospy.loginfo("Metric Name: %s", msg.metric_name)
                rospy.loginfo("Metric Value: %s", msg.value)
                """
                    If our distance to goal metric drops below .5 and we've
                    achieved our goal count then we are complete and we tag the
                    job success and cancel. Else, we continue checking progress
                    towards a new goal once the distance to goal climbs back
                    above 1. Note that we're using what the nav stack thinks
                    is the distance to goal, in the real world we'd want to use
                    a ground truth value to ensure accuracy.
                """
                if msg.value <= .5 and self.is_completed == False and self.latch == False:
                    self.set_latched()
                    self.increment_navigations()
                    self.set_tag(
                        name=self.test_name + "_Successful_Nav_" + str(
                            self.successful_navigations), value=str(
                            self.successful_navigations))
                    if self.is_complete():
                        self.is_completed = True
                        self.set_tag(
                            name=self.test_name + "_Status", value="Passed")
                        self.cancel_job()
                elif msg.value > 1 and self.is_completed == False:
                    self.set_unlatched()

    def test_navigation(self):
        try:
            self.is_cancelled = False
            self.set_tag(
                name=self.test_name +
                "_Time_Elapsed_Start",
                value=str(
                    time.time()).split(
                    ".",
                    1)[0])
            rospy.Subscriber("/metrics", MetricList, self.check_complete)
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            rospy.spin()
        except Exception as e:
            rospy.logerror("Error", e)
            self.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the
            # simulation. We don't exit.
            self.cancel_job()

    def runTest(self):
        # Start the navigation test
        self.test_navigation()


if __name__ == "__main__":
    rospy.init_node("navigation_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "navigation_test", NavigationTest)
