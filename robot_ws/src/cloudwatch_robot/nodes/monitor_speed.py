#!/usr/bin/env python

# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import Float32, Header


class Monitor:

    def __init__(self, data_topic, data_msg, metric_topic, transform):
        self.metrics_pub = rospy.Publisher(metric_topic, Twist, queue_size=1)
        self.topic_sub = rospy.Subscriber(data_topic, data_msg, self.callback)
        self.transform = transform

    def callback(self, message):
        twist = self.transform(message)
        rospy.loginfo('Robot speed: %s', twist)
        self.metrics_pub.publish(twist)


def odom_to_speed(odom):
    return odom.twist.twist


def main():
    rospy.init_node('speed_monitor')
    monitor = Monitor(data_topic='/odom',
                      data_msg=Odometry,
                      metric_topic='/robot_speed',
                      transform=odom_to_speed)
    if (monitor):
        rospy.spin()


if __name__ == '__main__':
    main()
