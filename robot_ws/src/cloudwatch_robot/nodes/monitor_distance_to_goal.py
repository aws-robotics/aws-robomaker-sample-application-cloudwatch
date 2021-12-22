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

from itertools import izip
import time

from nav_msgs.msg import Path
import numpy as np
import rospy
from std_msgs.msg import Float32, Header


class MonitorDistanceToGoal:

    def __init__(self):
        self.scan_sub = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, callback=self.report_metric
        )
        self.metric_pub = rospy.Publisher('/distance_to_goal', Float32, queue_size=1)

    def calc_path_distance(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        array = np.array(points, dtype=np.dtype('f8', 'f8'))
        return sum((np.linalg.norm(p0 - p1) for p0, p1 in izip(array[:-2], array[1:])))

    def report_metric(self, msg):
        if not msg.poses:
            rospy.loginfo('Path empty, not calculating distance')
            return

        distance = self.calc_path_distance(msg)
        rospy.loginfo('Distance to goal: %s', distance)

        self.metric_pub.publish(distance)


def main():
    rospy.init_node('monitor_goal_to_distance')
    try:
        monitor = MonitorDistanceToGoal()
        if (monitor):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
