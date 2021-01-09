#!/usr/bin/env python
"""
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the 'Software'), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so.

THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import time

from ros_monitoring_msgs.msg import MetricData, MetricDimension, MetricList
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class MonitorNearestObstacle:

    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, callback=self.report_metric)
        self.metrics_pub = rospy.Publisher('/metrics', MetricList, queue_size=1)

    def filter_scan(self, msg):
        rospy.loginfo(
            'Filtering scan values in value range (%s,%s)', msg.range_min, msg.range_max
        )
        return [
            msg.ranges[i]
            for i in range(360)
            if msg.ranges[i] >= msg.range_min and msg.ranges[i] <= msg.range_max
        ]

    def report_metric(self, msg):
        filtered_scan = self.filter_scan(msg)
        if not filtered_scan:
            rospy.loginfo(
                'No obstacles with scan range (%s,%s)', msg.range_min, msg.range_max
            )
            return

        min_distance = min(filtered_scan)
        rospy.loginfo('Nearest obstacle: %s', min_distance)

        header = Header()
        header.stamp = rospy.Time.from_sec(time.time())

        dimensions = [
            MetricDimension(name='robot_id', value='Turtlebot3'),
            MetricDimension(name='category', value='RobotOperations'),
        ]
        metric = MetricData(
            header=header,
            metric_name='nearest_obstacle_distance',
            unit=MetricData.UNIT_NONE,
            value=min_distance,
            time_stamp=rospy.Time.from_sec(time.time()),
            dimensions=dimensions,
        )

        self.metrics_pub.publish(MetricList([metric]))


def main():
    rospy.init_node('monitor_obstacle_distance')
    try:
        monitor = MonitorNearestObstacle()
        if (monitor):
            rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
