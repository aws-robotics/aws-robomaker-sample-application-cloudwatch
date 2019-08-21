#!/usr/bin/env python
"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Permission is hereby granted, free of charge, to any person obtaining a copy of this
 software and associated documentation files (the "Software"), to deal in the Software
 without restriction, including without limitation the rights to use, copy, modify,
 merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from ros_monitoring_msgs.msg import MetricList, MetricData, MetricDimension


class MonitorDistanceToGoal(Node):
    def __init__(self):
        super().__init__('monitor_distance_to_goal')

        self.scan_sub = self.create_subscription("/move_base/NavfnROS/plan", Path, self.report_metric, 5)
        self.metrics_pub = self.create_publisher("/metrics", MetricList, 1)

    def calc_path_distance(self, msg):
        points = [(p.pose.position.x,p.pose.position.y) for p in msg.poses]
        array = np.array(points, dtype=np.dtype('f8','f8'))
        return sum((np.linalg.norm(p0-p1) for p0,p1 in zip(array[:-2],array[1:])))

    def report_metric(self, msg):
        if not msg.poses:
            self.get_logger().debug('Path empty, not calculating distance')
            return

        distance = self.calc_path_distance(msg)
        self.get_logger().debug('Distance to goal: %s', distance)
        
        header = Header()
        header.stamp = rclpy.Time.now()

        dimensions = [MetricDimension(name="robot_id", value="Turtlebot3"),
                      MetricDimension(name="category", value="RobotOperations")]
        metric = MetricData(header=header, metric_name="distance_to_goal",
                            unit=MetricData.UNIT_NONE,
                            value=distance,
                            time_stamp=rclpy.Time.now(),
                            dimensions=dimensions)

        self.metrics_pub.publish(MetricList([metric]))


def main():
    rclpy.init()
    monitor = MonitorDistanceToGoal()
    rclpy.spin(monitor)

if __name__ == '__main__':
    main()
