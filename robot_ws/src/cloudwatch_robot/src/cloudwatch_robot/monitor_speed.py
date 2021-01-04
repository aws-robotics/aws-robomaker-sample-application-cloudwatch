#!/usr/bin/env python
"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the "Software"),
 to deal in the Software without restriction, including without limitation
 the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from ros_monitoring_msgs.msg import MetricList, MetricData, MetricDimension
from nav_msgs.msg import Odometry


class Monitor(Node):
    def __init__(self, data_topic, data_msg, metric_topic):
        super().__init__('speed_monitor')
        self.metrics_pub = self.create_publisher(MetricList, metric_topic, 1)
        self.topic_sub = self.create_subscription(
            data_msg, data_topic, self.callback, 5
        )

    def callback(self, message):
        self.metrics_pub.publish(self.odom_to_speed(message))

    def odom_to_speed(self, odom):
        header = Header()
        timestamp = self.get_clock().now().to_msg()
        header.stamp = timestamp

        dimensions = [
            MetricDimension(name='robot_id', value='Turtlebot3'),
            MetricDimension(name='category', value='RobotOperations'),
        ]

        linear_speed = MetricData(
            header=header,
            metric_name='linear_speed',
            unit=MetricData.UNIT_NONE,
            value=odom.twist.twist.linear.x,
            time_stamp=timestamp,
            dimensions=dimensions,
        )

        angular_speed = MetricData(
            header=header,
            metric_name='angular_speed',
            unit=MetricData.UNIT_NONE,
            value=odom.twist.twist.angular.z,
            time_stamp=timestamp,
            dimensions=dimensions,
        )

        return MetricList(metrics=[linear_speed, angular_speed])


def main():
    rclpy.init()
    monitor = Monitor(
        data_topic='/odom',
        data_msg=Odometry,
        metric_topic='/metrics')
    rclpy.spin(monitor)


if __name__ == '__main__':
    main()
