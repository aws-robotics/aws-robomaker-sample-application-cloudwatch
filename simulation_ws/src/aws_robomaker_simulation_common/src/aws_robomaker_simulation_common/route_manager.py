#!/usr/bin/env python
"""
 Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

 Licensed under the Apache License, Version 2.0 (the "License").
 You may not use this file except in compliance with the License.
 A copy of the License is located at

  http://aws.amazon.com/apache2.0

 or in the "license" file accompanying this file. This file is distributed
 on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 express or implied. See the License for the specific language governing
 permissions and limitations under the License.
"""


import itertools
import random
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.action import MoveBase


class RouteManager(Node):
    '''Send goals to move_base server for the specified route. Routes forever.

       Loads the route from yaml. 
       Use RViz to record 2D nav goals. 
       Echo the input goal on topic /move_base_simpl/goal

       Format: 

            order: inorder
            poses: 
                - pose: 
                      position: 
                        x: -5.41667556763
                        y: -3.14395284653
                        z: 0.0
                      orientation: 
                        x: 0.0
                        y: 0.0
                        z: 0.785181432231
                        w: 0.619265789851
    
    '''

    # return an iterator over the goals
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
    }

    def __init__(self):
        super().__init__('route_manager')
        self.route = []

        self.client = ActionClient(self, MoveBase, 'move_base')
        self.client.wait_for_server()

        route_file = rclpy.get_param('route_file')
        with open(route_file, 'r') as f:
            route_file_contents = f.read()
        route_yaml = yaml.safe_load(route_file_contents)

        self.route_mode = route_yaml['mode']
        if self.route_mode not in RouteManager.route_modes:
            self.get_logger().error("Route mode '%s' unknown, exiting route manager", self.route_mode)
            return

        poses = route_yaml['poses']
        if not poses:
            self.get_logger().info("Route manager initialized no goals, unable to route")

        self.goals = RouteManager.route_modes[self.route_mode](poses)
        self.get_logger().info("Route manager initialized with %s goals in %s mode", len(poses), self.route_mode)

    def to_move_goal(self, pose):
        goal = MoveBase()
        goal.target_pose.header.stamp = self.get_clock().now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(**pose['pose']['position'])
        goal.target_pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def route_forever(self):
        while rclpy.ok():
            try:
                self.get_logger().info("Route mode is '%s', getting next goal", self.route_mode)
                current_goal = self.to_move_goal(next(self.goals))
                self.get_logger().info("Sending target goal: %s", current_goal)
                self.client.send_goal(current_goal)

                if not self.client.wait_for_result():
                    self.get_logger().error("Move server not ready, will try again...")
                else:
                    if self.client.get_result():
                        self.get_logger().info("Goal done: %s", current_goal)
                time.sleep(1)
            except StopIteration:
                self.get_logger().info("No goals, stopping route manager")
                return


def main():
    rclpy.init()
    try:
        route_manger = RouteManager()
        route_manger.route_forever()
    except rclpy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
