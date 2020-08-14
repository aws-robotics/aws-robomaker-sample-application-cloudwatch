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
import random
import itertools
import rospy
import actionlib
import tf.transformations as transform
from geometry_msgs.msg import Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from geometry_msgs.msg import Pose
from math import cos, sin
import numpy as np
import time


class GoalGenerator():
    '''
    Reads map data published on /map and /map_metadata topics and provides valid random goal poses in the map.

    Assumes that the map is held static after node initialisation and is not updated while the node is running.
    '''

    def __init__(self):
        # Assuming map is static after node init and not updated while the node is running. If not, this must be refreshed at regular intervals or on some callbacks. 
        self.meta_data = rospy.wait_for_message("map_metadata", MapMetaData)
        self.occupancy_data = rospy.wait_for_message("map", OccupancyGrid)
        
        # map.yaml only specifies x,y and yaw transforms of the origin wrt world frame 
        self.map_orientation = transform.euler_from_quaternion([self.meta_data.origin.orientation.x,\
                                                               self.meta_data.origin.orientation.y,\
                                                               self.meta_data.origin.orientation.z,\
                                                               self.meta_data.origin.orientation.w])
        self.map_yaw = self.map_orientation[2]  #(in radians)
        self.map_origin_x0 = self.meta_data.origin.position.x
        self.map_origin_y0 = self.meta_data.origin.position.y 
        self.resolution = self.meta_data.resolution


    def ravel_index(self, x, y):
        '''
        - description:
            ravel 2d grid coordinates in row-major order   
        - input: 
            - ints
                - x, y (in grid coordinates)
        - output: int
        '''
        return y*(self.meta_data.width) + x


    def grid_to_world_2d(self, x, y):
        '''
        - description:
            transform x-y planar grid coordinates to world coordinates
            adheres to the assumption that grid-world transform is only x-y translation and yaw rotation   
        - input: 
            - int, int
                - x, y (in grid coordinates)
        - output: [int, int]
                x_world, y_world (in world coordinates)
        '''
        x_world = self.map_origin_x0 + (cos(self.map_yaw)*(self.resolution*x) - sin(self.map_yaw)*(self.resolution*y))
        y_world = self.map_origin_y0 + (sin(self.map_yaw)*(self.resolution*x) + cos(self.map_yaw)*(self.resolution*y))

        return [x_world, y_world]


    def _create_pos(self, x_world, y_world, z_world, euler_orientation_x, euler_orientation_y, euler_orientation_z):
        '''
        - description:
            wrap 3D euler location and orientation input to a Pose  
        - input: 
            - x, y, z (in world coordinates)
            - orientation_x, orientation_y, orientation_z (in radians)
        - output: 
                format 
                    - pose: 
                          position: 
                            x: double
                            y: double
                            z: double
                          orientation:
                            x: double
                            y: double
                            z: double
                            w: double
        '''
        position = {
                    'x':x_world,
                    'y':y_world,
                    'z':z_world
                    }
        
        # Make sure the quaternion is valid and normalized
        quaternion_orientation = transform.quaternion_from_euler(euler_orientation_x, euler_orientation_y, euler_orientation_z)
        orientation = {
                        'x':quaternion_orientation[0],
                        'y':quaternion_orientation[1],
                        'z':quaternion_orientation[2],
                        'w':quaternion_orientation[3]
                      }

        p = {
                'pose':
                    {
                        'position':position,
                        'orientation':orientation
                    }
            }

        return p


    def check_noise(self, x, y, row_id=None):
        '''
        - description:
            Low resolution/ noisy sensor data might lead to noisy patches in the map.
            This function checks if the random valid point is not a noisy bleap on the map by looking for its neighbor consistency. 
        - input: 
            - x (in grid coordinates)
            - y (in grid coordinates)
        - output: bool
            - false if noise, else true
        '''

        delta_x=max(2,self.meta_data.width//50) #to make it depend on resolution
        delta_y=max(2,self.meta_data.height//50)

        l_bound, r_bound = max(0, x-delta_x), min(self.meta_data.width-1, x+delta_x)
        t_bound, b_bound = max(0, y-delta_y), min(self.meta_data.height-1, y+delta_y)

        for _x in range(l_bound, r_bound):
            for _y in range(t_bound, b_bound):
                _row_id = self.ravel_index(_x, _y)
                if self.occupancy_data.data[_row_id]!=0:
                    return False

        return True

    def __iter__(self):
        return self


    def next(self):
        '''
        - description
            for python 2.x support
        '''
        return self.__next__()


    def __next__(self):
        '''
        - description:
            Scans the map for a valid goal. 
            Converts to world coordinates and wraps as a Pose to be consumed by route manager.
        - input: 
        - output:  
                - Pose: 
                      position: 
                        x: double
                        y: double
                        z: double
                      orientation: 
                        x: double
                        y: double
                        z: double
                        w: double
        '''
        z_world_floor = 0.
        euler_orientation = [0.,0.,0.]

        rospy.loginfo("Searching for a valid goal")
        timeout_iter = 100
        iteration = 0
        while iteration<timeout_iter:
          _x, _y = random.randint(0,self.meta_data.width-1), random.randint(0,self.meta_data.height-1)
          _row_id = self.ravel_index(_x, _y)
          if self.occupancy_data.data[_row_id]==0 and self.check_noise(_x, _y, row_id=_row_id): 
            x_world, y_world  = self.grid_to_world_2d(_x, _y)
            rospy.loginfo("Valid goal found!")
            return self._create_pos(x_world, y_world, z_world_floor, *euler_orientation)

        rospy.logerr("Could not find a valid goal in the world. Check that your occupancy map has 'Trinary' value representation and is not visually noisy/incorrect")
        return None


class RouteManager():
    '''Send goals to move_base server for the specified route. Routes forever.

       Loads the route from yaml. 
       Use RViz to record 2D nav goals. 
       Echo the input goal on topic /move_base_simple/goal

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
        'dynamic': lambda goals: GoalGenerator()
    }

    def __init__(self):
        self.route = []

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.route_mode = rospy.get_param('~mode')
        if self.route_mode not in RouteManager.route_modes:
            rospy.logerr("Route mode '%s' unknown, exiting route manager", self.route_mode)
            return

        poses = rospy.get_param('~poses', [])
        if not poses and self.route_mode!='dynamic':
            rospy.loginfo("Route manager initialized no goals, unable to route")
        
        self.goals = RouteManager.route_modes[self.route_mode](poses)
        rospy.loginfo("Route manager initialized in %s mode", self.route_mode)

        self.bad_goal_counter = 0


    def to_move_goal(self, pose):
        if pose is None:
            raise ValueError("Goal position cannot be NULL")

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position = Point(**pose['pose']['position'])
        goal.target_pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def route_forever(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.bad_goal_counter > 10:
                rospy.loginfo("Stopping route manager due to too many bad goals. Check that your occupancy map has 'Trinary' value representation and is not visually noisy/incorrect")
                return
            else:
                rospy.loginfo("Route mode is '%s', getting next goal", self.route_mode)
                try:
                    current_goal = self.to_move_goal(next(self.goals))
                except ValueError as e:
                    rospy.loginfo("No valid goal was found in the map, stopping route manager due to following exception,\n{0}".format(str(e)))
                    return

                rospy.loginfo("Sending target goal: %s", current_goal)
                self.client.send_goal(current_goal)

                try:
                    #wait 5sec for global plan to be published. If not, scan for a new goal..
                    rospy.wait_for_message("/move_base/DWAPlannerROS/global_plan", Path, timeout=5)

                    if not self.client.wait_for_result():
                        rospy.logerr("Move server not ready, will try again...")
                    elif self.client.get_result():
                            rospy.loginfo("Goal done: %s", current_goal)

                    rate.sleep()

                except rospy.exceptions.ROSException:
                    self.bad_goal_counter += 1
                    rospy.logwarn("No plan found for goal. Scanning for a new goal...")

def main():
    rospy.init_node('route_manager')
    try:
        route_manger = RouteManager()
        route_manger.route_forever()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
