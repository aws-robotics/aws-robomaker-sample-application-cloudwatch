#!/bin/bash

ament_flake8 ../simulation_ws/src/aws_robomaker_simulation_common ../simulation_ws/src/cloudwatch_simulation ../robot_ws/src/cloudwatch_robot
ament_pep257 ../simulation_ws/src/aws_robomaker_simulation_common ../simulation_ws/src/cloudwatch_simulation ../robot_ws/src/cloudwatch_robot
ament_xmllint ../simulation_ws/src/aws_robomaker_simulation_common ../simulation_ws/src/cloudwatch_simulation ../robot_ws/src/cloudwatch_robot 