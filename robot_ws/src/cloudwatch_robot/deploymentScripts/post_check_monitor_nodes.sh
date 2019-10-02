#!/bin/sh
# This is a script which can be run after launching the ROS processes.
# You can include a post-check of ROS processes in this post-launch file. 
# Non-zero exit status from script would cause robot deployment failure. 
# Use "deploymentScripts/post_launch_file.sh" as the postLaunchFile path.

# Wait all monitoring nodes starts.
sleep 30
# ping and verify nodes started
set -e
ros2 node info /monitor_speed
ros2 node info /monitor_obstacle_distance
ros2 node info /monitor_distance_to_goal
ros2 node info /cloudwatch_logger
ros2 node info /cloudwatch_metrics_collector
ros2 node info /health_metric_collector
