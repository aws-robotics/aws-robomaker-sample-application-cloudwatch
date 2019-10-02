#!/bin/sh
# This is a script which can be run after launching the ROS processes.
# You can include a post-check of ROS processes in this post-launch file. 
# Non-zero exit status from script would cause robot deployment failure. 
# Use "deploymentScripts/post_launch_file.sh" as the postLaunchFile path.

# Wait all monitoring nodes starts.
sleep 30
# ping and verify nodes started
set -e
rosnode ping -c 3 monitor_speed
rosnode ping -c 3 monitor_obstacle_distance
rosnode ping -c 3 monitor_distance_to_goal
rosnode ping -c 3 cloudwatch_logger
rosnode ping -c 3 cloudwatch_metrics_collector
rosnode ping -c 3 health_metric_collector
