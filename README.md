# AWS RoboMaker Sample Application - CloudWatch Monitoring

Monitor health and operational metrics for a fleet of robots in a simulated home using AWS CloudWatch Metrics and AWS CloudWatch Logs. Streamed metrics include speed, distance to nearest obstacle, distance to current goal, robot CPU utilization, and RAM usage.  

It demonstrates how to emit metrics and logs to AWS CloudWatch to monitor your robots.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/cloudwatch/?region=us-east-1)._


## Requirements

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) - Other versions may work, however they have not been tested
- [Colcon](https://colcon.readthedocs.io/en/released/user/installation.html) - Used for building and bundling the application. 

## AWS Setup

### AWS Credentials
You will need to create an AWS Account and configure the credentials to be able to communicate with AWS services. You may find [AWS Configuration and Credential Files](https://docs.aws.amazon.com/cli/latest/userguide/cli-config-files.html) helpful.

### AWS Permissions
To run this application you will need an IAM user with the following permissions:
```
   logs:PutLogEvents
   logs:DescribeLogStreams 
   logs:CreateLogStream 
   logs:CreateLogGroup
```
  
You can find instructions for creating a new IAM Policy [here](https://docs.aws.amazon.com/IAM/latest/UserGuide/access_policies_create.html#access_policies_create-start). In the JSON tab paste the following policy document:

```
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Sid": "CloudWatchRobotRole",
      "Effect": "Allow",
      "Action": [
        "cloudwatch:PutMetricData",
        "logs:PutLogEvents",
        "logs:DescribeLogStreams",
        "logs:CreateLogStream",
        "logs:CreateLogGroup"
      ],
      "Resource": "*"
    }
  ]
}
```

## Build

### Pre-build commands

```bash
sudo apt-get update
rosdep update
```

### Robot

```bash
sudo apt-get update
rosdep update
cd robot_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

### Simulation

```bash
cd simulation_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Run

The `TURTLEBOT3_MODEL` environment variable must be set when running the simulation application (not needed for robot application). Valid values are `burger`, `waffle`, and `waffle_pi`.

Launch the application with the following commands:

- *Running Robot Application on a Robot*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch cloudwatch_robot deploy_rotate.launch
    ```

- *Running Robot Application Elsewhere*
    ```bash
    source robot_ws/install/local_setup.sh
    roslaunch cloudwatch_robot [command]
    ```
    There are two robot launch commands:
    - `rotate.launch` - The robot starts rotating  
    - `await_commands.launch` - The robot is idle waiting movement commands, use this for teleop and navigation


- *Running Simulation Application*
    ```bash
    export TURTLEBOT3_MODEL=waffle_pi
    source simulation_ws/install/local_setup.sh
    roslaunch cloudwatch_simulation [command]
    ```
    There are three simulation launch commands for three different worlds:
    - `empty_world.launch` - Empty world with some balls surrounding the turtlebot at (0,0)
    - `bookstore_turtlebot_navigation.launch` - A retail space where the robot navigates to random goals

![CloudWatchMetrics01.png](docs/images/BookstoreRVizPlan01.png)

### Monitoring with CloudWatch Logs
Robot logs from ROS nodes are streamed into CloudWatch Logs to Log Group `robomaker_cloudwatch_monitoring_example`. See `cloudwatch_robot/config/cloudwatch_logs_config.yaml`.

### Monitoring with CloudWatch Metrics
Robot metrics from ROS nodes are reported into CloudWatch Metrics `robomaker_cloudwatch_monitoring_example`. Metric resolution is configured at 10 seconds. See `cloudwatch_robot/config/cloudwatch_metrics_config.yaml`.

Operational metrics include:
- linear speed
- angular speed
- distance to nearest obstacle (closest lidar scan return)
- distance to planned goal (bookstore only, requires its navigation system)

Health metrics include CPU and RAM usage.

![CloudWatchMetrics01.png](docs/images/CloudWatchMetrics01.png)

## Using this sample with RoboMaker

You first need to install colcon-ros-bundle. Python 3.5 or above is required. 

```bash
pip3 install colcon-ros-bundle
```

After colcon-ros-bundle is installed you need to build your robot or simulation, then you can bundle with:

```bash
# Bundling Robot Application
cd robot_ws
source install/local_setup.sh
colcon bundle

# Bundling Simulation Application
cd simulation_ws
source install/local_setup.sh
colcon bundle
```

This produces the artifacts `robot_ws/bundle/output.tar.gz` and `simulation_ws/bundle/output.tar.gz` respectively. 

You'll need to upload these to an s3 bucket, then you can use these files to 
[create a robot application](https://docs.aws.amazon.com/robomaker/create-robot-application.html),  
[create a simulation application](https://docs.aws.amazon.com/robomaker/create-simulation-application.html), 
and [create a simulation job](https://docs.aws.amazon.com/robomaker/create-simulation-job.html) in RoboMaker.

## AWS ROS Packages used by this Sample

- RoboMakerUtils-Common
- RoboMakerUtils-ROS1
- CloudWatch-Common
- CloudWatchLogs-ROS1
- CloudWatchMetrics-ROS1
- HealthMetricsCollector-ROS1
- MonitoringMessages-ROS1

## License

MIT-0 - See LICENSE.txt for further information

## How to Contribute

Create issues and pull requests against this Repository on Github


