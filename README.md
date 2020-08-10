# AWS RoboMaker Sample Application - CloudWatch Monitoring

Monitor health and operational metrics for a fleet of robots in a simulated home using AWS CloudWatch Metrics and AWS CloudWatch Logs. Streamed metrics include speed, distance to nearest obstacle, distance to current goal, robot CPU utilization, and RAM usage.

It demonstrates how to emit metrics and logs to AWS CloudWatch to monitor your robots.

_RoboMaker sample applications include third-party software licensed under open-source licenses and is provided for demonstration purposes only. Incorporation or use of RoboMaker sample applications in connection with your production workloads or a commercial products or devices may affect your legal rights or obligations under the applicable open-source licenses. Source code information can be found [here](https://s3.console.aws.amazon.com/s3/buckets/robomaker-applications-us-east-1-72fc243f9355/cloudwatch/?region=us-east-1)._


## Requirements

- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) / [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - Other versions may work, however they have not been tested
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

### Install requirements
Follow links above for instructions on installing required software.

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

This produces the artifacts `robot_ws/bundle/output.tar` and `simulation_ws/bundle/output.tar` respectively.

You'll need to upload these to an s3 bucket, then you can use these files to
[create a robot application](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot-application.html),
[create a simulation application](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html),
and [create a simulation job](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html) in RoboMaker.


## Generate map for your world

Procedurally generate an occupancy map for any gazebo world. This map can then be plugged into your navigation stack to navigate a robot in your world. 

Note: This is an OPTIONAL functionality and is NOT required for using any functionalities listed above.

### Pre-build

```bash
# Install dependencies (Ubuntu >18.04)
sudo apt-get install ruby-dev libxml-xpath-perl libxml2-utils
```

```bash
# Fetch and install ROS dependencies
cd simulation_ws
rosws update
rosdep install --from-paths src -r -y
cd ..
```

For Worldforge worlds,
```bash
# Extract and copy Worldforge exported worlds
unzip exported_world.zip
cp -r ros_ws/src/aws_robomaker_worldforge_* simulation_ws/src/
```

### Generate Occupancy Map via map generation plugin

```bash
#For worldforge worlds, set WORLD_ID to the name of your WF exported world (eg: generation_40r67s111n9x_world_3),
export WORLD_ID=<worldforge-world-name>

# Add map generation plugin to a robomaker world
python scripts/add_map_plugin.py default --world_name <world_name>
```
world_name can be:  
    - [`bookstore`](https://github.com/aws-robotics/aws-robomaker-bookstore-world)  
    - [`small_house`](https://github.com/aws-robotics/aws-robomaker-small-house-world)  
    - [`small_warehouse`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)  
    - [`no_roof_small_warehouse`](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)
    - worldforge

```bash
# Alternatively for your custom world/config,
python scripts/add_map_plugin.py custom -c <path-to-config> -w <path-to-world> -o <output-path>
```

```bash
# Build with plugin added
cd simulation_ws
colcon build
source install/local_setup.sh

# Start map service (for custom worlds, relocate your world file with the added plugin to src/cloudwatch_simulation/worlds/map_plugin.world before running this)
roslaunch cloudwatch_simulation start_map_service.launch

# Generate map (start in a different terminal AFTER you see "[INFO] [*] occupancy map plugin started" message in previous terminal)
rosservice call /gazebo_2Dmap_plugin/generate_map

# Save map
rosrun map_server map_saver -f <file-name> /map:=/map2d
```

## AWS ROS Packages used by this Sample

- RoboMakerUtils-Common
- RoboMakerUtils-ROS1
- CloudWatch-Common
- CloudWatchLogs-ROS1
- CloudWatchMetrics-ROS1
- HealthMetricsCollector-ROS1
- MonitoringMessages-ROS1

## License

MIT-0 - See LICENSE for further information

## How to Contribute

Create issues and pull requests against this Repository on Github
