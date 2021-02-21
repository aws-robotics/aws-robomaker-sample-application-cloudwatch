# AWS RoboMaker ROS Message Service

This package contains ROS service definitions for service endpoints provided inside of an AWS RoboMaker simulation.

**Visit the [AWS RoboMaker website](https://aws.amazon.com/robomaker/) to learn more about building intelligent robotic applications with Amazon Web Services.**

## Install

You will need the source code in your workspace. 

1. Add a .rosinstall file with the contents below into your workspace folder then run `rosws update`:
```
- git: {local-name: src/aws-robomaker-simulation-ros-pkgs, uri: 'https://github.com/aws-robotics/aws-robomaker-simulation-ros-pkgs.git', version: master}
```

2. Add a dependency to your package.xml to access the service types (.srv):
```
<package>
   ...
   <depend>aws_robomaker_simulation_ros_pkgs</depend>
</package>
```

3. Follow the Python usage below and then build and bundle your workspace as per usual:
```bash
# build for ROS
rosws update
rosdep install --from-paths . --ignore-src -r -y
colcon build
colcon bundle
```

## Usage

You can tag, untag, and list tags in your simulation job from the ROS command-line or in your ROS application while it is running. See [AWS RoboMaker documentation](https://docs.aws.amazon.com/robomaker/latest/dg/simulation-job-tags.html). You must have an IAM role with the permissions below. Replace account# with your account number.

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "robomaker:TagResource",
                "robomaker:UntagResource",
                "robomaker:ListTagsForResource"
            ],
            "Resource": [
                "arn:aws:robomaker:*:account#:simulation-job/*"
            ],
            "Effect": "Allow"
        }
    ]
}
```

### AddTags

#### Python

```python
import rospy
from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import AddTags

def add_tags_to_my_sim():
    add_tags([Tag(key="name", value="my_test")])

def add_tags(tags):
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/add_tags', timeout=timeoutInSeconds)
    requestAddTags = rospy.ServiceProxy('/robomaker/job/add_tags', AddTags)
    response = requestAddTags(tags)
    if not response.success:
        # AddTags failed
        print response.message

```

### RemoveTag

#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import RemoveTags

def remove_tags_from_my_sim():
    remove_tags([Tag(key="name", value="my_test")])

def remove_tags(tags):
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/remove_tags', timeout=timeoutInSeconds)
    requestRemoveTags = rospy.ServiceProxy('/robomaker/job/remove_tags', RemoveTags)
    response = requestRemoveTags(tags)
    if not response.success:
        # RemoveTags failed
        print response.message

```

### ListTags

#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import ListTags

def list_tags():
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/list_tags', timeout=timeoutInSeconds)
    requestListTags = rospy.ServiceProxy('/robomaker/job/list_tags', ListTags)
    response = requestListTags()
    if response.success:
        # ListTags succeeded
        print response.tags
    else:
        # ListTags failed
        print response.message

```

### CancelSimulation

You can cancel your simulation job from the ROS command-line or in your ROS application while it is running. See [AWS RoboMaker documentation](https://docs.aws.amazon.com/robomaker/latest/dg/simulation-job-playback-rosbags.html#simulation-job-playback-rosbags-cancel). You must have an IAM role with the permissions below. Replace account# with your account number. 

```
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": [
                "robomaker:CancelSimulationJob"
            ],
            "Resource": [
                "arn:aws:robomaker:*:account#:simulation-job/*"
            ],
            "Effect": "Allow"
        }
    ]
}
```


#### Python

```python
import rospy
from robomaker_simulation_msgs.srv import Cancel

def cancel_job():
    timeoutInSeconds = 60
    rospy.wait_for_service('/robomaker/job/cancel', timeout=timeoutInSeconds)
    requestCancel = rospy.ServiceProxy('/robomaker/job/cancel', Cancel)
    response = requestCancel()
    if not response.success:
        # Request cancel job failed
        print response.message
```

## License

This library is licensed under the Apache 2.0 License. 
