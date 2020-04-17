# AWS RoboMaker Sample Application CI Github Action

This action will build and bundle your AWS RoboMaker Sample Application package.
It must run in an environment that has all core ROS dependencies already installed for the ROS distro you are using (Kinetic, Melodic, Dashing, etc). 

You use a [ros-core docker container], see usage section to see how to use this container. 

## Usage

Using a [ros-core docker container] docker container:

```
jobs:
  build-and-bundle-robot_ws-kinetic:
    runs-on: ubuntu-latest
    name: Build Kinetic
    container:
      image: ros:kinetic-ros-core
    steps:
    - name: Build
      uses: actions/aws-robomaker-sample-application-ci@v1
      with:
        ros-distro: kinetic
        workspace-dir: robot_ws
  build-and-bundle-simulation_ws-kinetic:
    runs-on: ubuntu-latest
    name: Build Kinetic
    container:
      image: ros:kinetic-ros-core
    steps:
    - name: Build
      uses: actions/aws-robomaker-sample-application-ci@v1
      with:
        ros-distro: kinetic
        workspace-dir: simulation_ws
```

## Inputs

### `ros-distro`

**Required** Distribution of ROS you are using (`[kinetic|melodic|dashing]`)

### `workspace-dir`

Path to the workspace folder of your package (*eg.*: `[robot_ws|simulation_ws]`, *default:* `./`). 

[setup-ros1]: https://github.com/ros-tooling/setup-ros1
[ros-core docker container]: https://hub.docker.com/_/ros/
