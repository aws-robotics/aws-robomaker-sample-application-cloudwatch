# AWS RoboMaker Simulations Common utilities and helpers

## Route Manager
Node that sends a route of target goals to the robot. Sends to the `NavigateToPose` action server.

Includes in your .launch.py:
```
from ament_index_python.packages import get_package_share_directory

launch_ros.actions.Node(
    package='aws_robomaker_simulation_common',
    node_executable='route_manager',
    node_name='route_manager',
    output='screen',
    parameters=[{
        'route_file': get_package_share_directory(<your_package>) + '/routes/route.yaml'
    }]
)
```
