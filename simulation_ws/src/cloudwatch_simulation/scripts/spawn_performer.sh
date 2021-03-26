#!/bin/bash

# just print this out
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_1"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_2"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_3"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_4"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_5"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_6"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_7"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_8"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_9"'
ign service -s /world/default/level/set_performer --reqtype ignition.msgs.StringMsg --reptype ignition.msgs.Boolean --timeout 2000 --req 'data: "turtlebot3_waffle_pi_10"'

# exit gracefully by returning a status 
exit 0