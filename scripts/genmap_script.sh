#!/usr/bin/env bash

# -------------------------------------------------------------------------

# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -------------------------------------------------------------------------

# This script generates map files for the input world

# Input args via command line:
#   <world-name>  [required] 

set -e

if [ $# -ne 1 ]; then
    echo "Expect 1 argument";
    exit 2;
fi
        
if [ $1 = "worldforge" ] && [ -z "$WORLD_ID" ]; then
    echo "For WorldForge world, please set WORLD_ID to your worldforge world as per the README instructions"
    exit 2;
fi

echo "Sudo password may be needed to install system dependencies"
sudo apt-get install ruby-dev libxml-xpath-perl libxml2-utils

cd simulation_ws
rosws update
rosdep install --from-paths src --ignore-src -r -y
cd ..

set +e

OUTPUT=`python scripts/add_map_plugin.py default --world_name $1`

if [ $? -eq 2 ]
then
	echo $OUTPUT
	exit $?
else
	world_source_path=$OUTPUT
fi

set -e

cd simulation_ws
colcon build
source install/local_setup.sh
cd ..

map_output_path=$(dirname $(dirname $world_source_path))/maps/map

roslaunch cloudwatch_simulation start_map_service.launch &

python << END
import rospy

rospy.wait_for_service('/gazebo_2Dmap_plugin/generate_map')
END

rosservice call /gazebo_2Dmap_plugin/generate_map
rosrun map_server map_saver -f $map_output_path /map:=/map2d

cd simulation_ws
colcon build

kill $!

echo "--- Map file generated at $map_output_path"
