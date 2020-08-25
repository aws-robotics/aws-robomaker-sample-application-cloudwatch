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

# This script adds map generation plugin to the input world and plugin config parameters

# Input args via command line:
#   <path-to-config>  [required] 
#   <path-to-world-file>  [required]
#   <path-to-output-file>  [required]

set -e

if [ $# -ne 3 ]; then
    echo "expects 3 arguments"
    exit 2
fi

worldfile=$2
config=$1
worldbody=`xpath -q -e '/sdf/world/*' $worldfile`
template="""
<%
%>
<sdf version='1.6'>
  <world name='default'>
    $worldbody
    <plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
         <map_resolution><%= @map_res %></map_resolution>
         <map_height>0.3</map_height>
         <map_size_x><%= @map_size_x %></map_size_x>
         <map_size_y><%= @map_size_y %></map_size_y>
         <init_robot_x><%= @robot_init_x %></init_robot_x>
         <init_robot_y><%= @robot_init_y %></init_robot_y>
     </plugin>
 </world>
</sdf>
"""
echo $template | erb -r "$config"  | xmllint --format - > $3
echo $3
