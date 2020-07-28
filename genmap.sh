#!/usr/bin/env bash

worldfile=$(python map_config/get_pkg_path.py $2)
config=`basename $1`
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
echo $template | erb -r "./map_config/$config"  | xmllint --format - > "simulation_ws/src/cloudwatch_simulation/worlds/map_plugin.world"