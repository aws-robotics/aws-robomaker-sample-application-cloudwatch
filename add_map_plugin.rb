#!/usr/bin/ruby

require 'nokogiri'

world_file = String.new("simulation_ws/src/deps/aws-robomaker-bookstore-world/worlds/bookstore.world")

doc = File.open(world_file) { |f| Nokogiri::XML(f) }
_world_ptr = doc.at('world')

plugin_doc = Nokogiri::XML("<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.1</map_resolution> <!-- in meters, optional, default 0.1 -->
    <map_height>0.3</map_height>         <!-- in meters, optional, default 0.3 -->
    <map_size_x>10</map_size_x>          <!-- in meters, optional, default 10 -->
    <map_size_y>10</map_size_y>          <!-- in meters, optional, default 10 -->
</plugin>
")

_world_ptr.add_child(plugin_doc.at('plugin'))

dump_file = String.new("simulation_ws/src/deps/aws-robomaker-bookstore-world/worlds/bookstore_map_plugin.world") 

File.write(dump_file, doc.to_xml)
# puts doc.to_xml