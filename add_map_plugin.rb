#!/usr/bin/ruby

require 'nokogiri'

if ARGV.empty?
  puts "Please provide a world name, eg: ruby add_map_plugin.rb bookstore"
  exit
end

world_ID = ARGV[0]
robot_init_x = 0.0
robot_init_y = 0.0

if world_ID=="bookstore"
	world_file = String.new("simulation_ws/src/deps/aws-robomaker-bookstore-world/worlds/bookstore.world")
elsif world_ID=="smallhouse"
	world_file = String.new("simulation_ws/src/deps/aws-robomaker-small-house-world/worlds/small_house.world")
	robot_init_x = 3.0
	robot_init_y = 3.0
end

doc = File.open(world_file) { |f| Nokogiri::XML(f) }
_world_ptr = doc.at('world')

plugin_doc = Nokogiri::XML("<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.05</map_resolution> <!-- in meters, optional, default 0.1 -->
    <map_height>0.3</map_height>         <!-- in meters, optional, default 0.3 -->
    <map_size_x>25</map_size_x>          <!-- in meters, optional, default 25 -->
    <map_size_y>25</map_size_y>          <!-- in meters, optional, default 25 -->
    <init_robot_x>#{robot_init_x}</init_robot_x>          <!-- in meters, optional, default 0. -->
    <init_robot_y>#{robot_init_y}</init_robot_y>          <!-- in meters, optional, default 0. -->
</plugin>
")

_world_ptr.add_child(plugin_doc.at('plugin'))

if world_ID=="bookstore"
	dump_file = String.new("simulation_ws/src/deps/aws-robomaker-bookstore-world/worlds/bookstore_map_plugin.world") 
elsif world_ID=="smallhouse"
	dump_file = String.new("simulation_ws/src/deps/aws-robomaker-small-house-world/worlds/small_house_map_plugin.world") 
end

File.write(dump_file, doc.to_xml)
puts "plugin added successfully to " + world_file
puts "new world file saved at " + dump_file + "\n"