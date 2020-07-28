import sys
world_name = sys.argv[1]
world_pkg = {'bookstore':"aws-robomaker-bookstore-world",\
			'small_house':"aws-robomaker-small-house-world",\
			'small_warehouse':"aws-robomaker-small-warehouse-world",\
			'no_roof_small_warehouse':"aws-robomaker-small-warehouse-world"
			}
			
world_path = "simulation_ws/src/deps/{0}/worlds/{1}.world".format(world_pkg[world_name], world_name)

print(world_path)