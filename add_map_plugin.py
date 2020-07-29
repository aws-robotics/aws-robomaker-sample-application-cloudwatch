import sys
import os
import subprocess

def get_args():
	# default arguments
	if len(sys.argv) == 2:
		default_args = {'bookstore':["./map_config/bookstore.rb", "aws-robomaker-bookstore-world"],\
				'small_house':["./map_config/small_house.rb", "aws-robomaker-small-house-world"],\
				'small_warehouse':["./map_config/small_warehouse.rb","aws-robomaker-small-warehouse-world"],\
				'no_roof_small_warehouse':["./map_config/no_roof_small_warehouse.rb","aws-robomaker-small-warehouse-world"]
				}
		try:
			world_name = sys.argv[1]
			config_file = default_args[world_name][0]
			world_file = "simulation_ws/src/deps/{0}/worlds/{1}.world".format(default_args[world_name][1], world_name)
			output_file = "simulation_ws/src/cloudwatch_simulation/worlds/map_plugin.world"
		except KeyError:
			raise ValueError("Invalid argument")

	#custom world file, config file, output file
	elif len(sys.argv) == 4:
		config_file, world_file, output_file = sys.argv[1:]
	
	else:
		raise ValueError("Expects 1 or 3 number of arguments")

	args = list(map(lambda x: os.path.abspath(x), [config_file, world_file, output_file]))
	
	return args

def main():
	args = get_args()
	cmd = './scripts/genmap.sh '+' '.join(args)
	try:
		out = subprocess.check_output(cmd, shell=True)
	except subprocess.CalledProcessError:
		sys.exit(2)

if __name__=="__main__":
	main()
