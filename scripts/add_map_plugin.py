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

import sys
import os
import subprocess

def get_args():
	'''
	Generates arguments for map plugin tool.
	'''
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
	plugin_tool_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "genmap.sh")
	cmd = ' '.join([plugin_tool_path] + args)
	try:
		out = subprocess.check_output(cmd, shell=True)
	except subprocess.CalledProcessError:
		sys.exit(2)

if __name__=="__main__":
	main()
