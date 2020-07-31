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
import argparse
import json


def process_args(args, default_args):
    '''
    Processes arguments for map plugin tool.
    '''

    # default arguments
    if args.world_name is not None:
        world_name = args.world_name
        config_file = default_args[world_name][0]
        world_file = "simulation_ws/src/deps/{0}/worlds/{1}.world".format(default_args[world_name][1], world_name)
        output_file = "simulation_ws/src/cloudwatch_simulation/worlds/map_plugin.world"

    #custom world file, config file, output file
    else:
        config_file, world_file, output_file = args.config_file, args.world_file, args.output_file
    
    p_args = [os.path.abspath(x) for x in [config_file, world_file, output_file]]
    
    return p_args


def main():

    default_arg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, 'map_config/default_args.json')
    with open(default_arg_path, 'r') as f:
        default_args = json.load(f)

    parser = argparse.ArgumentParser(description="Arguments for default/custom usage of map_plugin tool.")

    parser.add_argument("--world_name", help="takes a default world_name, each referring to an existing aws-robotics worlds", choices=list(default_args.keys()), type=str)
    parser.add_argument("-c", "--config_file", help="config file (.rb) for the map plugin parameters", type=str)
    parser.add_argument("-w", "--world_file", help="path to the original world file", type=str)
    parser.add_argument("-o", "--output_file", help="output path of the new world file", type=str)

    if len(sys.argv)==1:
        print("Either provide [--world_name] OR [-c, -w and -o] arguments.\n")
        parser.print_help()
        parser.exit()

    args = process_args(parser.parse_args(), default_args)

    plugin_tool_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "genmap.sh")
    cmd = ' '.join([plugin_tool_path] + args)
    try:
        out = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as e:
        print("Execution failed with exitcode {0}\n\n{1}".format(e.returncode, e.output))
        sys.exit(e.returncode)
    else:
        print("{}\n".format(out))

if __name__=="__main__":
    main()
