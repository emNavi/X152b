#!/usr/bin/env python
import sys
import yaml
import getpass
import os

param_file="{}/../src/autonomous_drone_sdk/global_interface/config".format(os.path.dirname(os.path.abspath(__file__)))
with open(param_file+"/drone_param.yaml", "r") as stream:
    try:
        dictionary = yaml.safe_load(stream)
        for key, value in dictionary.items():
            if(str(sys.argv[1]) == key):
                # print (key + " : " + str(value))
                print(str(value))
                sys.exit(0)
        
    except yaml.YAMLError as exc:
        print(exc)
sys.exit(1)
