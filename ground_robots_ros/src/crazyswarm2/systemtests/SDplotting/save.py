# -*- coding: utf-8 -*-
"""
Tool for saving parameters from the crazyflie.yaml before every experiment.
"""
import os
import yaml


def write_info(experiment=None, ros2_ws_path=None):
    # Dictionary to store extracted information
    info = {}

    # Prompt the user for the experiment number
    # experiment_number = int(input("Enter the experiment number: "))

    # File paths
    crazyflies_config_path = "../crazyflie/config/crazyflies.yaml"
    if ros2_ws_path != None:
        crazyflies_config_path = str(ros2_ws_path) + "/src/crazyswarm2/crazyflie/config/crazyflies.yaml"
        info_file_prepath = str(ros2_ws_path) + "/src/crazyswarm2/systemtests"
        

    # Read information from crazyflies.yaml
    try:
        with open(crazyflies_config_path, "r") as config_file:
            config_data = yaml.safe_load(config_file)
            # extract the trajectory and timescale
            obj = config_data["all"]["firmware_params"]
            if "ctrlLeeInfo" in obj:
                for key, value in obj["ctrlLeeInfo"].items():
                    info[key] = value
            # extract the ctrlLee parameters
            if "ctrlLee" in obj:
                for key, value in obj["ctrlLee"].items():
                    info[key] = value
    except FileNotFoundError:
        print(f"(save.py) File not found: {crazyflies_config_path}")
        exit(1)


    if experiment != None:
        info["experiment"] = experiment
        info["trajectory"] = str(experiment) + ".csv"
        
    experiment_name = info["experiment"]
    info_file_path = info_file_prepath + f"/SDplotting/info/info_{experiment_name}.yaml"
    # info_file_path = str(ros2_ws_path) + f"systemtests/SDplotting/info/info{experiment_name}.yaml"
    
    # file guard
    if os.path.exists(info_file_path):
        print(f"File already exists: {info_file_path}  It will be replaced")
        # ans = input("Overwrite? [y/n]: ")
        # if ans == "n":
        #     print("Exiting...")
        #     exit(1)

    print("========================================")

    try:
        with open(info_file_path, "w") as info_file:
            print(f"Writing experiment info to {info_file_path}")
            for key in info:
                print(f">>> {key}: {info[key]}")
                yaml.dump({key: info[key]}, info_file, default_flow_style=False, sort_keys=False)
        print(f"Experiment info written to {info_file_path}")
    except Exception as e:
        print(f"Error writing to {info_file_path}: {str(e)}")

    print("========================================")

if __name__ == "__main__":
    write_info("multi_trajectory", "/home/jthevenoz/ros2_ws")