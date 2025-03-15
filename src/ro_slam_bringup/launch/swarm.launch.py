"""
Swarm launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

October 29, 2024
"""

import os, sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import sys
sys.path.append("/home/neo/workspace/")
from update_yaml import update_params

def generate_launch_description():
    ld = LaunchDescription()

    n_robots = 6

    # Check if user specified the number of robots
    for arg in sys.argv:
        if arg.startswith("n_robots:="):
            n_robots = int(arg.split(":=")[1])

    launch_file_path = os.path.join(
        get_package_share_directory('ro_slam_py'), 'launch', 'ro_slam_py.launch.py')
    config_file_path_alias = os.path.join(
        get_package_share_directory('ro_slam_bringup'), 'config', 'swarm_alias.yaml')

    config_file_path = update_params(config_file_path_alias)

    # Launch the robots
    for robot_id in range(1, n_robots+1):
        namespace = f'robot{robot_id}'
        robot_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={'namespace': namespace,
                              'config_file' : config_file_path}.items()
        )
        ld.add_action(robot_launch_file)

    return ld
