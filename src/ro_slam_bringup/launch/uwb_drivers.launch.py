"""
Swarm launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

October 29, 2024
"""

import os, sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('ro_slam_bringup'), 'config', 'uwb_drivers.yaml')

    robot_id = os.getenv('ROBOT_ID')

    if robot_id == '1':
        node1 = Node(
            package='uwb_driver',
            executable='uwb_driver',
            namespace='robot1/M1',
            shell=False,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[config],
        )
        ld.add_action(node1)

        node2 = Node(
            package='uwb_driver',
            executable='uwb_driver',
            namespace='robot1/M2',
            shell=False,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[config],
        )
        ld.add_action(node2)

    elif robot_id == '2':
        node1 = Node(
            package='uwb_driver',
            executable='uwb_driver',
            namespace='robot2/M1',
            shell=False,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[config],
        )
        ld.add_action(node1)

        node2 = Node(
            package='uwb_driver',
            executable='uwb_driver',
            namespace='robot2/M3',
            shell=False,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[config],
        )
        ld.add_action(node2)

    else:
        node1 = Node(
            package='uwb_driver',
            executable='uwb_driver',
            namespace='robot3/M1',
            shell=False,
            emulate_tty=True,
            output='both',
            log_cmd=True,
            parameters=[config],
        )
        ld.add_action(node1)

    return ld
