import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Builds a LaunchDescription for the MeanVar app"""
    ld = LaunchDescription()

    # Create node launch description
    node = Node(
        package='mean_var',
        executable='mean_var_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        remappings=[('/uwb', '/robot0/uwb_tag')],
    )

    ld.add_action(node)

    return ld
