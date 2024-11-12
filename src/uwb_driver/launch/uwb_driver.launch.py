"""
UWB driver launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

February 02, 2022
"""
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Builds a LaunchDescription for the UWB driver app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('uwb_driver'), 'config', 'uwb_driver.yaml')

    # Create node launch description
    node = Node(
        package='uwb_driver',
        executable='uwb_driver',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config],
    )
    ld.add_action(node)

    return ld