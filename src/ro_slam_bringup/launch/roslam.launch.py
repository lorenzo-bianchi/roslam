"""
ROSlam launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

December 11, 2024
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    ld = LaunchDescription()

    launch_uwb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/uwb_drivers.launch.py'])
    )
    ld.add_action(launch_uwb)

    launch_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_bringup'), 'launch', 'roslam_robot.launch')
        )
    )
    ld.add_action(launch_turtlebot)

    return ld
