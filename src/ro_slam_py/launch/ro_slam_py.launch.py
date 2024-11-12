"""
Single robot launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

October 29, 2024
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Builds a LaunchDescription for the ROSlam app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('ro_slam_py'), 'config', 'ro_slam_py.yaml')

    # Declare launch arguments
    config_file = LaunchConfiguration('config_file')
    config_file_launch_arg = DeclareLaunchArgument('config_file', default_value=config)
    ld.add_action(config_file_launch_arg)

    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument('namespace', default_value='robot1')
    ld.add_action(namespace_launch_arg)

    # Create node launch description
    node = Node(
        package='ro_slam_py',
        executable='ro_slam_py',
        namespace=namespace,
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config_file],
        remappings=[('/joint_states', ['/', namespace, '/joint_states']),
                    ('/uwb_tag', ['/', namespace, '/uwb_tag']),
                    ('/debug', ['/', namespace, '/debug']),
                    ('/visualization/landmarks', ['/', namespace, '/visualization/landmarks'])],
    )
    ld.add_action(node)

    return ld
