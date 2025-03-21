"""
Joystick app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

October 26, 2024
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    """Builds a LaunchDescription for the Joystick app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(get_package_share_directory('joystick'), 'config', 'joystick.yaml')

    # Declare launch arguments
    cf = LaunchConfiguration('cf')
    cf_launch_arg = DeclareLaunchArgument('cf', default_value=config)
    ld.add_action(cf_launch_arg)

    use_gdbserver = LaunchConfiguration('use_gdbserver')
    use_gdbserver_launch_arg = DeclareLaunchArgument('use_gdbserver', default_value='False')
    ld.add_action(use_gdbserver_launch_arg)

    # Conditional prefix for gdbserver
    gdb_prefix = PythonExpression([
        "'gdbserver localhost:5000' if '", use_gdbserver, "' == 'True' else ''"])
    print(gdb_prefix)

    # Create node launch description
    node = Node(
        package='joystick',
        executable='joystick_app',
        shell=False,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf],
        prefix=gdb_prefix,
        remappings=[],
    )
    ld.add_action(node)

    return ld

