"""
Joystick app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

October 26, 2024
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
  ld = LaunchDescription()

  container = ComposableNodeContainer(
    name='joy_container_robot3',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    emulate_tty=True,
    output='both',
    log_cmd=True,
    composable_node_descriptions=[
      ComposableNode(
        package='joystick',
        plugin='joystick::JoystickNode',
        name='joystick',
        namespace='',
        parameters=[{
          'axis_deadzone_val': 300,
          'axis_max_val': 32767.0,
          'invert_y': True,
          'joy_topic_name': '/robot3/cmd_joystick',
          'joy_path': '/dev/input/js2'
          }],
        remappings=[]),
      ComposableNode(
        package='joy2cmdvel',
        plugin='Joy2cmdvel::Joy2cmdvelNode',
        name='joy2cmdvel',
        namespace='',
        parameters=[{
          'cmd_vel_topic_name': '/robot3/cmd_vel',
          'joy_topic_name': '/robot3/cmd_joystick',
          'max_ang_vel': 2.0,
          'max_lin_vel': 0.22}],
        remappings=[])])
  ld.add_action(container)

  return ld
