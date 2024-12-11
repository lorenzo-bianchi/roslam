import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def modify_sdf_namespace(sdf_file, data):
  with open(sdf_file, 'r') as file:
    sdf_content = file.read()

  robot_name = data['robot_name']
  color = data['color']
  inter_robot_distances = data['inter_robot_distances']
  prob_loss_measurement = data['prob_loss_measurement']
  mean = data['mean']
  sigma = data['sigma']

  sdf_content = sdf_content.replace('{namespace}', robot_name)
  sdf_content = sdf_content.replace('1.1 1.1 1.1', color)
  sdf_content = sdf_content.replace('{inter_robot_distances}', inter_robot_distances)
  sdf_content = sdf_content.replace('{prob_loss_measurement}', prob_loss_measurement)
  sdf_content = sdf_content.replace('{mean}', mean)
  sdf_content = sdf_content.replace('{sigma}', sigma)

  temp_sdf_file = f'/tmp/{robot_name}_model.sdf'
  with open(temp_sdf_file, 'w') as file:
    file.write(sdf_content)

  return temp_sdf_file

def generate_launch_description():
  ld = LaunchDescription()

  # Set env vars
  gz_resource_path = SetEnvironmentVariable(
    name='IGN_GAZEBO_RESOURCE_PATH',value=[
      os.path.join('/opt/ros/humble', 'share'),
      ':' +
      os.path.join(get_package_share_directory('ro_slam_descriptions'), 'models')])
  ld.add_action(gz_resource_path)

  # Set launch arguments
  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  use_sim_time_declare = DeclareLaunchArgument(
    'use_sim_time',
    default_value=use_sim_time,
    description='If true, use simulated clock')
  ld.add_action(use_sim_time_declare)

  params = {'use_sim_time': use_sim_time}

  # Spawn entities
  parser = Node(
    package='uwb_parser',
    executable='uwb_parser',
    output='screen',
    parameters=[params]
    )
  ld.add_action(parser)

  robots_pos = [[-2.0, -0.5, 0.01, 0.0],
                [ 2.0,  0.5, 0.01, 0.0],
                [ 0.0, -1.0, 0.01, 1.57],]
  colors = ['0.7 0.0 0.0',
            '0.0 0.0 0.7',
            '0.0 0.7 0.0']
  for i, pos in enumerate(robots_pos):
    robot_name = f'robot{i+1}'

    tf_map_robot = Node(
      package='tf2_ros',
      executable='static_transform_publisher',
      name='static_transform_publisher',
      output='log',
      parameters=[params],
      arguments=[str(pos[0]), str(pos[1]), str(pos[2]), str(pos[3]), '0.0', '0.0', 'map', f'{robot_name}/odom'])
    ld.add_action(tf_map_robot)

    data = {}
    data['robot_name'] = robot_name
    data['color'] = colors[i]
    data['inter_robot_distances'] = 'true'
    data['prob_loss_measurement'] = '0.05'
    data['mean'] = '0.0'
    data['sigma'] = '0.01'
    sdf_file = os.path.join(get_package_share_directory('ro_slam_descriptions'), 'models', 'robot', 'model.sdf')
    modified_sdf_file = modify_sdf_namespace(sdf_file, data)

    create_robot = Node(
      package='ros_gz_sim',
      executable='create',
      name=robot_name,
      output='screen',
      namespace=robot_name,
      parameters=[params],
      arguments=['-name', robot_name,
                 '-file', modified_sdf_file,
                 '-allow_renaming', 'true',
                 '-x', str(pos[0]),
                 '-y', str(pos[1]),
                 '-z', str(pos[2]),
                 '-Y', str(pos[3])],
      )
    ld.add_action(create_robot)

    urdf = os.path.join(get_package_share_directory('ro_slam_descriptions'), 'urdf', 'robot.urdf')
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    robot_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      namespace=robot_name,
      output='screen',
      parameters=[{'use_sim_time': use_sim_time,
                   'frame_prefix': f'{robot_name}/',
                   'robot_description': doc.toxml()}])
    ld.add_action(robot_state_publisher)

  ### Bridge ###
  bridge_args = [
    # Clock (IGN -> ROS2)
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    # String for UWB data (IGN -> ROS2)
    '/uwb_raw_data@std_msgs/msg/String[ignition.msgs.StringMsg']

  for i in range(len(robots_pos)):
    robot_name = f'robot{i+1}'
    bridge_args += [
      # Velocity command (ROS2 -> IGN)
      f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
      # Ground truth (IGN -> ROS2)
      f'/{robot_name}/ground_truth@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
      # Odometry (IGN -> ROS2)
      f'/{robot_name}/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
      # TF (IGN -> ROS2)
      f'/odom/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
      # Joint states (IGN -> ROS2)
      f'/{robot_name}/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
      # IMU (IGN -> ROS2)
      # f'/{robot_name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    ]
  bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=bridge_args,
    remappings=[
        ('/odom/tf', 'tf'),
    ],
    output='screen'
  )
  ld.add_action(bridge)

  ### Gazebo ###
  world_only = os.path.join(get_package_share_directory('ro_slam_descriptions'), 'models', 'worlds', 'world_only_ign.sdf')
  gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
    launch_arguments=[('gz_args', f' -r {world_only} -v3 ')])
  ld.add_action(gazebo_launch)

  gz_spawn_walls = Node(
    package='ros_gz_sim',
    executable='create',
    output='screen',
    parameters=[params],
    arguments=['-file', PathJoinSubstitution([
                  get_package_share_directory('ro_slam_descriptions'),
                  'models',
                  'wall',
                  'model.sdf']),
               '-allow_renaming', 'false'],
    )
  ld.add_action(gz_spawn_walls)

  anchors = [[ 1.0,  1.0, 0.21],
             [ 1.0, -1.0, 0.21],
             [-1.0,  1.0, 0.21],
             [-1.0, -1.0, 0.21]]
  for i, anchor in enumerate(anchors):
    gz_spawn_anchor = Node(
      package='ros_gz_sim',
      executable='create',
      output='screen',
      parameters=[params],
      arguments=['-name', f'anchor_{i}',
                 '-file', PathJoinSubstitution([
                    get_package_share_directory('ro_slam_descriptions'),
                    'models',
                    'uwb_anchor',
                    'model.sdf']),
                 '-allow_renaming', 'true',
                 '-x', str(anchor[0]),
                 '-y', str(anchor[1]),
                 '-z', str(anchor[2])],
      )
    ld.add_action(gz_spawn_anchor)

  return ld
