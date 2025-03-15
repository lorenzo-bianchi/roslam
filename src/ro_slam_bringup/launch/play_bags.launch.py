"""
Play bags launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

February 22, 2025
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Topics to play and remap
topics = [
    # "/cmd_vel",
    # "/imu",
    "/joint_states",
    "/odom",
    "/robot_description",
    "/uwb_tag"
]

bags = [
    {"path": "logs/20241220_test1",        "robot_from": 1, "robot_to": 1, "initial_pose": (2.0, 2.0, 0.0)},
    {"path": "logs/20241220_robot2_test4", "robot_from": 2, "robot_to": 2, "initial_pose": (1.0, 0.5, 0.4)},
    {"path": "logs/20241220_robot3_test7", "robot_from": 3, "robot_to": 3, "initial_pose": (-0.5, -1.0, -0.6)},
    {"path": "logs/20241220_robot3_test9", "robot_from": 3, "robot_to": 4, "initial_pose": (-0.5, -1.0, -0.6)},
    {"path": "logs/20241220_test3",        "robot_from": 1, "robot_to": 5, "initial_pose": (-0.5, -1.0, -0.6)},
    {"path": "logs/20241220_robot2_test6", "robot_from": 2, "robot_to": 6, "initial_pose": (-0.5, -1.0, -0.6)},
]

def generate_launch_description():
    ld = LaunchDescription()

    for bag in bags:
        remaps = [f"/robot{bag['robot_from']}{t}:=/robot{bag['robot_to']}{t}" for t in topics]
        remap_str = " ".join(remaps)

        topics_str = " ".join([f"/robot{bag['robot_from']}{t}" for t in topics])

        process = ExecuteProcess(
            cmd=["ros2", "bag", "play", "--rate", "1.0", bag["path"], "--remap"] +
            remap_str.split() +
            ["--topics"] + topics_str.split(),
            output="screen"
        )
        ld.add_action(process)

        # Static transform from /map to /robotX/odom
        x0, y0, theta0 = bag["initial_pose"]
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                str(x0), str(y0), "0", str(theta0), "0", "0", "/map", f"/robot{bag['robot_to']}/odom"
            ]
        )
        ld.add_action(static_tf)

    return ld
