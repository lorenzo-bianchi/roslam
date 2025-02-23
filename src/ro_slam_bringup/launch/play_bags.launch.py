"""
Play bags launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>

February 22, 2025
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

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
    {"path": "logs/20241220_test1", "robot_from": 1, "robot_to": 4},
    {"path": "logs/20241220_robot2_test4", "robot_from": 2, "robot_to": 8}
]

def generate_launch_description():
    ld = LaunchDescription()

    for bag in bags:
        remaps = [f"/robot{bag['robot_from']}{t}:=/robot{bag['robot_to']}{t}" for t in topics]
        remap_str = " ".join(remaps)

        topics_str = " ".join([f"/robot{bag['robot_from']}{t}" for t in topics])

        process = ExecuteProcess(
            cmd=["ros2", "bag", "play", bag["path"], "--remap"] +
            remap_str.split() +
            ["--topics"] + topics_str.split(),
            output="screen"
        )
        ld.add_action(process)

    return ld
