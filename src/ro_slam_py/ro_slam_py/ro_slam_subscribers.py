import os
import numpy as np
import time

import cProfile

from ro_slam_interfaces.msg import UwbArray, Landmark, LandmarkArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from ro_slam_py.ro_slam_fed_ekf import FedEkfSharedData

def debug_clbk(self, msg: Empty):
    """
    Debug callback
    """
    msg = msg
    if os.getenv('DEBUG') == '1':
        np.set_printoptions(suppress=True, edgeitems=18, linewidth=200, precision=4)
        self.get_logger().info('Debug mode activated')
        pass

def uwb_array_clbk(self, msg: UwbArray):
    """
    UWB array callback
    """
    if len(self.uwb_idx) != 0:
        distances = np.full((self.data.n_tags, ), np.inf)
        for anchor in msg.uwbs:
            distances[self.uwb_idx[anchor.id_str]] = anchor.dist
    else:
        distances = np.zeros((msg.anchor_num, ))
        for anchor in msg.uwbs:
            distances[anchor.id] = anchor.dist

    distances += self.bias_range

    if self.corrupt_measurement_enable and self.fed_ekf.k <= 600:
        distances += np.random.normal(self.corrupt_measurement_bias, self.corrupt_measurement_sigma, distances.shape)

    tic = time.time()
    self.fed_ekf.correction(distances)
    # print(f"Correction time: {time.time() - tic}")

    if self.pruning_enable:
        self.fed_ekf.pruning()

    # Get new tags poses after correction and pruning
    self.tags_poses = self.fed_ekf.get_tags_poses()

    # Share data
    if self.sharing_enable and self.fed_ekf.k >= self.sharing_min_step_to_start - 1:
        if np.all(self.tags_poses['last_hyp']):
            if self.actual_step_start_sharing == -1:
                self.actual_step_start_sharing = self.fed_ekf.k

            # Populate shared data msg and publish
            landmark_array_msg = LandmarkArray()
            landmark_array_msg.header.stamp = self.get_clock().now().to_msg()
            landmark_array_msg.id = self.robot_id
            for tag in self.tags_poses:
                landmark = Landmark()
                landmark.x = float(tag['x'])
                landmark.y = float(tag['y'])
                landmark.var_x = float(tag['var_x'])
                landmark.var_y = float(tag['var_y'])
                landmark.cov_xy = float(tag['cov_xy'])
                landmark_array_msg.landmarks.append(landmark)
            self.landmark_pub.publish(landmark_array_msg)

    self.broadcast_pose()

    # print(self.fed_ekf.weights)
    # print()


def odometry_clbk(self, msg: JointState):
    """
    Odometry callback
    """
    if len(msg.name) != 2:
        self.get_logger().error('Invalid JointState message')
        return

    # print(self.fed_ekf.k + 2)

    if 'right' in msg.name[0]:
        right_idx = 0
        left_idx = 1
    else:
        right_idx = 1
        left_idx = 0

    vel_wheel_right = msg.velocity[right_idx] * self.wheel_radius
    vel_wheel_left = msg.velocity[left_idx] * self.wheel_radius

    if abs(vel_wheel_left) < 1e-5 and abs(vel_wheel_right) < 1e-5:  # TODO: check this threshold
        return

    tic = time.time()
    self.fed_ekf.prediction(vel_wheel_right, vel_wheel_left)
    # print(f"Prediction time: {time.time() - tic}")

    self.broadcast_pose()

    # print(self.fed_ekf.x_hat_slam)
    # print()


# For Matlab tests
def landmark_array_test_clbk(self, msg: LandmarkArray):
    """
    Landmark array callback
    """
    # Do not consider data from the same robot
    if msg.id == self.robot_id:
        return

    print(f"Received shared landmarks from robot {msg.id}")

    if msg.id == 100:
        this_robot = FedEkfSharedData()
        this_robot.robot_id = self.robot_id
        this_robot.tags_positions = np.array([[l['x'], l['y']] for l in self.tags_poses]).T
        this_robot.tags_vars = np.array([[l['var_x'], l['var_y'], l['cov_xy']] for l in self.tags_poses]).T

        profiler = cProfile.Profile()
        profiler.enable()
        tic = time.time()
        self.fed_ekf.correction_shared(this_robot, self.shared_data)
        # print(f'Correction shared time: {time.time() - tic}')
        profiler.disable()
        profiler.dump_stats(f'logs/profiler_{self.fed_ekf.k}.prof')

        if self.pruning_enable:
            self.fed_ekf.pruning()

        if self.reset_enable and self.fed_ekf.do_reset:
            # TODO: find a way to get real position at reset

            time_reset = self.get_clock().now().nanoseconds
            self.get_logger().info(f'Robot {self.robot_id} resetting at t = {time_reset}')
            self.t_resets.append((self.fed_ekf.k, time_reset))

            tic = time.time()
            self.fed_ekf.reset()
            # print(f"Reset time: {time.time() - tic}")

        self.shared_data = np.array([])
        return

    tags_positions = np.array([[l.x, l.y] for l in msg.landmarks]).T
    tags_vars = np.array([[l.var_x, l.var_y, l.cov_xy] for l in msg.landmarks]).T

    data = FedEkfSharedData()
    data.robot_id = msg.id
    data.tags_positions = tags_positions
    data.tags_vars = tags_vars

    self.shared_data = np.append(self.shared_data, data)

def landmark_array_clbk(self, msg: LandmarkArray):
    """
    Landmark array callback
    """
    # Do not consider data from the same robot
    if msg.id == self.robot_id:
        return

    # print(f"Received shared landmarks from robot {msg.id}")

    # Check if a new robot is sharing data
    if self.other_robots_idx.get(msg.id) is None:
        print(f"Robot {msg.id} is sharing data")

    # Update shared data
    tags_positions = np.array([[l.x, l.y] for l in msg.landmarks]).T
    tags_vars = np.array([[l.var_x, l.var_y, l.cov_xy] for l in msg.landmarks]).T

    data = FedEkfSharedData()
    data.robot_id = msg.id
    data.tags_positions = tags_positions
    data.tags_vars = tags_vars

    self.shared_data[msg.id] = data
    self.other_robots_idx[msg.id] = True

    if len(self.other_robots_idx) > 1 and all(self.other_robots_idx.values()):
        this_robot = FedEkfSharedData()
        this_robot.robot_id = self.robot_id
        this_robot.tags_positions = np.array([[l['x'], l['y']] for l in self.tags_poses]).T
        this_robot.tags_vars = np.array([[l['var_x'], l['var_y'], l['cov_xy']] for l in self.tags_poses]).T

        profiler = cProfile.Profile()
        profiler.enable()
        tic = time.time()
        # transform self.shared_data to a list
        shared_data_array = np.array(list(self.shared_data.values()))
        self.fed_ekf.correction_shared(this_robot, shared_data_array)
        # print(f'Correction shared time: {time.time() - tic}')
        profiler.disable()
        profiler.dump_stats(f'logs/profiler_{self.fed_ekf.k}.prof')

        if self.pruning_enable:
            self.fed_ekf.pruning()

        if self.reset_enable and self.fed_ekf.do_reset:
            # TODO: find a way to get real position at reset

            time_reset = self.get_clock().now().nanoseconds
            self.get_logger().info(f'Robot {self.robot_id} resetting at t = {time_reset}')
            self.t_resets.append((self.fed_ekf.k, time_reset))

            tic = time.time()
            self.fed_ekf.reset()
            self.corrupt_measurement_enable = False
            self.reset_enable = False
            # print(f"Reset time: {time.time() - tic}")

        self.shared_data = {}
        self.other_robots_idx = {k: False for k in self.other_robots_idx.keys()}
        return
