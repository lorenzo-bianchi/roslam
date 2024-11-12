import numpy as np
from numpy import pi, ceil
import re
from threading import Thread, Lock

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from ro_slam_interfaces.msg import UwbArray, LandmarkArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray

from .ro_slam_qos import qos_best_effort, qos_reliable
from .ro_slam_fed_ekf import FedEkf, FedEkfData, FedEkfSharedData


class ROSlamNode(Node):
    def __init__(self):
        super().__init__('ro_slam_py')

        self.init_parameters()
        self.init_tf2()
        self.init_publishers()
        self.init_ekf()
        self.init_callback_groups()
        self.init_timers()
        self.init_subscriptions()

        self.get_logger().info('Node initialized')

    # Import methods
    from .ro_slam_params import init_parameters
    from .ro_slam_subscribers import debug_clbk, uwb_array_clbk, odometry_clbk, landmark_array_clbk, landmark_array_test_clbk
    from .ro_slam_utils import broadcast_pose, publish_tags


    def cleanup(self):
        pass


    def init_ekf(self):
        """
        Init EKF
        """
        self.robot_id = int(re.findall(r"\d+$", self.get_namespace())[0])
        self.use_pruning = True
        self.min_step_start_pruning = 70
        self.use_sharing = True
        self.min_step_start_sharing = 100
        self.actual_step_start_sharing = -1
        self.use_reset = True
        self.t_resets = []

        self.data = FedEkfData()
        self.data.n_tags = 10
        self.data.n_phi = 8
        self.data.sigma_phi = 2 * pi / (1.5 * self.data.n_phi)
        self.data.sigma_range = 0.2
        self.data.min_zeros_start_pruning = ceil(0.6 * self.data.n_phi)
        self.data.num_iterations = 100
        self.data.distance_threshold = 0.1
        self.data.percent_min_inliers = 0.7
        self.data.wheels_separation = self.wheels_separation
        self.data.kr = 0.0001
        self.data.kl = 0.0001
        self.data.min_steps_reset = 100
        self.data.min_tags_after_reset = 3

        self.shared_data = np.empty((0, ), dtype=object)
        self.tags_poses: np.ndarray = None

        x0 = np.array([0.0, 0.0, 0.0])

        self.fed_ekf = FedEkf(x0, self.data)


    def init_tf2(self):
        """
        Init tf2
        """
        self.tf_broadcaster = TransformBroadcaster(self)


    def init_callback_groups(self):
        """
        Init callback groups
        """
        # Subscribers
        self.uwb_array_cgroup = MutuallyExclusiveCallbackGroup()
        self.odometry_cgroup = MutuallyExclusiveCallbackGroup()
        self.uwb_array_cgroup = MutuallyExclusiveCallbackGroup()
        self.landmark_array_cgroup = MutuallyExclusiveCallbackGroup()

        # Timers
        self.tf_timer_cgroup = MutuallyExclusiveCallbackGroup()


    def init_publishers(self):
        """
        Init publishers
        """
        # Visualization landmarks
        self.tags_pub = self.create_publisher(
            MarkerArray,
            '/visualization/landmarks',
            qos_best_effort
        )

        # Shared landmarks
        self.landmark_pub = self.create_publisher(
            LandmarkArray,
            '/shared_landmarks',
            qos_best_effort
        )


    def init_subscriptions(self):
        """
        Init subscriptions
        """
        # Debug
        self.debug_sub = self.create_subscription(
            Empty,
            '/debug',
            self.debug_clbk,
            qos_best_effort
        )

        # Odometry
        self.odometry_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.odometry_clbk,
            qos_best_effort,
            callback_group=self.odometry_cgroup
        )

        # UWB array
        self.uwb_array_sub = self.create_subscription(
            UwbArray,
            '/uwb_tag',
            self.uwb_array_clbk,
            qos_best_effort,
            callback_group=self.uwb_array_cgroup
        )

        # Shared landmarks
        self.landmark_array_sub = self.create_subscription(
            LandmarkArray,
            '/shared_landmarks',
            self.landmark_array_clbk,
            qos_best_effort,
            callback_group=self.landmark_array_cgroup
        )

        # Shared landmarks (TEST) # TODO: remove
        self.landmark_array_sub_test = self.create_subscription(
            LandmarkArray,
            '/shared_landmarks_test',
            self.landmark_array_test_clbk,
            qos_best_effort,
            callback_group=self.landmark_array_cgroup
        )


    def init_timers(self):
        """
        Init timers
        """
        self.tf_timer = self.create_timer(
            4,
            lambda: self.broadcast_pose(),
            callback_group=self.tf_timer_cgroup
        )