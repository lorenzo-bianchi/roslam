def init_parameters(self):
    """
    Init parameters
    """
    # Declare parameters
    self.declare_parameters(
        namespace='',
        parameters=[('frame_local', ''),
                    ('frame_odom', ''),
                    ('robot_color', [0.0]),
                    ('show_tags_var', False),
                    ('uwb_id_order', ['']),
                    ('wheel_radius', 0.0),
                    ('wheels_separation', 0.0),

                    ('pruning.dim_deque', 0),
                    ('pruning.enable', False),
                    ('pruning.min_step_to_start', 0),

                    ('sharing.enable', False),
                    ('sharing.min_step_to_start', 0),

                    ('reset.enable', False),
                    ('reset.min_steps', 0),
                    ('reset.min_tags_converged', 0),

                    ('n_tags', 0),
                    ('n_phi', 0),
                    ('sigma_range', 0.0),
                    ('sigma_shared', 0.0),
                    ('bias_range', 0.0),
                    ('roma2D.num_iterations_pre', 0),
                    ('roma2D.distance_threshold_pre', 0.0),
                    ('roma2D.percent_min_inliers_pre', 0.0),
                    ('roma2D.num_iterations_post', 0),
                    ('roma2D.distance_threshold_post', 0.0),
                    ('roma2D.percent_min_inliers_post', 0.0),
                    ('kr', 0.0),
                    ('kl', 0.0),
                    ('corrupt_measurement.enable', False),
                    ('corrupt_measurement.sigma', 0.0),
                    ('corrupt_measurement.bias', 0.0),
                    ('corrupt_measurement.time_stop', 0),
                    ])

    # Get parameters
    self.frame_local = self.get_parameter('frame_local').value
    self.frame_odom = self.get_parameter('frame_odom').value
    self.robot_color = self.get_parameter('robot_color').value
    self.show_tags_var = self.get_parameter('show_tags_var').value
    self.uwb_id_order = self.get_parameter('uwb_id_order').value
    self.wheel_radius = self.get_parameter('wheel_radius').value
    self.wheels_separation = self.get_parameter('wheels_separation').value
    self.pruning_enable = self.get_parameter('pruning.enable').value
    self.pruning_dim_deque = self.get_parameter('pruning.dim_deque').value
    self.pruning_min_step_to_start = self.get_parameter('pruning.min_step_to_start').value
    self.sharing_enable = self.get_parameter('sharing.enable').value
    self.sharing_min_step_to_start = self.get_parameter('sharing.min_step_to_start').value
    self.reset_enable = self.get_parameter('reset.enable').value
    self.reset_min_steps = self.get_parameter('reset.min_steps').value
    self.reset_min_tags_converged = self.get_parameter('reset.min_tags_converged').value
    self.n_tags = self.get_parameter('n_tags').value
    self.n_phi = self.get_parameter('n_phi').value
    self.sigma_range = self.get_parameter('sigma_range').value
    self.sigma_shared = self.get_parameter('sigma_shared').value
    self.bias_range = self.get_parameter('bias_range').value
    self.num_iterations_pre = self.get_parameter('roma2D.num_iterations_pre').value
    self.distance_threshold_pre = self.get_parameter('roma2D.distance_threshold_pre').value
    self.percent_min_inliers_pre = self.get_parameter('roma2D.percent_min_inliers_pre').value
    self.num_iterations_post = self.get_parameter('roma2D.num_iterations_post').value
    self.distance_threshold_post = self.get_parameter('roma2D.distance_threshold_post').value
    self.percent_min_inliers_post = self.get_parameter('roma2D.percent_min_inliers_post').value
    self.kr = self.get_parameter('kr').value
    self.kl = self.get_parameter('kl').value
    self.corrupt_measurement_enable = self.get_parameter('corrupt_measurement.enable').value
    self.corrupt_measurement_sigma = self.get_parameter('corrupt_measurement.sigma').value
    self.corrupt_measurement_bias = self.get_parameter('corrupt_measurement.bias').value
    self.corrupt_measurement_time_stop = self.get_parameter('corrupt_measurement.time_stop').value

    # Print parameters
    self.get_logger().info(f'frame_local: {self.frame_local}')
    self.get_logger().info(f'frame_odom: {self.frame_odom}')
    self.get_logger().info(f'robot_color: {self.robot_color}')
    self.get_logger().info(f'show_tags_var: {self.show_tags_var}')
    self.get_logger().info(f'uwb_id_order: {self.uwb_id_order}')
    self.get_logger().info(f'wheel_radius: {self.wheel_radius}')
    self.get_logger().info(f'wheels_separation: {self.wheels_separation}')
    self.get_logger().info(f'pruning_enable: {self.pruning_enable}')
    self.get_logger().info(f'pruning_dim_deque: {self.pruning_dim_deque}')
    self.get_logger().info(f'pruning_min_step_to_start: {self.pruning_min_step_to_start}')
    self.get_logger().info(f'sharing_enable: {self.sharing_enable}')
    self.get_logger().info(f'sharing_min_step_to_start: {self.sharing_min_step_to_start}')
    self.get_logger().info(f'reset_enable: {self.reset_enable}')
    self.get_logger().info(f'n_tags: {self.n_tags}')
    self.get_logger().info(f'n_phi: {self.n_phi}')
    self.get_logger().info(f'sigma_range: {self.sigma_range}')
    self.get_logger().info(f'sigma_shared: {self.sigma_shared}')
    self.get_logger().info(f'bias_range: {self.bias_range}')
    self.get_logger().info(f'num_iterations_pre: {self.num_iterations_pre}')
    self.get_logger().info(f'distance_threshold_pre: {self.distance_threshold_pre}')
    self.get_logger().info(f'percent_min_inliers_pre: {self.percent_min_inliers_pre}')
    self.get_logger().info(f'num_iterations_post: {self.num_iterations_post}')
    self.get_logger().info(f'distance_threshold_post: {self.distance_threshold_post}')
    self.get_logger().info(f'percent_min_inliers_post: {self.percent_min_inliers_post}')
    self.get_logger().info(f'kr: {self.kr}')
    self.get_logger().info(f'kl: {self.kl}')
    self.get_logger().info(f'corrupt_measurement_enable: {self.corrupt_measurement_enable}')
    self.get_logger().info(f'corrupt_measurement_sigma: {self.corrupt_measurement_sigma}')
    self.get_logger().info(f'corrupt_measurement_bias: {self.corrupt_measurement_bias}')
    self.get_logger().info(f'corrupt_measurement_time_stop: {self.corrupt_measurement_time_stop}')
