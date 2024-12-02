def init_parameters(self):
    """
    Init parameters
    """
    # Declare parameters
    self.declare_parameters(
        namespace='',
        parameters=[('local_frame', ''),
                    ('odom_frame', ''),
                    ('robot_color', [0.0]),
                    ('show_tags_var', False),
                    ('uwb_id_order', ['']),
                    ('wheel_radius', 0.0),
                    ('wheels_separation', 0.0),

                    ('use_pruning', False),
                    ('min_step_start_pruning', 0),
                    ('use_sharing', False),
                    ('min_step_start_sharing', 0),
                    ('actual_step_start_sharing', 0),
                    ('use_reset', False),

                    ('n_tags', 0),
                    ('n_phi', 0),
                    ('sigma_range', 0.0),
                    ('sigma_shared', 0.0),
                    ('roma2D.num_iterations_pre', 0),
                    ('roma2D.distance_threshold_pre', 0.0),
                    ('roma2D.percent_min_inliers_pre', 0.0),
                    ('roma2D.num_iterations_post', 0),
                    ('roma2D.distance_threshold_post', 0.0),
                    ('roma2D.percent_min_inliers_post', 0.0),
                    ('kr', 0.0),
                    ('kl', 0.0),
                    ('min_steps_reset', 0),
                    ('min_tags_after_reset', 0)
                    ])

    # Get parameters
    self.local_frame = self.get_parameter('local_frame').value
    self.odom_frame = self.get_parameter('odom_frame').value
    self.robot_color = self.get_parameter('robot_color').value
    self.show_tags_var = self.get_parameter('show_tags_var').value
    self.uwb_id_order = self.get_parameter('uwb_id_order').value
    self.wheel_radius = self.get_parameter('wheel_radius').value
    self.wheels_separation = self.get_parameter('wheels_separation').value
    self.use_pruning = self.get_parameter('use_pruning').value
    self.min_step_start_pruning = self.get_parameter('min_step_start_pruning').value
    self.use_sharing = self.get_parameter('use_sharing').value
    self.min_step_start_sharing = self.get_parameter('min_step_start_sharing').value
    self.actual_step_start_sharing = self.get_parameter('actual_step_start_sharing').value
    self.use_reset = self.get_parameter('use_reset').value
    self.n_tags = self.get_parameter('n_tags').value
    self.n_phi = self.get_parameter('n_phi').value
    self.sigma_range = self.get_parameter('sigma_range').value
    self.sigma_shared = self.get_parameter('sigma_shared').value
    self.num_iterations_pre = self.get_parameter('roma2D.num_iterations_pre').value
    self.distance_threshold_pre = self.get_parameter('roma2D.distance_threshold_pre').value
    self.percent_min_inliers_pre = self.get_parameter('roma2D.percent_min_inliers_pre').value
    self.num_iterations_post = self.get_parameter('roma2D.num_iterations_post').value
    self.distance_threshold_post = self.get_parameter('roma2D.distance_threshold_post').value
    self.percent_min_inliers_post = self.get_parameter('roma2D.percent_min_inliers_post').value
    self.kr = self.get_parameter('kr').value
    self.kl = self.get_parameter('kl').value
    self.min_steps_reset = self.get_parameter('min_steps_reset').value
    self.min_tags_after_reset = self.get_parameter('min_tags_after_reset').value

    # Print parameters
    self.get_logger().info(f'local_frame: {self.local_frame}')
    self.get_logger().info(f'odom_frame: {self.odom_frame}')
    self.get_logger().info(f'robot_color: {self.robot_color}')
    self.get_logger().info(f'show_tags_var: {self.show_tags_var}')
    self.get_logger().info(f'uwb_id_order: {self.uwb_id_order}')
    self.get_logger().info(f'wheel_radius: {self.wheel_radius}')
    self.get_logger().info(f'wheels_separation: {self.wheels_separation}')
    self.get_logger().info(f'use_pruning: {self.use_pruning}')
    self.get_logger().info(f'min_step_start_pruning: {self.min_step_start_pruning}')
    self.get_logger().info(f'use_sharing: {self.use_sharing}')
    self.get_logger().info(f'min_step_start_sharing: {self.min_step_start_sharing}')
    self.get_logger().info(f'actual_step_start_sharing: {self.actual_step_start_sharing}')
    self.get_logger().info(f'use_reset: {self.use_reset}')
    self.get_logger().info(f'n_tags: {self.n_tags}')
    self.get_logger().info(f'n_phi: {self.n_phi}')
    self.get_logger().info(f'sigma_range: {self.sigma_range}')
    self.get_logger().info(f'sigma_shared: {self.sigma_shared}')
    self.get_logger().info(f'num_iterations_pre: {self.num_iterations_pre}')
    self.get_logger().info(f'distance_threshold_pre: {self.distance_threshold_pre}')
    self.get_logger().info(f'percent_min_inliers_pre: {self.percent_min_inliers_pre}')
    self.get_logger().info(f'num_iterations_post: {self.num_iterations_post}')
    self.get_logger().info(f'distance_threshold_post: {self.distance_threshold_post}')
    self.get_logger().info(f'percent_min_inliers_post: {self.percent_min_inliers_post}')
