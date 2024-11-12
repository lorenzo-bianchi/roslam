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
                    ('wheel_radius', 0.0),
                    ('wheels_separation', 0.0)
                    ])

    # Get parameters
    self.local_frame = self.get_parameter('local_frame').value
    self.odom_frame = self.get_parameter('odom_frame').value
    self.robot_color = self.get_parameter('robot_color').value
    self.show_tags_var = self.get_parameter('show_tags_var').value
    self.wheel_radius = self.get_parameter('wheel_radius').value
    self.wheels_separation = self.get_parameter('wheels_separation').value

    # Print parameters
    self.get_logger().info(f'local_frame: {self.local_frame}')
    self.get_logger().info(f'odom_frame: {self.odom_frame}')
    self.get_logger().info(f'robot_color: {self.robot_color}')
    self.get_logger().info(f'show_tags_var: {self.show_tags_var}')
    self.get_logger().info(f'wheel_radius: {self.wheel_radius}')
    self.get_logger().info(f'wheels_separation: {self.wheels_separation}')
