import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ro_slam_interfaces.msg import Uwb, UwbArray

class UWBParserNode(Node):
    def __init__(self):
        super().__init__('uwb_parser_node')

        # Subscribe to topic
        self.subscription = self.create_subscription(
            String,
            '/uwb_raw_data',
            self.listener_callback,
            10
        )
        self.subscription # prevent unused variable warning
        self.pubs = {}

    def listener_callback(self, msg):
        input_string = msg.data

        # Parse the input string
        parts = input_string.split(';')
        self.nun_anchors = len(parts) - 2
        # Extract the robot name
        robot_name = parts[0]

        if robot_name not in self.pubs:
            self.get_logger().info('Received first message from ' + robot_name)
            self.get_logger().info(f'Number of anchors: {self.nun_anchors}')

            # Initialize publishers
            self.pubs[robot_name] = self.create_publisher(UwbArray, f'/{robot_name}/uwb_tag', 10)

        timestamp = self.get_clock().now().to_msg()
        uwb_array_msg = UwbArray()
        uwb_array_msg.header.stamp = timestamp
        uwb_array_msg.anchor_num = self.nun_anchors
        if len(parts) > 1:
            # Process the subsequent data
            for part in parts[1:-1]:
                fields = part.split(',')

                uwb_msg = Uwb()
                uwb_msg.header.frame_id = robot_name
                uwb_msg.header.stamp = timestamp
                uwb_msg.id = int(fields[0])
                uwb_msg.x = float(fields[1])
                uwb_msg.y = float(fields[2])
                uwb_msg.z = float(fields[3])
                uwb_msg.dist = float(fields[4])

                uwb_array_msg.uwbs.append(uwb_msg)

            self.pubs[robot_name].publish(uwb_array_msg)
        else:
            self.get_logger().error("Invalid input string.")

def main(args=None):
    rclpy.init(args=args)
    node = UWBParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()