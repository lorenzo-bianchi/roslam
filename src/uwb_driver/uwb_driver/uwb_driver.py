# UWB driver node.
#
# Lorenzo Bianchi <lnz.bnc@gmail.com>
#
# March 9, 2023

# This is free software.
# You can redistribute it and/or modify this file under the
# terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This file is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this file; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.

import serial, threading, time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ro_slam_interfaces.msg import Uwb, UwbArray
from geometry_msgs.msg import PoseStamped

class DWM1001_API_COMMANDS:
    LEC          = b'lec\n'     # Show measurement and position in CSV format
    RESET        = b'reset\n'   # Reset dev board
    SHELL_MODE   = b'\x0D\x0D'  # Send double ENTER

class UWBDriver(Node):
    serial_is_open = False

    def __init__(self):
        # Initialize ROS2 Node
        super().__init__('uwb_driver')

        # Initialize parameters
        self.declare_parameter('id', 0)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameters('print_tty', False)
        self.declare_parameter('publish_rviz', False)

        self.id = int(self.get_parameter('id').value)
        self.dwm_port = self.get_parameter('port').value
        self.print_tty = self.get_parameter('print_tty').value
        self.publish_rviz = self.get_parameter('publish_rviz').value

        self.get_logger().info(f'id: {self.id}')
        self.get_logger().info(f'port: {self.dwm_port}')
        self.get_logger().info(f'print_tty: {self.print_tty}')
        self.get_logger().info(f'publish_rviz: {self.publish_rviz}')

        # Serial port settings
        self.uwb_pub = self.create_publisher(UwbArray, f'/robot{self.id}/uwb_tag', 0)
        if self.publish_rviz:
            self.uwb_pub_rviz = self.create_publisher(PoseStamped, f'/robot{self.id}/rviz/uwb_pose', 0)

        try:
            self.serial_port_DWM1001 = serial.Serial(
                port = self.dwm_port,
                baudrate = 115200,
                timeout = 2
            )

            self.get_logger().info(f'Opened port {self.serial_port_DWM1001.name}')

            self.serial_port_DWM1001.write(DWM1001_API_COMMANDS.SHELL_MODE)
            time.sleep(1)

            # Send 'lec' command to get distances in CSV format
            self.serial_port_DWM1001.write(DWM1001_API_COMMANDS.LEC)
            time.sleep(1)

            # Clean output from unnecessary lines
            i = 0
            serial_read_line = b''
            while not serial_read_line.decode().startswith('DIST'):
                i += 1
                # Read from serial port
                serial_read_line = self.serial_port_DWM1001.read_until(expected=b'\r\n')

            self.get_logger().info('Starting thread to read anchors distances')
            self.stop_thread = False
            self.serial_thread = threading.Thread(target=self.read_serial)
            self.serial_thread.start()

            self.serial_is_open = True

        except:
            self.get_logger().error(f'Can\'t open port {self.dwm_port}')

    def read_serial(self):
        while not self.stop_thread:
            try:
                msg_array = UwbArray()
                msg_array.header.stamp = self.get_clock().now().to_msg()

                # Read from serial port
                serial_read_line_enc = self.serial_port_DWM1001.read_until(expected=b'\r\n')
                serial_read_line = serial_read_line_enc.decode()[:-2]
                splits = serial_read_line.split(',')
                if splits[0] == '':
                    continue
                if self.print_tty:
                    print(splits)

                n_anchors = int(splits[1])
                msg_array.anchor_num = n_anchors
                splits = splits[2:]

                for i in range(n_anchors):
                    msg = Uwb()
                    msg.header = msg_array.header

                    data = splits[i*6 : (i+1)*6]

                    msg.id = i
                    msg.id_str = data[1]
                    msg.x = float(data[2])
                    msg.y = float(data[3])
                    msg.z = float(data[4])
                    msg.dist = float(data[5])

                    msg_array.uwbs.append(msg)

                if self.publish_rviz and len(splits) > 2+n_anchors*6:
                    i = n_anchors*6+1
                    msg_array.x_est = float(splits[i])
                    msg_array.y_est = float(splits[i+1])
                    msg_array.z_est = float(splits[i+2])
                    msg_array.quality_factor = int(float(splits[i+3]))

                    msg_pose = PoseStamped()
                    msg_pose.header.stamp = self.get_clock().now().to_msg()
                    msg_pose.header.frame_id = f'uwb{self.id}_link'
                    msg_pose.pose.position.x = msg_array.x_est
                    msg_pose.pose.position.y = msg_array.y_est
                    msg_pose.pose.position.z = msg_array.z_est

                    self.uwb_pub_rviz.publish(msg_pose)

                self.uwb_pub.publish(msg_array)

            except:
                self.get_logger().error('Error inside thread. Press CTRL+C to quit...')
                time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    uwb_driver = UWBDriver()
    if uwb_driver.serial_is_open:
        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(uwb_driver)
        try:
            executor.spin()
        except:
            executor.shutdown()

            uwb_driver.serial_port_DWM1001.write(DWM1001_API_COMMANDS.RESET)
            uwb_driver.stop_thread = True
            time.sleep(2)
            uwb_driver.destroy_node()

            # rclpy.shutdown()


if __name__ == '__main__':
    main()
