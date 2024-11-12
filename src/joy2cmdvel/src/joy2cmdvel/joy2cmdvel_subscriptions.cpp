/**
 * Joy2cmdvel topic subscription callbacks.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * October 26, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <joy2cmdvel/joy2cmdvel.hpp>

namespace Joy2cmdvel
{

/**
 * @brief Publish data on cmd_vel topic
 */
void Joy2cmdvelNode::joy_clbk(const Joystick::SharedPtr msg)
{
  if (msg->connected && disconnected_)
  {
    RCLCPP_INFO(this->get_logger(), "Joystick connected");
    disconnected_ = false;
  }
  else if (!msg->connected)
  {
    disconnected_ = true;
    RCLCPP_ERROR(this->get_logger(), "Joystick disconnected");
    return;
  }

  // Cmd_vel
  Twist twist_msg;
  double sign, cmd;
  double thr = 1e-1;

  sign = msg->ly <= 0 ? 1 : -1;
  cmd = msg->ly * msg->ly;
  twist_msg.linear.set__x(cmd > thr ? sign * cmd * max_lin_vel_ : 0.0);

  sign = msg->rx <= 0 ? 1 : -1;
  cmd = msg->rx * msg->rx;
  twist_msg.angular.set__z(cmd > thr ? sign * cmd * max_ang_vel_ : 0.0);

  cmd_vel_pub_->publish(twist_msg);
}

} // namespace Joy2cmdvel
