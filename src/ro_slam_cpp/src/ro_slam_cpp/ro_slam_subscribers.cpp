/**
 * ROSlam subscribers callbacks.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * August 21, 2024
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

#include <ro_slam_cpp/ro_slam.hpp>

namespace ROSlam
{
  /**
   * @brief UWBAarray callback.
   */
  void ROSlamNode::uwb_array_clbk(const UwbArray::SharedPtr msg)
  {
    // uint8_t anchor_num = msg->anchor_num;
    for (auto anchor : msg->uwbs)
    {
      int anchor_id = anchor.id;
      double anchor_dist = anchor.dist;
      // RCLCPP_INFO(this->get_logger(), "Anchor %d: %f", anchor_id, anchor_dist);
      UNUSED(anchor_id);
      UNUSED(anchor_dist);
    }
  }

  /**
   * @brief Joint state callback.
   *
   * @param msg Joint state message.
   */
  void ROSlamNode::odometry_clbk(const JointState::SharedPtr msg)
  {
    if (msg->name.size() != 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint state message");
      return;
    }

    int right_idx, left_idx;
    if (msg->name[0].find("right") != std::string::npos)
    {
      right_idx = 0;
      left_idx = 1;
    }
    else
    {
      right_idx = 1;
      left_idx = 0;
    }

    double omega_wheel_right = msg->velocity[right_idx];
    double omega_wheel_left = msg->velocity[left_idx];

    double v_lin = (omega_wheel_right + omega_wheel_left) * wheel_radius_ / 2.0;
    double v_ang = (omega_wheel_right - omega_wheel_left) * wheel_radius_ / wheels_separation_;

    std::cout << "v_lin: " << v_lin << ", v_ang: " << v_ang << std::endl;
  }

} // namespace ROSlam
