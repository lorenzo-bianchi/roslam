/**
 * ROSlam node initialization and parameters routines.
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
 * @brief Routine to initialize parameters.
 */
void ROSlamNode::init_parameters()
{
  // Declare parameters
  this->declare_parameter("wheel_radius", rclcpp::ParameterValue(0.1));
  this->declare_parameter("wheels_separation", rclcpp::ParameterValue(0.1));
  // this->declare_parameter("distance_thresh_max_fails", rclcpp::ParameterValue(1));
  // this->declare_parameter("field_size", rclcpp::ParameterValue(std::vector<double>{20.0, 10.0, 2.0}));
  // this->declare_parameter("plot_size", rclcpp::ParameterValue(std::vector<int64_t>{1, 1}));
  // this->declare_parameter("save_yaml", rclcpp::ParameterValue(false));
  // this->declare_parameter("save_yaml_path", rclcpp::ParameterValue(std::string("")));

  // Get parameters
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheels_separation_ = this->get_parameter("wheels_separation").as_double();
  // distance_thresh_max_fails_ = this->get_parameter("distance_thresh_max_fails").as_int();
  // field_size_ = this->get_parameter("field_size").as_double_array();
  // plot_size_ = this->get_parameter("plot_size").as_integer_array();
  // save_yaml_ = this->get_parameter("save_yaml").as_bool();
  // save_yaml_path_ = this->get_parameter("save_yaml_path").as_string();

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "wheel_radius: %f", wheel_radius_);
  RCLCPP_INFO(this->get_logger(), "wheels_separation: %f", wheels_separation_);
}

} // namespace ROSlam
