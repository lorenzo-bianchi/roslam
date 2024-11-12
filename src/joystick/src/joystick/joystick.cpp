/**
 * Joystick node initialization and parameters routines.
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

#include <joystick/joystick.hpp>

namespace joystick
{

/**
 * @brief Joystick node constructor.
 *
 * @param node_opts Options for the base node.
 */
JoystickNode::JoystickNode(const rclcpp::NodeOptions & node_options)
: Node("joystick", node_options)
{
  init_parameters();
  init_atomics();
  init_publishers();
  init_joystick();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Joystick node destructor.
 */
JoystickNode::~JoystickNode()
{
  // Stop thread and wait for it
  stop_thread.store(true, std::memory_order_release);
  joy_thread_.join();
  RCLCPP_INFO(this->get_logger(), "Thread joined correctly");

  // Close joystick device file
  if (js_ != -1) close(js_);
}

/**
 * @brief Routine to initialize atomic members.
 */
void JoystickNode::init_atomics()
{
  stop_thread.store(false, std::memory_order_release);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void JoystickNode::init_publishers()
{
  // Joystick
  joy_pub_ = this->create_publisher<Joystick>(
    joy_topic_name_,
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize joystick buttons and axes.
 */
void JoystickNode::init_joystick()
{
  joy_thread_ = std::thread{
    &JoystickNode::joy_routine,
    this};
}

/**
 * @brief Routine to initialize parameters.
 */
void JoystickNode::init_parameters()
{
  // Declare parameters
  this->declare_parameter("axis_deadzone_val", rclcpp::ParameterValue(1));
  this->declare_parameter("axis_max_val", rclcpp::ParameterValue(0.1));
  this->declare_parameter("joy_topic_name", rclcpp::ParameterValue(std::string("")));
  this->declare_parameter("joy_path", rclcpp::ParameterValue(std::string("")));

  // Get parameters
  axis_deadzone_val_ = this->get_parameter("axis_deadzone_val").as_int();
  axis_max_val_ = this->get_parameter("axis_max_val").as_double();
  joy_topic_name_ = this->get_parameter("joy_topic_name").as_string();
  joy_path_ = this->get_parameter("joy_path").as_string();

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "axis_deadzone_val: %ld", axis_deadzone_val_);
  RCLCPP_INFO(this->get_logger(), "axis_max_val: %f", axis_max_val_);
  RCLCPP_INFO(this->get_logger(), "joy_topic_name: %s", joy_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "joy_path: %s", joy_path_.c_str());
}

} // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickNode)