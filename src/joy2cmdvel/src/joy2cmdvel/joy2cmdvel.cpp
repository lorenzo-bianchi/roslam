/**
 * Joy2cmdvel node initialization and parameters routines.
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
 * @brief Joy2cmdvel node constructor.
 *
 * @param node_opts Options for the base node.
 */
Joy2cmdvelNode::Joy2cmdvelNode(const rclcpp::NodeOptions & node_options)
: Node("joy2cmdvel", node_options)
{
  init_parameters();
  init_publishers();
  init_cgroups();
  init_subscriptions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief Joy2cmdvel node destructor.
 */
Joy2cmdvelNode::~Joy2cmdvelNode() {}

/**
 * @brief Routine to initialize callback groups.
 */
void Joy2cmdvelNode::init_cgroups()
{
  // Topic subscriptions
  joy_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void Joy2cmdvelNode::init_publishers()
{
  // Cmd_vel
  cmd_vel_pub_ = this->create_publisher<Twist>(
    cmd_vel_topic_name_,
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void Joy2cmdvelNode::init_subscriptions()
{
  // Joystick message
  auto joy_sub_opts = rclcpp::SubscriptionOptions();
  joy_sub_opts.callback_group = joy_clbk_group_;
  joy_sub_ = this->create_subscription<Joystick>(
    joy_topic_name_,
    1,
    std::bind(
      &Joy2cmdvelNode::joy_clbk,
      this,
      std::placeholders::_1),
    joy_sub_opts);
}

/**
 * @brief Routine to initialize node parameters.
 */
void Joy2cmdvelNode::init_parameters()
{
  // Declare parameters
  this->declare_parameter("cmd_vel_topic_name", rclcpp::ParameterValue(std::string("")));
  this->declare_parameter("joy_topic_name", rclcpp::ParameterValue(std::string("")));
  this->declare_parameter("max_ang_vel", rclcpp::ParameterValue(0.1));
  this->declare_parameter("max_lin_vel", rclcpp::ParameterValue(0.1));

  // Get parameters
  cmd_vel_topic_name_ = this->get_parameter("cmd_vel_topic_name").as_string();
  joy_topic_name_ = this->get_parameter("joy_topic_name").as_string();
  max_ang_vel_ = this->get_parameter("max_ang_vel").as_double();
  max_lin_vel_ = this->get_parameter("max_lin_vel").as_double();

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "joy_topic_name: %s", joy_topic_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "max_ang_vel: %f", max_ang_vel_);
  RCLCPP_INFO(this->get_logger(), "max_lin_vel: %f", max_lin_vel_);
}

} // namespace Joy2cmdvel

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Joy2cmdvel::Joy2cmdvelNode)