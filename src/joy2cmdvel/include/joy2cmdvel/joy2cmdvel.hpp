/**
 * Joy2cmdvel module headers.
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

#ifndef JOY2CMDVEL_HPP
#define JOY2CMDVEL_HPP

#include <cfloat>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <mutex>
#include <pthread.h>
#include <unistd.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ro_slam_interfaces/msg/joystick.hpp>

using namespace geometry_msgs::msg;
using namespace std::chrono_literals;
using namespace rcl_interfaces::msg;
using namespace ro_slam_interfaces::msg;

#define UNUSED(arg) (void)(arg)

namespace Joy2cmdvel
{

/**
 * Convert messages and transform data
 */
class Joy2cmdvelNode : public rclcpp::Node
{
public:
  Joy2cmdvelNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~Joy2cmdvelNode();

private:
  /* Callback groups */
  rclcpp::CallbackGroup::SharedPtr joy_clbk_group_;

  /* Subscribers */
  rclcpp::Subscription<Joystick>::SharedPtr joy_sub_;

  /* Publishers */
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;

  /* Node parameters */
  std::string cmd_vel_topic_name_ = "";
  std::string joy_topic_name_ = "";
  double max_ang_vel_ = 0.0;
  double max_lin_vel_ = 0.0;

  /* Callbacks */
  void joy_clbk(const Joystick::SharedPtr msg);

  /* Node init functions */
  void init_cgroups();
  void init_parameters();
  void init_publishers();
  void init_subscriptions();

  /* Internal state variables */
  bool disconnected_ = true;
};

} // namespace Joy2cmdvel

#endif // JOY2CMDVEL_HPP
