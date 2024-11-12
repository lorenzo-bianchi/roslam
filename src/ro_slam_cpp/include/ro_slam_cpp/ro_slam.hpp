/**
 * ROSlam headers.
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

#ifndef ROSLAM__ROSLAM_HPP
#define ROSLAM__ROSLAM_HPP

#include <algorithm>
#include <atomic>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <ro_slam_interfaces/msg/uwb_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ro_slam_cpp/fed_ekf.hpp>

#define UNUSED(arg) (void)(arg)
#define LINE std::cout << __FUNCTION__ << ", LINE: " << __LINE__ << std::endl;

using namespace std::chrono_literals;
using namespace rcl_interfaces::msg;
using namespace ro_slam_interfaces::msg;
using namespace sensor_msgs::msg;

namespace ROSlam
{

/**
 * @brief ROSlam node class.
 */
class ROSlamNode : public rclcpp::Node
{
public:
  ROSlamNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~ROSlamNode();

private:
  /* Node init functions */
  void init_parameters();
  void init_callback_groups();
  void init_publishers();
  void init_subscribers();
  void init_timers();

  /* Publishers */
  // rclcpp::Publisher<...>::SharedPtr marker_pub_;

  /* Callback groups */
  rclcpp::CallbackGroup::SharedPtr uwb_array_cgroup_;
  rclcpp::CallbackGroup::SharedPtr odometry_cgroup_;

  /* Subscribers */
  rclcpp::Subscription<UwbArray>::SharedPtr uwb_array_sub_;
  rclcpp::Subscription<JointState>::SharedPtr odometry_sub_;

  /* Timers */
  rclcpp::TimerBase::SharedPtr visualization_timer_;

  /* Callbacks */
  void uwb_array_clbk(const UwbArray::SharedPtr msg);
  void odometry_clbk(const JointState::SharedPtr msg);

  /* Utility routines */

  /* Node parameters */
  double wheel_radius_ = 0.0;
  double wheels_separation_ = 0.0;
  // int64_t distance_thresh_max_fails_;
  // std::vector<double> field_size_;
  // std::vector<int64_t> plot_size_;
  // bool save_yaml_;
  // std::string save_yaml_path_;

  /* Synchronization primitives for internal update operations */
  std::atomic<bool> stop_thread;

  /* Internal state variables */
  FedEkf ekf_;
};

} // namespace ROSlam

#endif // ROSLAM__ROSLAM_HPP