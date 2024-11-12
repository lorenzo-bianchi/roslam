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
 * @brief ROSlam node constructor.
 *
 * @param node_opts Options for the base node.
 */
ROSlamNode::ROSlamNode(const rclcpp::NodeOptions & node_options)
: Node("ro_slam_cpp", node_options)
{
  init_parameters();
  init_publishers();
  init_callback_groups();
  init_subscribers();
  init_timers();

  //
  fed_ekf_data_t data;
  data.n_tags = 10;
  data.n_phi = 16;
  data.sigma_phi = 2 * M_PI / (1.5 * data.n_phi);
  data.sigma_range = 0.2;
  data.use_pruning = true;
  data.min_zeros_start_pruning = ceil(0.6 * data.n_phi);
  data.step_start_pruning = 100;
  data.num_iterations = 100;
  data.distance_threshold = 0.1;
  data.percent_min_inliers = 0.7;
  data.use_reset = true;
  data.min_steps_reset = 100;

  Vector3d x0 = Vector3d::Zero();

  FedEkf fed_ekf = FedEkf(x0, data);
  //

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief ROSlam node destructor.
 */
ROSlamNode::~ROSlamNode()
{
  RCLCPP_INFO(this->get_logger(), "Node destroyed");
}

/**
 * @brief Routine to initialize topic publishers.
 */
void ROSlamNode::init_publishers()
{
  // // UWB array
  // uwb_array_pub_ = this->create_publisher<ro_slam_interfaces::msg::UWBArray>(
  //   "~/uwb_tag",
  //   rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize callback groups.
 */
void ROSlamNode::init_callback_groups()
{
  uwb_array_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  odometry_cgroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize subscribers.
 */
void ROSlamNode::init_subscribers()
{
  // Odometry
  auto odometry_opts = rclcpp::SubscriptionOptions();
  odometry_opts.callback_group = odometry_cgroup_;
  odometry_sub_ = this->create_subscription<JointState>(
    "/joint_states",
    rclcpp::QoS(1).best_effort(),
    std::bind(
      &ROSlamNode::odometry_clbk,
      this,
      std::placeholders::_1),
    odometry_opts);

  // UWB Array
  auto uwb_array_opts = rclcpp::SubscriptionOptions();
  uwb_array_opts.callback_group = uwb_array_cgroup_;
  uwb_array_sub_ = this->create_subscription<UwbArray>(
    "/uwb_tag",
    rclcpp::QoS(1).best_effort(),
    std::bind(
      &ROSlamNode::uwb_array_clbk,
      this,
      std::placeholders::_1),
    uwb_array_opts);
}

/**
 * @brief Routine to initialize timers.
 */
void ROSlamNode::init_timers()
{
  // visualization_timer_ = this->create_wall_timer(
  //   std::chrono::duration<double>(5.0),
  //   std::bind(
  //       &ROSlamNode::visualization_timer_clbk,
  //       this));
}

} // namespace ROSlam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ROSlam::ROSlamNode)
