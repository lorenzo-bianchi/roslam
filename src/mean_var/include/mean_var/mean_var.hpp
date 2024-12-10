/**
 * MeanVar module headers.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 * Roberto Masocco <robmasocco@gmail.com>
 * Intelligent Systems Lab <isl.torvergata@gmail.com>
 *
 * May 20, 2022
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

#ifndef MEAN_VAR_HPP
#define MEAN_VAR_HPP

#include <cstdio>
#include <cfloat>
#include <chrono>
#include <vector>

#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <ro_slam_interfaces/msg/uwb_array.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace rcl_interfaces::msg;
using namespace ro_slam_interfaces::msg;

#define UNUSED(arg) (void)(arg)

namespace MeanVar
{

class MeanVarNode : public rclcpp::Node
{
public:
  MeanVarNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~MeanVarNode();

private:
  /* Node init functions */
  void init_cgroups();
  void init_subscriptions();

  /* Subscribers */
  rclcpp::Subscription<UwbArray>::SharedPtr uwb_sub_;

  /* Callback groups */
  rclcpp::CallbackGroup::SharedPtr uwb_clbk_group_;

  /* Callback functions */
  void uwb_clbk(const UwbArray::ConstSharedPtr msg);

  /* Internal variables */
  int N = 0;
  float M2 = 0.0f;
  float mean = 0.0f;
  float var = 0.0f;
  float hz = 0.0f;
  bool first_msg = true;
  rclcpp::Time start;
};

} // namespace MeanVar

#endif // MEAN_VAR_HPP
