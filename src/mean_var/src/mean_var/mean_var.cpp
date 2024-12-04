/**
 * MeanVar node initialization and parameters routines.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * January 11, 2023
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

#include <mean_var/mean_var.hpp>

namespace MeanVar
{

/**
 * @brief MeanVar node constructor.
 *
 * @param node_opts Options for the base node.
 */
MeanVarNode::MeanVarNode(const rclcpp::NodeOptions & node_options)
: Node("mean_var", node_options)
{
  init_cgroups();
  init_subscriptions();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief MeanVar node destructor.
 */
MeanVarNode::~MeanVarNode() {}

/**
 * @brief Routine to initialize callback groups.
 */
void MeanVarNode::init_cgroups()
{
  // Topic subscriptions
  uwb_clbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

/**
 * @brief Routine to initialize topic subscriptions.
 */
void MeanVarNode::init_subscriptions()
{
  // UwbArray
  auto uwb_sub_opt = rclcpp::SubscriptionOptions();
  uwb_sub_opt.callback_group = uwb_clbk_group_;
  uwb_sub_ = this->create_subscription<UwbArray>(
    "/uwb",
    1,
    std::bind(
      &MeanVarNode::uwb_clbk,
      this,
      std::placeholders::_1),
    uwb_sub_opt);
}

} // namespace MeanVar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MeanVar::MeanVarNode)