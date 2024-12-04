/**
 * MeanVar topic subscription callbacks.
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

#include <algorithm>

#include <mean_var/mean_var.hpp>

namespace MeanVar
{

/**
 * @brief Logs drone's local pose from PX4's EKF2.
 *
 * @param msg Pose message to parse.
 */
void MeanVarNode::uwb_clbk(const UwbArray::ConstSharedPtr msg)
{
  Eigen::ArrayXd imu_measures(6);

  float dist = msg->uwbs[0].dist;

  N += 1;
  float delta = dist - mean;
  mean += delta / N;
  float delta2 = dist - mean;
  M2 += delta * delta2;
  var = M2 / N;

  std::cout << "N = " << N << "\tmean = " << mean << "\tvar = " << var << std::endl;
}

} // namespace MeanVar
