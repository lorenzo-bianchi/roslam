/**
 * FedEkf headers.
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

#ifndef ROSLAM__FED_EKF_HPP
#define ROSLAM__FED_EKF_HPP

#define MAX_STEPS 5000

#include <algorithm>
#include <atomic>
#include <Eigen/Dense>
#include <iostream>
#include <numeric>
#include <vector>

#define UNUSED(arg) (void)(arg)
#define LINE std::cout << __FUNCTION__ << ", LINE: " << __LINE__ << std::endl;

using namespace Eigen;

namespace ROSlam
{

/**
 * Data struct definition.
 */
typedef struct
{
  uint8_t n_tags;                   // Number of tags
  uint8_t n_phi;                    // Number oh hypotheses
  // Stadard deviations
  double sigma_phi;                 // Phi standard deviation
  double sigma_range;               // Range measurement standard deviation
  // Pruning
  bool use_pruning;                 // Pruning flag
  uint8_t min_zeros_start_pruning;  // Minimum number of zeros to start pruning
  uint16_t step_start_pruning;      // Step to start pruning
  // "ICP" parameters
  uint16_t num_iterations;          // Number of iterations
  double distance_threshold;        // Distance threshold
  double percent_min_inliers;       // Minimum percentage of inliers
  // Reset parameters
  bool use_reset;                   // Reset flag
  uint16_t min_steps_reset;         // Minimum numbers of steps to reset
} fed_ekf_data_t;

/**
 * @brief FedEkf node class.
 */
class FedEkf
{
public:
  FedEkf() = default;
  FedEkf(Vector3d x0, fed_ekf_data_t data);

private:
  //Initial conditions
  Vector3d x0_;
  // Data
  fed_ekf_data_t data_;
  uint8_t n_tags_;
  uint8_t n_phi_max_;
  VectorXd phi_angles_;
  // Stadard deviations
  double sigma_phi_;
  double sigma_range_;
  // Pruning
  bool use_pruning_;
  uint8_t min_zeros_start_pruning_;
  uint16_t step_start_pruning_;
  // "ICP" parameters
  uint16_t num_iterations_;
  double distance_threshold_;
  double percent_min_inliers_;
  // Reset parameters
  bool use_reset_;
  uint16_t min_steps_reset_;

  uint8_t n_reset = 0;
  bool use_reset = false;

  // Matrices
  RowVectorXd n_phi_vett_;
  VectorXd innovation_, innovation_x_, innovation_y_;
  MatrixXd weights_;
  MatrixXd x_hat_slam_;
  VectorXd x_hat_slam_minus_;
  ArrayXd x_hat_indices_, x_hat_cumul_indices_;
  MatrixXd P_, P_tag_;
  MatrixXd F_;
  MatrixXd W_;
  MatrixXd H_, Hx_, Hy_;
  MatrixXd Rs_, Rs_x_, Rs_y_;
  VectorXd hyps_steps_start_pruning_;
  VectorXd var_x, var_y, cov_xy;

};

} // namespace ROSlam

#endif // ROSLAM__FED_EKF_HPP