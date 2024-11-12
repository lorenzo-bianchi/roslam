/**
 * FedEkf implementation.
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

#include <ro_slam_cpp/fed_ekf.hpp>

namespace ROSlam
{

/**
 * @brief FedEkf node constructor.
 *
 * @param x0 Initial state.
 * @param data Data struct.
 */
FedEkf::FedEkf(Vector3d x0, fed_ekf_data_t data)
{
  x0_ = x0;

  // Get values from data
  n_tags_ = data.n_tags;
  n_phi_max_ = data.n_phi;
  sigma_phi_ = data.sigma_phi;
  sigma_range_ = data.sigma_range;
  use_pruning_ = data.use_pruning;
  min_zeros_start_pruning_ = data.min_zeros_start_pruning;
  step_start_pruning_ = data.step_start_pruning;
  num_iterations_ = data.num_iterations;
  distance_threshold_ = data.distance_threshold;
  percent_min_inliers_ = data.percent_min_inliers;
  use_reset_ = data.use_reset;
  min_steps_reset_ = data.min_steps_reset;

  phi_angles_ = VectorXd::LinSpaced(n_phi_max_, -M_PI + 2 * M_PI / n_phi_max_, M_PI);

  // Define matrices
  n_phi_vett_ = RowVectorXd::Constant(n_tags_, n_phi_max_);   // FIXME: Check this
  innovation_ = VectorXd::Zero(n_tags_ * n_phi_max_);
  weights_ = VectorXd::Constant(n_tags_ * n_phi_max_, 1 / n_phi_max_);    // FIXME: Check this
  x_hat_slam_ = VectorXd::Zero(3 + (3 + n_phi_max_) * n_tags_, MAX_STEPS);
  P_ = MatrixXd::Zero(3 + (3 + n_phi_max_) * n_tags_, 3 + (3 + n_phi_max_) * n_tags_);

  VectorXd P_tag_diagonal(n_phi_max_ + 3);
  P_tag_diagonal << 0, 0, sigma_range_ * sigma_range_,
                    VectorXd::Constant(sigma_phi_ * sigma_phi_, n_phi_max_);   // FIXME: Check this
  P_tag_ = P_tag_diagonal.asDiagonal();

  x_hat_slam_minus_ = VectorXd::Zero(3 + (3 + n_phi_max_) * n_tags_);
  F_ = MatrixXd::Identity(3 + (3 + n_phi_max_) * n_tags_, 3 + (3 + n_phi_max_) * n_tags_);
  W_ = MatrixXd::Zero(3 + (3 + n_phi_max_) * n_tags_, 2);
  H_ = MatrixXd::Zero(n_tags_ * n_phi_max_, 3 + (3 + n_phi_max_) * n_tags_);
  Rs_ = MatrixXd::Zero(n_tags_ * n_phi_max_, n_tags_ * n_phi_max_);

  x_hat_indices_ << 1, 3, ArrayXd::Constant(n_tags_, 3 + n_phi_max_);   // FIXME: Check this
  std::partial_sum(x_hat_indices_.begin(),
                   x_hat_indices_.end(),
                   x_hat_cumul_indices_.begin(),
                   std::plus<double>());

  Hx_ = MatrixXd::Zero(n_tags_ * n_phi_max_, 3 + (3 + n_phi_max_) * n_tags_);
  Hy_ = MatrixXd::Zero(n_tags_ * n_phi_max_, 3 + (3 + n_phi_max_) * n_tags_);
  innovation_x_ = VectorXd::Zero(n_tags_ * n_phi_max_);
  innovation_y_ = VectorXd::Zero(n_tags_ * n_phi_max_);
  Rs_x_ = MatrixXd::Zero(n_tags_ * n_phi_max_, n_tags_ * n_phi_max_);
  Rs_y_ = MatrixXd::Zero(n_tags_ * n_phi_max_, n_tags_ * n_phi_max_);

  // Initialization
  x_hat_slam_.block(0, 0, 3, 1) = x0_;
  // obj.xHatSLAM(4:nPhiMax+3:end, 1) = obj.xHatSLAM(1,1)
  for (int i = 3; i < x_hat_slam_.rows(); i += (n_phi_max_ + 3)) {
    x_hat_slam_(i, 0) = x_hat_slam_(0, 0);
  }
  // obj.xHatSLAM(5:nPhiMax+3:end, 1) = obj.xHatSLAM(2,1)
  for (int i = 4; i < x_hat_slam_.rows(); i += (n_phi_max_ + 3)) {
    x_hat_slam_(i, 0) = x_hat_slam_(1, 0);
  }
  // for jndPhi = 1:nPhiMax
  //   obj.xHatSLAM(6+jndPhi:nPhiMax+3:end,1) = data.possibiliPhi(jndPhi);
  // end
  for (int j = 0; j < n_phi_max_; j++) {
    for (int i = 5 + j; i < x_hat_slam_.rows(); i += (n_phi_max_ + 3)) {
      x_hat_slam_(i, 0) = phi_angles_[j];
    }
  }
  // for indTag = 1:nTag
  //   obj.P(4+(3+nPhiMax)*(indTag-1) : 3+(3+nPhiMax)*indTag, 4+(3+nPhiMax)*(indTag-1) : 3+(3+nPhiMax)*indTag) = obj.Ptag;
  // end
  for (int i = 0; i < n_tags_; i++) {       // FIXME: Check this
    int start = 4 + (3 + n_phi_max_) * i;
    P_.block(start, start, 3 + n_phi_max_, 3 + n_phi_max_) = P_tag_;
  }

  hyps_steps_start_pruning_ = VectorXd::Zero(n_tags_);

  var_x = VectorXd::Zero(n_tags_);
  var_y = VectorXd::Zero(n_tags_);
  cov_xy = VectorXd::Zero(n_tags_);

}

} // namespace ROSlam