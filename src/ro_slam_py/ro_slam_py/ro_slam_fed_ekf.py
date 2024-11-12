import numpy as np
from numpy import pi, ceil
from numpy.linalg import multi_dot, pinv
from dataclasses import dataclass

from .ro_slam_roma2d import roma2d

mtype = np.float32
pose_type = np.dtype([('x', mtype), ('y', mtype), \
                      ('var_x', mtype), ('var_y', mtype), ('cov_xy', mtype), \
                      ('last_hyp', bool)])

@dataclass
class FedEkfData:
    n_tags: int = 0                     # Number of tags
    n_phi: int = 0                      # Number of hypotheses
    # Standard deviations
    sigma_phi: float = 0.0              # Phi standard deviation
    sigma_range: float = 0.0            # Range measurement standard deviation
    # Pruning
    min_zeros_start_pruning: int = 0    # Minimum number of zeros to start pruning
    # ICP parameters
    num_iterations: int = 0             # Number of iterations
    distance_threshold: float = 0.0     # Distance threshold
    percent_min_inliers: float = 0.0    # Minimum percentage of inliers
    # Reset parameters
    min_steps_reset: int = 0            # Minimum numbers of steps to reset
    min_tags_after_reset: int = 0       # Minimum number of converged tags after reset
    #
    wheels_separation: float = 0.0      # Wheels separation
    kr: float = 0.0                     # Unsystematic odometric error
    kl: float = 0.0                     # Unsystematic odometric error

@dataclass
class FedEkfSharedData:
    robot_id: int = -1                  # Robot ID
    tags_positions: np.ndarray = None   # Tags positions
    tags_vars: np.ndarray = None        # Tags variances

class FedEkf:
    def __init__(self, x0: np.ndarray, data: FedEkfData):
        """
        Federated EKF constructor
        """
        np.set_printoptions(suppress=True, edgeitems=18, linewidth=1000, precision=4)

        # Initialize variables
        self.k = -1
        self.n_reset = 0
        self.do_reset = False
        self.first_correction = True

        # Get values from data
        self.n_tags = data.n_tags
        self.n_phi_max = data.n_phi
        self.sigma_phi = data.sigma_phi
        self.sigma_range = data.sigma_range
        self.min_zeros_start_pruning = data.min_zeros_start_pruning
        self.num_iterations = data.num_iterations
        self.distance_threshold = data.distance_threshold
        self.percent_min_inliers = data.percent_min_inliers
        self.min_steps_reset = data.min_steps_reset
        self.wheels_separation = data.wheels_separation
        self.kr = data.kr
        self.kl = data.kl
        self.min_tags_after_reset = data.min_tags_after_reset

        self.phi_angles = np.linspace(-pi + 2 * pi / self.n_phi_max, pi, self.n_phi_max)

        # Define matrices
        dim_h = 3 + self.n_phi_max              # hypothesis dimension
        dim = 3 + dim_h * self.n_tags           # state dimension
        n_hyps = self.n_tags * self.n_phi_max   # total number of hypotheses

        self.n_phi_vett = self.n_phi_max * np.ones((self.n_tags, ), dtype=np.uint16)
        self.innovation = np.zeros((n_hyps, ), dtype=mtype)
        self.weights = np.ones((self.n_tags, self.n_phi_max), dtype=mtype) / self.n_phi_max
        self.x_hat_slam = np.zeros((dim, ), dtype=mtype)
        self.P = np.zeros((dim, dim), dtype=mtype)
        self.F = np.eye(dim, dtype=mtype)
        self.W = np.zeros((dim, 2), dtype=mtype)
        self.H = np.zeros((n_hyps, dim), dtype=mtype)
        self.Rs = np.zeros((n_hyps, n_hyps), dtype=mtype)
        self.x_hat_indices = np.concatenate(([0, 3],
                                             dim_h * np.ones((self.n_tags, ), dtype=np.uint16)))
        self.x_hat_cumul_indices = np.cumsum(self.x_hat_indices[:-1])
        self.Hx = np.zeros((n_hyps, dim), dtype=mtype)
        self.Hy = np.zeros((n_hyps, dim), dtype=mtype)
        self.innovation_x = np.zeros((n_hyps, ), dtype=mtype)
        self.innovation_y = np.zeros((n_hyps, ), dtype=mtype)
        self.Rs_x = np.zeros((n_hyps, n_hyps), dtype=mtype)
        self.Rs_y = np.zeros((n_hyps, n_hyps), dtype=mtype)
        self.hyps_steps_start_pruning = np.zeros((self.n_tags, ), dtype=np.uint16)       # obj.startPruning

        # Initialization
        self.x_hat_slam[0:3] = x0
        P_tag = np.diag(np.concatenate((
            [0, 0, self.sigma_range ** 2],
            self.sigma_phi ** 2 * np.ones((self.n_phi_max, ), dtype=mtype))))
        for idx in range(self.n_phi_max):
            self.x_hat_slam[6+idx::dim_h] = self.phi_angles[idx]
        for idx in range(self.n_tags):
            self.P[3+dim_h*idx:3+dim_h*(idx+1),
                   3+dim_h*idx:3+dim_h*(idx+1)] = P_tag

    def prediction(self, vel_wheel_right: float, vel_wheel_left: float):
        self.k += 1

        vel_robot = (vel_wheel_right + vel_wheel_left) / 2.0
        omega_robot = (vel_wheel_right - vel_wheel_left) / self.wheels_separation

        cosk, sink = np.cos(self.x_hat_slam[2]), np.sin(self.x_hat_slam[2])
        half_cosk, half_sink = 0.5 * cosk, 0.5 * sink

        # Update state
        self.x_hat_slam[0] += vel_robot * cosk
        self.x_hat_slam[1] += vel_robot * sink
        self.x_hat_slam[2] += omega_robot

        # Update jacobian matrix F = df/dx
        self.F[0:2, 2] = np.array([-vel_robot * sink, vel_robot * cosk], dtype=mtype)

        # Update jacobian W = df/dw
        self.W[0, 0] = half_cosk
        self.W[0, 1] = half_cosk
        self.W[1, 0] = half_sink
        self.W[1, 1] = half_sink
        self.W[2, 0] =  1.0 / self.wheels_separation
        self.W[2, 1] = -1.0 / self.wheels_separation

        # Odometric error covariance matrix
        Q = np.diag([self.kr * np.abs(vel_wheel_right), self.kl * np.abs(vel_wheel_left)])

        # P computation
        self.P = multi_dot([self.F, self.P, self.F.T]) + multi_dot([self.W, Q, self.W.T])

        print('Prediction step done')

    def correction(self, distances: np.ndarray):
        if self.first_correction:
            # Initialize state vector with ranges
            self.first_correction = False
            self.x_hat_slam[3::3+self.n_phi_max] = self.x_hat_slam[0]
            self.x_hat_slam[4::3+self.n_phi_max] = self.x_hat_slam[1]
            self.x_hat_slam[5::3+self.n_phi_max] = distances

        self.last_distances = distances
        n_tags = self.n_tags

        # A priori robot position
        x_robot, y_robot = self.x_hat_slam[0], self.x_hat_slam[1]

        idx_mat_cumul = np.cumsum(np.concatenate(([0], self.n_phi_vett[:-1])))

        for idx_tag in range(n_tags):
            idx_mat = idx_mat_cumul[idx_tag]

            n_phi = self.n_phi_vett[idx_tag]
            idx_0 = self.x_hat_cumul_indices[idx_tag+1]
            x_i, y_i, rho_i = self.x_hat_slam[idx_0:idx_0+3]

            prob_distances = np.zeros((n_phi, ), dtype=mtype)

            phi = self.x_hat_slam[3+idx_0:3+idx_0+n_phi]
            cos_phi, sin_phi = np.cos(phi), np.sin(phi)

            x_tag = x_i + rho_i * cos_phi
            y_tag = y_i + rho_i * sin_phi

            delta_x = x_robot - x_tag
            delta_y = y_robot - y_tag

            distances_hat = np.sqrt(delta_x**2 + delta_y**2)
            delta_distances = distances[idx_tag] - distances_hat

            self.innovation[idx_mat:idx_mat+n_phi] = delta_distances

            prob_distances = np.exp(-0.5 * delta_distances**2 / self.sigma_range**2)
            prob_distances = np.maximum(prob_distances, 1e-100)
            self.weights[idx_tag, :n_phi] *= prob_distances

            lambdas = prob_distances / np.sum(prob_distances)

            # Update H matrix
            self.H[idx_mat:idx_mat+n_phi, 0:2] = \
                np.vstack((delta_x, delta_y)).T / distances_hat[:, None]
            self.H[idx_mat:idx_mat+n_phi, 0+idx_0] = -delta_x / distances_hat
            self.H[idx_mat:idx_mat+n_phi, 1+idx_0] = -delta_y / distances_hat
            self.H[idx_mat:idx_mat+n_phi, 2+idx_0] = \
                (-delta_x * cos_phi - delta_y * sin_phi) / distances_hat
            self.H[idx_mat + np.arange(n_phi), 3 + idx_0 + np.arange(n_phi)] = \
                rho_i * (delta_x * sin_phi - delta_y * cos_phi) / distances_hat

            # Update Rs matrix
            self.Rs[idx_mat:idx_mat+n_phi, idx_mat:idx_mat+n_phi] = \
                    np.diag(self.sigma_range**2 / np.maximum(0.0001, lambdas))

        # Update a posteriori estimate
        kalman_gain = multi_dot([self.P, self.H.T, \
                                 pinv(multi_dot([self.H, self.P, self.H.T]) + self.Rs)])
        self.x_hat_slam += kalman_gain @ self.innovation
        I = np.eye(np.sum(self.x_hat_indices), dtype=mtype)
        self.P = (I - kalman_gain @ self.H) @ self.P

        # Update weights
        self.weights /= np.sum(self.weights, axis=1, keepdims=True)

        print('Correction step done')

    def delete_rows_cols(self, idx_tag: int, idx_phi: int):
        temp = self.weights[idx_tag, idx_phi+1:]
        self.weights[idx_tag, idx_phi:idx_phi+temp.shape[0]] = temp
        self.weights[idx_tag, -1] = 0

        self.x_hat_indices[2+idx_tag] -= 1
        self.x_hat_cumul_indices[2+idx_tag:] -= 1
        self.n_phi_vett[idx_tag] -= 1

        idx_0 = self.x_hat_cumul_indices[idx_tag+1]
        i = idx_0 + 3 + idx_phi
        self.P = np.delete(np.delete(self.P, i, axis=0), i, axis=1)
        self.x_hat_slam = np.delete(self.x_hat_slam, i, axis=0)

    def pruning(self):
        n_tags = self.n_tags
        change = False

        # Check if at least min_zeros_start_pruning hypotheses have weights below threshold
        if np.any(self.hyps_steps_start_pruning == 0):
            for idx_tag in range(n_tags):
                if self.hyps_steps_start_pruning[idx_tag] > 0:
                    continue

                n_phi = self.n_phi_vett[idx_tag]
                n_zeros = 0
                for idx_phi in range(n_phi):
                    if self.weights[idx_tag, idx_phi] < 0.00001 / n_phi:
                        n_zeros += 1

                if n_zeros >= self.min_zeros_start_pruning:
                    self.hyps_steps_start_pruning[idx_tag] = self.k

        for idx_tag in range(n_tags):
            if self.hyps_steps_start_pruning[idx_tag] == 0:
                continue

            # If previous check is true, start pruning removing rows and cols from matrices
            n_phi = self.n_phi_vett[idx_tag]
            idx_phi = 0
            while idx_phi < n_phi:
                if self.weights[idx_tag, idx_phi] < 0.00001 / n_phi:
                    change = True
                    n_phi -= 1

                    self.delete_rows_cols(idx_tag, idx_phi)
                else:
                    idx_phi += 1

            idx_0 = self.x_hat_cumul_indices[idx_tag+1]

            # If there are two hypoteses left, check if they are close
            if n_phi == 2:
                phi1 = self.x_hat_slam[2 + idx_0 + 1]
                phi2 = self.x_hat_slam[2 + idx_0 + 2]

                weight1 = self.weights[idx_tag, 0]
                weight2 = self.weights[idx_tag, 1]

                min_w = 0.9
                w = np.maximum(min_w, -(1-min_w) / 6000 * self.k + 1)
                if np.maximum(weight1, weight2) > w and np.abs(phi1 - phi2) < 5 * pi / 180.0:
                    change = True
                    self.delete_rows_cols(idx_tag, 1 if weight1 > weight2 else 0)

        if change:
            self.reshape_matrices(pruning=True)

    def get_robot_pose(self) -> np.ndarray:
        return self.x_hat_slam[0:3]

    def get_tags_poses(self) -> np.ndarray:
        n_tags = self.n_tags
        tags_poses = np.zeros((n_tags, ), dtype=pose_type)
        for idx_tag in range(n_tags):
            n_phi = self.n_phi_vett[idx_tag]
            idx_0 = self.x_hat_cumul_indices[idx_tag+1]
            x_i, y_i, rho_i = self.x_hat_slam[idx_0:idx_0+3]

            idx_max_weight = np.argmax(self.weights[idx_tag, :])
            phi_max_weight = self.x_hat_slam[3+idx_0+idx_max_weight]

            cos_phi_max_weight, sin_phi_max_weight = np.cos(phi_max_weight), np.sin(phi_max_weight)

            x_hat_tag = x_i + rho_i * cos_phi_max_weight
            y_hat_tag = y_i + rho_i * sin_phi_max_weight

            # Compute variances
            idx_x = 0 + idx_0
            idx_y = 1 + idx_0
            idx_r = 2 + idx_0
            idx_p = 3 + idx_0 + idx_max_weight

            rho_i = self.x_hat_slam[idx_r]
            phi_ij = self.x_hat_slam[idx_p]
            cos_phi_ij, sin_phi_ij = np.cos(phi_ij), np.sin(phi_ij)

            var_x0  = self.P[idx_x, idx_x]
            var_y0  = self.P[idx_y, idx_y]
            var_rho = self.P[idx_r, idx_r]
            var_phi = self.P[idx_p, idx_p]

            cov_x_rho   = self.P[idx_x, idx_r]
            cov_x_phi   = self.P[idx_x, idx_p]
            cov_y_rho   = self.P[idx_y, idx_r]
            cov_y_phi   = self.P[idx_y, idx_p]
            cov_rho_phi = self.P[idx_r, idx_p]

            var_x = var_x0 + cos_phi_ij**2 * var_rho + rho_i**2 * sin_phi_ij**2 * var_phi + \
                    2 * cos_phi_ij * cov_x_rho - 2 * rho_i * sin_phi_ij * cov_x_phi - \
                    2 * rho_i * cos_phi_ij * sin_phi_ij * cov_rho_phi
            var_y = var_y0 + sin_phi_ij**2 * var_rho + rho_i**2 * cos_phi_ij**2 * var_phi + \
                    2 * sin_phi_ij * cov_y_rho + 2 * rho_i * cos_phi_ij * cov_y_phi + \
                    2 * rho_i * cos_phi_ij * sin_phi_ij * cov_rho_phi
            cov_xy = 0.0

            last_hyp = True if n_phi == 1 else False

            tags_poses[idx_tag] = (x_hat_tag, y_hat_tag, var_x, var_y, cov_xy, last_hyp)

        return tags_poses

    def correction_shared(self, this_robot, other_robots: list):
        n_other_robots = len(other_robots)
        if n_other_robots < 2:
            return

        n_tags = self.n_tags
        empty = True
        good_inliers = np.zeros((2, n_tags, n_other_robots), dtype=bool)
        for robot1 in range(n_other_robots-1):
            tags1 = other_robots[robot1].tags_positions
            for robot2 in range(robot1+1, n_other_robots):
                tags2 = other_robots[robot2].tags_positions
                _, _, inliers = roma2d(tags1.T, tags2.T,
                                       self.num_iterations, self.distance_threshold,
                                       round(self.percent_min_inliers * n_tags))

                if inliers.size > 0:
                    empty = False
                    good_inliers[:, inliers, robot1] = True
                    good_inliers[:, inliers, robot2] = True

        if empty:
            return

        pos_tags_robot = np.zeros((2, n_tags, n_other_robots), dtype=mtype)
        vars = np.zeros((2, n_tags, n_other_robots), dtype=mtype)

        to_be_used = []
        to_be_removed = []
        for idx in range(n_other_robots):
            other_robots_tags = other_robots[idx].tags_positions
            R, t, inliers = roma2d(other_robots_tags.T, this_robot.tags_positions.T,
                                   self.num_iterations, self.distance_threshold,
                                   round(self.percent_min_inliers * n_tags))
            if inliers.size == 0:
                to_be_removed.append(idx)
                continue
            to_be_used.append(idx)

            # Apply rototrnslation to other robot tags
            pos_tags_robot[:, :, idx] = R @ other_robots_tags + t[:, None]

            # Apply rotation to other robot variances
            vars_rot = np.zeros((2, n_tags), dtype=mtype)
            for idx_tag in range(n_tags):
                var_x, var_y, cov_xy = other_robots[idx].tags_vars[0:3, idx_tag]

                sigma = np.array([[var_x, cov_xy], [cov_xy, var_y]], dtype=mtype)
                sigma_rot = multi_dot([R, sigma, R.T])

                vars_rot[:, idx_tag] = sigma_rot[0, 0], sigma_rot[1, 1]
            vars[:, :, idx] = vars_rot

        pos_tags_robot = pos_tags_robot[:, :, to_be_used]
        vars = vars[:, :, to_be_used]

        # Check reset
        if pos_tags_robot.shape[2] == 0:
            if np.sum(self.n_phi_vett == 1) >= self.min_tags_after_reset:
                if other_robots.shape[0] > 1 or other_robots[0].robot_id != self.id:
                    self.n_reset += 1
                if self.n_reset >= self.min_steps_reset:
                    self.do_reset = True

            return
        self.n_reset = 0

        # vars = (0 * vars + 1);    % FIXME
        # temp = 1 ./ vars;
        # W_ = temp ./ sum(temp, 3);

        good_inliers = np.delete(good_inliers, to_be_removed, axis=2)
        den = np.sum(good_inliers, axis=2)
        den[den == 0] = 1
        W = good_inliers / den[:, :, None]
        measures_weighted = np.sum(W * pos_tags_robot, axis=2)

        tags_to_use = np.sum(W[0, :, :], axis=1)
        if np.all(tags_to_use == 0):
            return

        for idx_pos in range(1):
            idx_mat_cumul = np.cumsum(np.concatenate(([0], self.n_phi_vett[:-1])))
            change = 0
            for idx_tag in range(n_tags):
                n_phi = self.n_phi_vett[idx_tag]
                idx_mat = idx_mat_cumul[idx_tag] - change
                if tags_to_use[idx_tag] == 0:
                    change += n_phi
                    indices_to_delete = np.arange(n_phi) + idx_mat  # modifica come necessario

                    self.Hx = np.delete(self.Hx, indices_to_delete, axis=0)
                    self.Hy = np.delete(self.Hy, indices_to_delete, axis=0)
                    self.innovation_x = np.delete(self.innovation_x, indices_to_delete, axis=0)
                    self.innovation_y = np.delete(self.innovation_y, indices_to_delete, axis=0)

                    self.Rs_x = np.delete(
                        np.delete(self.Rs_x, indices_to_delete, axis=0), indices_to_delete, axis=1)
                    self.Rs_y = np.delete(
                        np.delete(self.Rs_y, indices_to_delete, axis=0), indices_to_delete, axis=1)

                    continue

                var = 0.05
                fused_var_x = var
                fused_var_y = var
                # fused_var_x = 100 * vars(1, idx_tag, idx_pos)
                # fused_var_y = 100 * vars(2, idx_tag, idx_pos)
                # fused_var_x = 1 / np.sum(1 / vars[0, idx_tag, :])
                # fused_var_y = 1 / np.sum(1 / vars[1, idx_tag, :])

                sigma_x = np.sqrt(fused_var_x)
                sigma_y = np.sqrt(fused_var_y)

                idx_0 = self.x_hat_cumul_indices[idx_tag+1]
                x_i, y_i, rho_i = self.x_hat_slam[idx_0:3+idx_0]

                measure_x, measure_y = measures_weighted[0:2, idx_tag]

                prob_distances_x = np.zeros((n_phi, ), dtype=mtype)
                prob_distances_y = np.zeros((n_phi, ), dtype=mtype)

                phi = self.x_hat_slam[3+idx_0:3+idx_0+n_phi]
                cos_phi, sin_phi = np.cos(phi), np.sin(phi)

                x_tag = x_i + rho_i * cos_phi
                y_tag = y_i + rho_i * sin_phi

                delta_x = measure_x - x_tag
                delta_y = measure_y - y_tag

                self.innovation_x[idx_mat:idx_mat+n_phi] = delta_x
                self.innovation_y[idx_mat:idx_mat+n_phi] = delta_y

                prob_distances_x = np.exp(-0.5 * delta_x**2 / sigma_x**2)
                prob_distances_x = np.maximum(prob_distances_x, 1e-100)
                prob_distances_y = np.exp(-0.5 * delta_y**2 / sigma_y**2)
                prob_distances_y = np.maximum(prob_distances_y, 1e-100)

                self.weights[idx_tag, :n_phi] *= prob_distances_x * prob_distances_y

                self.Hx[idx_mat:idx_mat+n_phi, 0+idx_0] = 1
                self.Hx[idx_mat:idx_mat+n_phi, 2+idx_0] = cos_phi
                self.Hx[idx_mat + np.arange(n_phi), 3 + idx_0 + np.arange(n_phi)] = -rho_i * sin_phi
                self.Hy[idx_mat:idx_mat+n_phi, 1+idx_0] = 1
                self.Hy[idx_mat:idx_mat+n_phi, 2+idx_0] = sin_phi
                self.Hy[idx_mat + np.arange(n_phi), 3 + idx_0 + np.arange(n_phi)] =  rho_i * cos_phi

                lambdas_x = prob_distances_x / np.sum(prob_distances_x)
                lambdas_y = prob_distances_y / np.sum(prob_distances_y)

                self.Rs_x[idx_mat:idx_mat+n_phi, idx_mat:idx_mat+n_phi] = \
                    np.diag(sigma_x ** 2 / np.maximum(0.0001, lambdas_x))
                self.Rs_y[idx_mat:idx_mat+n_phi, idx_mat:idx_mat+n_phi] = \
                    np.diag(sigma_y ** 2 / np.maximum(0.0001, lambdas_y))

        # Update a posteriori estimate
        H = np.vstack((self.Hx, self.Hy))
        zeros_xy = np.zeros((self.Rs_x.shape[0], self.Rs_y.shape[1]), dtype=mtype)
        zeros_yx = np.zeros((self.Rs_y.shape[0], self.Rs_x.shape[1]), dtype=mtype)
        Rs = np.block([[self.Rs_x, zeros_xy], [zeros_yx, self.Rs_y]])
        innovation_tot = np.hstack((self.innovation_x, self.innovation_y))
        kalman_gain = multi_dot([self.P, H.T, pinv(multi_dot([H, self.P, H.T]) + Rs)])
        self.x_hat_slam += kalman_gain @ innovation_tot
        I = np.eye(np.sum(self.x_hat_indices), dtype=mtype)
        self.P = (I - kalman_gain @ H) @ self.P

        # Update weights
        self.weights /= np.sum(self.weights, axis=1, keepdims=True)

        if change > 0:
            self.reshape_matrices(pruning=False)


    def reshape_matrices(self, pruning: bool = True):
        n_phi_tag_new = np.sum(self.n_phi_vett)
        state_len_new = np.sum(self.x_hat_indices)

        if pruning:
            self.innovation = np.zeros((n_phi_tag_new, ), dtype=mtype)

            self.F = np.eye(state_len_new, dtype=mtype)
            self.W = np.zeros((state_len_new, 2), dtype=mtype)
            self.H = np.zeros((n_phi_tag_new, state_len_new), dtype=mtype)
            self.Rs = np.zeros((n_phi_tag_new, n_phi_tag_new), dtype=mtype)

        self.Hx = np.zeros((n_phi_tag_new, state_len_new), dtype=mtype)
        self.Hy = np.zeros((n_phi_tag_new, state_len_new), dtype=mtype)
        self.innovation_x = np.zeros((n_phi_tag_new, ), dtype=mtype)
        self.innovation_y = np.zeros((n_phi_tag_new, ), dtype=mtype)
        self.Rs_x = np.zeros((n_phi_tag_new, n_phi_tag_new), dtype=mtype)
        self.Rs_y = np.zeros((n_phi_tag_new, n_phi_tag_new), dtype=mtype)

    def reset(self):
        # Define matrices
        dim_h = 3 + self.n_phi_max              # hypothesis dimension
        dim = 3 + dim_h * self.n_tags           # state dimension
        n_hyps = self.n_tags * self.n_phi_max   # total number of hypotheses

        self.n_phi_vett = self.n_phi_max * np.ones((self.n_tags, ), dtype=np.uint16)

        self.innovation = np.zeros((n_hyps, ), dtype=mtype)
        self.weights = np.ones((self.n_tags, self.n_phi_max), dtype=mtype) / self.n_phi_max

        self.P = np.zeros((dim, dim), dtype=mtype)
        self.F = np.eye(dim, dtype=mtype)
        self.W = np.zeros((dim, 2), dtype=mtype)
        self.H = np.zeros((n_hyps, dim), dtype=mtype)
        self.Rs = np.zeros((n_hyps, n_hyps), dtype=mtype)

        self.x_hat_indices = np.concatenate(([0, 3],
                                             dim_h * np.ones((self.n_tags, ), dtype=np.uint16)))
        self.x_hat_cumul_indices = np.cumsum(self.x_hat_indices[:-1])

        self.Hx = np.zeros((n_hyps, dim), dtype=mtype)
        self.Hy = np.zeros((n_hyps, dim), dtype=mtype)
        self.innovation_x = np.zeros((n_hyps, ), dtype=mtype)
        self.innovation_y = np.zeros((n_hyps, ), dtype=mtype)
        self.Rs_x = np.zeros((n_hyps, n_hyps), dtype=mtype)
        self.Rs_y = np.zeros((n_hyps, n_hyps), dtype=mtype)

        self.x_hat_slam = np.zeros((dim, ), dtype=mtype)
        self.x_hat_slam[0:3] = np.array([0.0, 0.0, 0.0], dtype=mtype)
        for idx in range(self.n_phi_max):
            self.x_hat_slam[6+idx::dim_h] = self.phi_angles[idx]
        self.x_hat_slam[5::3+self.n_phi_max] = self.last_distances

        P_tag = np.diag(np.concatenate((
            [0, 0, self.sigma_range ** 2],
            self.sigma_phi ** 2 * np.ones((self.n_phi_max, ), dtype=mtype))))
        for idx in range(self.n_tags):
            self.P[3+dim_h*idx:3+dim_h*(idx+1), 3+dim_h*idx:3+dim_h*(idx+1)] = P_tag

        self.hyps_steps_start_pruning = np.zeros((self.n_tags, ), dtype=np.uint16)

        # self.min_steps_reset += 

        self.n_reset = 0
        self.do_reset = False


# Test case
if __name__ == '__main__':
    np.set_printoptions(suppress=True, edgeitems=18, linewidth=1000, precision=4)

    data = FedEkfData()
    data.n_tags = 3
    data.n_phi = 4
    data.sigma_phi = 2 * pi / (1.5 * data.n_phi)
    data.sigma_range = 0.2
    data.min_zeros_start_pruning = ceil(0.6 * data.n_phi)
    data.num_iterations = 100
    data.distance_threshold = 0.1
    data.percent_min_inliers = 0.7
    data.min_steps_reset = 100
    data.wheels_separation = 0.16
    data.kr = 0.0001
    data.kl = 0.0001

    x0 = np.array([2.2, 1.1, 1.2], dtype=mtype)

    fed_ekf = FedEkf(x0, data)

    omega_r = 1.0
    omega_l = -1.5
    r = 0.033
    fed_ekf.prediction(omega_r * r, omega_l * r)

    distances = np.array([5.0370, 7.3865, 7.3272])#, 6.8257, 2.5218, 8.6032, 4.4099, 7.3127, 3.0873, 3.8493], dtype=mtype)
    fed_ekf.correction(distances)
