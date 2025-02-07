% Define quadrilaterals (each row is a vertex [x, y])
quad1 = pos_anchors; % Reference
quad2 = anchors_poses_world; % Rotated and scaled quadrilateral

% ---- Normalize and center ----
centroid1 = mean(quad1, 1);
centroid2 = mean(quad2, 1);
Q1 = quad1 - centroid1;
Q2 = quad2 - centroid2;

% Compute optimal scaling factor
scale_opt = sum(vecnorm(Q1, 2, 2)) / sum(vecnorm(Q2, 2, 2));
Q2_scaled = Q2 * scale_opt;

% ---- Find optimal rotation angle using fminbnd ----
rmse_func = @(theta) sqrt(mean(vecnorm(( [cos(theta), -sin(theta); sin(theta), cos(theta)] * Q2_scaled')' - Q1, 2, 2).^2));
theta_opt = fminbnd(rmse_func, -pi, pi); % Search in [-π, π]

% ---- Compute final rotation matrix ----
R_opt = [cos(theta_opt), -sin(theta_opt); sin(theta_opt), cos(theta_opt)];
Q2_transformed = (R_opt * Q2_scaled')';

% Map transformed quadrilateral back to original reference frame
quad2_mapped = Q2_transformed + centroid1;

% ---- Display results ----
fprintf('Optimal rotation angle: %.4f rad (%.2f°)\n', theta_opt, rad2deg(theta_opt));
fprintf('RMSE after transformation: %.6f\n', sqrt(mean(vecnorm(Q2_transformed - Q1, 2, 2).^2)));

% ---- Plot the results ----
figure; hold on; axis equal;
scatter(Q1(:,1) + centroid1(1), Q1(:,2) + centroid1(2), 'ro', 'filled'); % Quad1 (reference)
scatter(quad2_mapped(:,1), quad2_mapped(:,2), 'gx'); % Transformed quad2
legend('Quad1 (reference)', 'Quad2 transformed');
title('Quadrilateral Alignment using Optimal Rotation');
grid on;