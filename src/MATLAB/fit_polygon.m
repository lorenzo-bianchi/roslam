function [scale_opt, R_opt, centroid1, centroid2, rmse] = fit_polygon(polygon_meters, polygon_pixels, use_debug)
    % Normalize and center
    centroid1 = mean(polygon_meters, 1);
    centroid2 = mean(polygon_pixels, 1);
    poly1 = polygon_meters - centroid1;
    poly2 = polygon_pixels - centroid2;
    
    % Compute optimal scaling factor
    scale_opt = sum(vecnorm(poly1, 2, 2)) / sum(vecnorm(poly2, 2, 2));
    poly2_scaled = poly2 * scale_opt;
    
    % Find optimal rotation angle using fminbnd
    rmse_func = @(theta) sqrt(mean(vecnorm(( [cos(theta), -sin(theta); sin(theta), cos(theta)] * poly2_scaled')' - poly1, 2, 2).^2));
    theta_opt = fminbnd(rmse_func, -pi, pi); % Search in [-π, π]
    
    % Compute final rotation matrix
    R_opt = [cos(theta_opt), -sin(theta_opt); sin(theta_opt), cos(theta_opt)];
    poly2_transformed = (R_opt * poly2_scaled')';
    
    % Map transformed quadrilateral back to original reference frame
    polygon_pixels_mapped = poly2_transformed + centroid1;

    rmse = sqrt(mean(vecnorm(poly2_transformed - poly1, 2, 2).^2));

    if use_debug
        % Display results
        fprintf('Optimal rotation angle: %.4f rad (%.2f°)\n', theta_opt, rad2deg(theta_opt));
        fprintf('RMSE after transformation: %.6f\n', rmse);
    
        % Plot the results
        figure; hold on; axis equal;
        scatter(poly1(:,1) + centroid1(1), poly1(:,2) + centroid1(2), 'ro', 'filled');
        scatter(polygon_pixels_mapped(:,1), polygon_pixels_mapped(:,2), 'gx');
        legend('polygon_meters (reference)', 'polygon_pixels transformed');
        title('Quadrilateral Alignment using Optimal Rotation');
        grid on;
    end
end