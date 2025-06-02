function [scale_opt, R_opt, centroid1, centroid2, rmse] = fit_polygon(polygon_meters, polygon_pixels, use_anchor, use_debug)
    if all(use_anchor)
        pm = polygon_meters;
        pp = polygon_pixels;
    else
        polygon_meters(~use_anchor, :) = [0, 0];
        polygon_pixels(~use_anchor, :) = [0, 0];
        pm = polygon_meters;
        pp = polygon_pixels;
    end
    % Normalize and center
    centroid1 = mean(pm, 1);
    centroid2 = mean(pp, 1);
    poly1 = pm - centroid1;
    poly2 = pp - centroid2;
    
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