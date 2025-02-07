%%
clear; close all; clc;

%% Parameters
n_anchors = 4;
n_robots = 3;
n_test = 2;
video_name = ['test', num2str(n_test)];       % test1 no / test2 yes / test3 no / test4 yes / test5 yes / test7 yes / test8 yes
min_times = [   0,  2,    0,    0,    0,    0,  7,    0];
max_times = [1000, 86, 1000, 1000, 1000, 1000, 80, 1000];
min_time = min_times(n_test);
max_time = max_times(n_test);
pause_time = 0.05;
skip_n_frames = 50;
use_debug = false;

area_threshold = 50;
anchors_rect_size = 300;
robots_rect_size = 200;
len_line = 100;
forward_skip = 500;

anchors_poses_real= [    0.0,    0.0;
                      1.7903,    0.0; 
                      1.7241, 3.6934;
                     -0.1471, 3.7211];

% Red HSV
anchors_h_min1 = 0;
anchors_h_max1 = 10;
anchors_h_min2 = 170;
anchors_h_max2 = 180;
anchors_s_min = 100;
anchors_s_max = 255;
anchors_v_min = 100;
anchors_v_max = 255;

robots_h_min1 = 0;
robots_h_max1 = 10;
robots_h_min2 = 170;
robots_h_max2 = 180;
robots_s_min = 60;
robots_s_max = 255;
robots_v_min = 130;
robots_v_max = 255;

anchors_min1 = [anchors_h_min1, anchors_s_min, anchors_v_min] ./ [180, 255, 255];
anchors_max1 = [anchors_h_max1, anchors_s_max, anchors_v_max] ./ [180, 255, 255];
anchors_min2 = [anchors_h_min2, anchors_s_min, anchors_v_min] ./ [180, 255, 255];
anchors_max2 = [anchors_h_max2, anchors_s_max, anchors_v_max] ./ [180, 255, 255];

robots_min1 = [robots_h_min1, robots_s_min, robots_v_min] ./ [180, 255, 255];
robots_max1 = [robots_h_max1, robots_s_max, robots_v_max] ./ [180, 255, 255];
robots_min2 = [robots_h_min2, robots_s_min, robots_v_min] ./ [180, 255, 255];
robots_max2 = [robots_h_max2, robots_s_max, robots_v_max] ./ [180, 255, 255];

%% 
video_path = ['./videos/', video_name, '.MP4'];
video = VideoReader(video_path);
video.CurrentTime = min_time;
video_height = video.Height;
video_width = video.Width;
total_frames = floor(video.Duration * video.FrameRate);

%% 
anchor_localizer = AnchorLocalizer(n_anchors, area_threshold, anchors_min1, anchors_min2, anchors_max1, anchors_max2, anchors_rect_size, use_debug);
robot_localizer = RobotLocalizer(n_robots, forward_skip, area_threshold, robots_min1, robots_min2, robots_max1, robots_max2, robots_rect_size, use_debug);

%%
robots_poses_world_history = zeros(0, n_robots, 3);
rmse_history = [];

%% Manual inputs from user
rects_name = ['./data/', video_name, '.mat'];

if isfile(rects_name)
    load(rects_name)
    disp('Loaded rectangles from file')
else
    frame = readFrame(video);

    % Initialize empty arrays to store rectangle coordinates
    anchors_rects = zeros(n_anchors, 4);
    robots_rects = zeros(n_robots, 4);
    
    figure;
    imshow(frame);
    title(['Draw rectangles on FIXED ANCHORS in order from 1 to', num2str(n_anchors)], 'FontSize', 15);
    hold on;
    for i = 1:n_anchors
        h = drawrectangle('Color', 'g', 'LineWidth', 2);
        wait(h);  % Wait for user input
        pos = h.Position;  % Get rectangle position [x, y, width, height]
        
        % Ensure the rectangle does not exceed the image bounds
        pt1 = [max(1, pos(1)), max(1, pos(2))];
        pt2 = [min(video_width, pos(1) + pos(3)), min(video_height, pos(2) + pos(4))];
    
        anchors_rects(i, :) = [int32(pt1) int32(pt2)];
    end
    
    imshow(frame);
    title(['Draw rectangles on ROBOTS in order from 1 to', num2str(n_robots)], 'FontSize', 15);
    if n_robots == 3
        disp('R1 has two UWBs, R2 has green parts, R3 has white parts')
    end
    hold on;
    for i = 1:n_robots
        h = drawrectangle('Color', 'g', 'LineWidth', 2);
        wait(h);  % Wait for user input
        pos = h.Position;  % Get rectangle position [x, y, width, height]
        
        % Ensure the rectangle does not exceed the image bounds
        pt1 = [max(1, pos(1)), max(1, pos(2))];
        pt2 = [min(video_width, pos(1) + pos(3)), min(video_height, pos(2) + pos(4))];
    
        robots_rects(i, :) = [int32(pt1) int32(pt2)];
    end

    save(rects_name, 'anchors_rects', 'robots_rects');
    disp('Saved rectangles to file')
end

%% Main loop
anchors_poses_frame = zeros(n_anchors, 2);
robots_poses_frame = zeros(n_robots, 3);

n_frame = -1;
figure;
while hasFrame(video) && video.CurrentTime <= max_time
    if mod(n_frame, 100) == 0
        disp(n_frame)
    end
    frame = readFrame(video);
    n_frame = n_frame + 1;
    
    %% Localizations
    [anchors_poses_frame, anchors_rects] = anchor_localizer.localize(frame, anchors_rects);
    [robots_poses_frame, robots_rects] = robot_localizer.localize(frame, robots_rects);

    alpha = atan2(anchors_poses_frame(2, 2) - anchors_poses_frame(1, 2),...
                  anchors_poses_frame(2, 1) - anchors_poses_frame(1, 1));
    T = [1.0, 0.0, anchors_poses_frame(1, 1);...
         0.0, 1.0, anchors_poses_frame(1, 2);...
         0.0, 0.0,                       1.0];
    S = [1.0,  0.0, 0.0;...
         0.0, -1.0, 0.0;...
         0.0,  0.0, 1.0];
    R = [cos(alpha), -sin(alpha), 0.0;...
         sin(alpha),  cos(alpha), 0.0;...
                0.0,         0.0, 1.0];
    M = (T * R * S)^-1;
    anchors_poses_world = M * [anchors_poses_frame, ones(n_anchors, 1)]';
    anchors_poses_world = anchors_poses_world(1:2, :)';

    [s, R, c1, c2, rmse] = fit_polygon(anchors_poses_real, anchors_poses_world, false);

    robots_pos_world = M * [robots_poses_frame(:, 1:2), ones(n_robots, 1)]';
    robots_pos_world = robots_pos_world(1:2, :)';

    pixels_to_meters = @(p_pixel) ((p_pixel - c2) * s * R' + c1);
    robots_pos_world = pixels_to_meters(robots_pos_world);

    robots_angles_world = wrapToPi(-robots_poses_frame(:, 3) + alpha);
    if n_frame > 3
        last_angles = robots_poses_world_history(end, :, 3)';
        for robot = 1:n_robots
            if abs(last_angles(robot) - robots_angles_world(robot) ) > 0.5 && ...
               abs(robots_angles_world(robot)) < pi - 0.2
                last_last_angle = robots_poses_world_history(end-1, robot, 3)';
                robots_angles_world(robot) = 2 * last_angles(robot) - last_last_angle;
            end
        end
    end

    robots_poses_world = [robots_pos_world, robots_angles_world];

    % Save data in history
    robots_poses_world_history(end+1, :, :) = robots_poses_world;
    rmse_history(end+1) = rmse;

    %% Plots
    % Anchors
    rects_xywh = [anchors_rects(:, 1), anchors_rects(:, 2), ...
                  anchors_rects(:, 3) - anchors_rects(:, 1), ...
                  anchors_rects(:, 4) - anchors_rects(:, 2)];
    frame = insertShape(frame, 'Rectangle', rects_xywh, 'LineWidth', 10, 'Color', 'yellow', 'Opacity', 1);

    centers = [anchors_poses_frame(:, 1), anchors_poses_frame(:, 2), 15*ones(n_anchors, 1)];
    frame = insertShape(frame, 'FilledCircle', centers, 'Color', 'yellow', 'Opacity', 1);

    % Robots
    rects_xywh = [robots_rects(:, 1), robots_rects(:, 2), ...
                  robots_rects(:, 3) - robots_rects(:, 1), ...
                  robots_rects(:, 4) - robots_rects(:, 2)];
    frame = insertShape(frame, 'Rectangle', rects_xywh, 'LineWidth', 10, 'Color', 'green', 'Opacity', 1);

    angles = robots_poses_frame(:, 3);
    x1 = robots_poses_frame(:, 1) + len_line * cos(angles);
    y1 = robots_poses_frame(:, 2) + len_line * sin(angles);
    x2 = robots_poses_frame(:, 1) - len_line * cos(angles);
    y2 = robots_poses_frame(:, 2) - len_line * sin(angles);

    centers = [robots_poses_frame(:, 1), robots_poses_frame(:, 2), 15*ones(n_robots, 1)];
    frame = insertShape(frame, 'FilledCircle', centers, 'Color', 'green', 'Opacity', 1);
    frame = insertShape(frame, 'Line', [x2, y2, x1, y1], 'LineWidth', 15, 'Color', 'green');
    
    if mod(n_frame, skip_n_frames) ~= 0
        continue
    end
    imshow(frame);

    if n_frame == 0
        title('Press ENTER', 'FontSize', 15)
        pause()
    else
        title([num2str(n_frame)], 'FontSize', 15)
        pause(pause_time)
    end
end

%% Plot poses
N = size(robots_poses_world_history, 1);

figure;
n_cols = 5;
tiledlayout(n_robots, n_cols, 'TileSpacing', 'compact', 'Padding', 'tight');

robot_colors = lines(n_robots);  % MATLAB's colormap for distinct colors
color_x = 'r';
color_y = 'g';
color_theta = 'b';

font_size = 13;
line_width = 1.5;

for robot = 1:n_robots
    % Extract data for the current robot
    x = squeeze(robots_poses_world_history(:, robot, 1));
    y = squeeze(robots_poses_world_history(:, robot, 2));
    theta = squeeze(robots_poses_world_history(:, robot, 3));

    % Unwrap theta to avoid jumps between π and -π
    theta = unwrap(theta);

    % Subplot 1: X coordinate
    nexttile((robot-1)*n_cols+1);
    plot(1:N, x, color_x, 'LineWidth', line_width);
    title(['Robot ', num2str(robot), ' - X']);
    xlabel('Samples');
    ylabel('x [m]');
    grid on;
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

    % Subplot 2: Y coordinate
    nexttile((robot-1)*n_cols+2);
    plot(1:N, y, color_y, 'LineWidth', line_width);
    title(['Robot ', num2str(robot), ' - Y']);
    xlabel('Samples');
    ylabel('y [m]');
    grid on;
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

    % Subplot 3: Theta (unwrapped)
    nexttile((robot-1)*n_cols+3);
    plot(1:N, theta, color_theta, 'LineWidth', line_width);
    title(['Robot ', num2str(robot), ' - Theta']);
    xlabel('Samples');
    ylabel('\theta [rad]');
    grid on;
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

    % Subplot 4: XY Position
    nexttile((robot-1)*n_cols+4);
    plot(x, y, 'k', 'LineWidth', 2,'HandleVisibility', 'off'); % Trajectory
    hold on;
    plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
    plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
    legend('Start', 'End');

    % % Plot anchors as black 'X' markers
    % plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 10, 'LineWidth', 2,'HandleVisibility', 'off');
    % 
    % % Add anchor labels
    % for i = 1:size(anchors_poses_real, 1)
    %     text(anchors_poses_real(i, 1) - 0.14, anchors_poses_real(i, 2) + 0.25, ...
    %          ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
    % end
    % 
    % title(['Robot ', num2str(robot), ' - XY']);
    % xlabel('x [m]');
    % ylabel('y [m]');
    % xlim([min(anchors_poses_real(:,1))-1, max(anchors_poses_real(:,1))+1]);
    % ylim([min(anchors_poses_real(:,2))-1, max(anchors_poses_real(:,2))+1]);
    % axis equal;
    % grid on;
    % set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
end
set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');


% Plot all robots together in a bigger tile
nexttile(n_cols-1, [3, 2]);
hold on;
for robot = 1:n_robots
    x = squeeze(robots_poses_world_history(:, robot, 1));
    y = squeeze(robots_poses_world_history(:, robot, 2));

    plot(x, y, 'Color', robot_colors(robot, :), 'LineWidth', line_width);
end

for robot = 1:n_robots
    x = squeeze(robots_poses_world_history(:, robot, 1));
    y = squeeze(robots_poses_world_history(:, robot, 2));

    if robot == 1
        plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
        plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
    else
        plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'HandleVisibility', 'off'); % Start
        plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off'); % End
    end
end

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

% Add anchor labels
for i = 1:size(anchors_poses_real, 1)
    text(anchors_poses_real(i, 1) - 0.035, anchors_poses_real(i, 2) + 0.08, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
end

title('All Robots - XY Trajectories');
xlabel('x [m]');
ylabel('y [m]');
legend('Robot 1', 'Robot 2', 'Robot 3', 'Start', 'End', 'Location', 'north');
xlim([min(anchors_poses_real(:,1))-0.2, max(anchors_poses_real(:,1))+0.2]);
ylim([min(anchors_poses_real(:,2))-0.2, max(anchors_poses_real(:,2))+0.2]);
axis equal;
grid on;
hold off;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');

set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, '/home/lorenzo/Desktop/plot.pdf', 'ContentType', 'vector');