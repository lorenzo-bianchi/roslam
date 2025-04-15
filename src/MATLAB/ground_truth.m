%%
clear; close all; clc;

%% Parameters
test_case = 1;

anchors_poses_real_all = [      0,      0
                           1.7903,      0
                           1.7241, 3.6934
                          -0.1471, 3.7211
                           2.0991, 1.8572
                           0.7446, 3.1161
                          -0.3386, 2.1962
                           0.8853, 1.2402];

if test_case == 1
    % B1
    n_anchors = 4;
    n_robots = 3;
    n_test = 7;
    test_name = ['test', num2str(n_test)];       % B1:  test1 no / test2 yes / test3 no / test4 yes / test5 yes / test7 yes / test8 yes
    % video_path = ['./videos/', test_name, '.MP4'];
    video_path = ['/media/lorenzo/52387916-e258-4af3-95a4-c8701e29a684/@home/lorenzo/Desktop/DJI/', test_name, '.MP4'];
    min_times = [   0,  2,    0,    0,    0,    0,  7,    0];
    max_times = [1000, 86, 1000, 1000, 1000, 1000, 80, 1000];

    anchors_poses_real = [    0.0,    0.0;
                           1.7903,    0.0;
                           1.7241, 3.6934;
                          -0.1471, 3.7211];

    robots_last_poses = zeros(length(min_times), n_robots, 2);
    robots_last_poses(1, :, :) = [[ 0.1800, 0.2891]; [ 0.1119, 2.0519]; [1.1136, 1.8637]];
    robots_last_poses(2, :, :) = [[ 1.4006, 0.1307]; [-0.0750, 2.5112]; [1.2069, 2.2457]];
    robots_last_poses(3, :, :) = [[ 0.6113, 1.7061]; [ 1.1093, 2.3438]; [0.2111, 0.9648]];
    robots_last_poses(4, :, :) = [[ 0.4647, 1.7903]; [ 1.0888, 1.5127]; [0.2654, 1.0379]];
    robots_last_poses(5, :, :) = [[-0.6319, 1.3289]; [ 0.5811, 2.4525]; [1.3406, 2.6115]];
    robots_last_poses(6, :, :) = [[ 1.0710, 3.4887]; [ 0.2338, 1.1868]; [1.2776, 0.9613]];
    robots_last_poses(7, :, :) = [[-0.1876, 1.1250]; [ 0.9643, 1.8190]; [0.9052, 2.4240]];
    robots_last_poses(8, :, :) = [[ 0.9393, 2.7796]; [ 0.3895, 1.7389]; [1.6472, 1.3353]];

    % robot_colors = lines(7);
    % robot_colors = robot_colors(end-2:end, :);
    robot_colors = [0.5660, 0.8740, 0.2880;
                    0.3010, 0.7450, 0.9330;
                    1.0000, 0.6500, 0.0000];
    robot_colors_255 = round(robot_colors * 255);
else
    % B2
    n_anchors = 7;
    n_robots = 1;
    n_test = 6;         % 5 has problem
    test_name = ['B2_test', num2str(n_test)];       % B2:  B2_test1 / ... / B2_test11
    video_path = ['/media/lorenzo/52387916-e258-4af3-95a4-c8701e29a684/@home/lorenzo/Desktop/DJI/', test_name, '.MP4'];
                  %1   %2   %3   %4   %5   %6   %7   %8   %9  %10  %11
    min_times = [  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0];
    max_times = [119, 113, 111, 108, 123, 119, 111, 128,  95, 107, 118];

    anchors_poses_real= [      0,      0
                          1.7903,      0
                          1.7241, 3.6934
                         -0.1471, 3.7211
                          % 2.0991, 1.8572
                          0.7446, 3.1161
                         -0.3386, 2.1962
                          0.8853, 1.2402];

    robot_last_poses = [0.51, 0.34;
                        1.70, 0.75;
                        0.01, 0.85;
                        1.60, 1.39;
                        0.70, 2.16;
                        0.09, 2.68;
                        1.19, 1.84;
                        0.51, 0.34;
                        1.48, 1.22;
                        0.39, 0.95;
                        0.73, 2.23];


    % robot_colors = lines(7);
    % robot_colors = robot_colors(end-2:end, :);
    robot_colors = [0.5660, 0.8740, 0.2880];
    robot_colors_255 = round(robot_colors * 255);
end

min_time = min_times(n_test);
max_time = max_times(n_test);
pause_time = 0.001;
skip_n_frames = 10;
use_debug = false;
make_gif = false;
gif_filename = ['./gifs/', test_name, '.gif'];
frame_idx = 1;

area_threshold = 50;
anchors_rect_size = 300;
robots_rect_size = 200;
len_line = 100;
forward_skip = 500;

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
rects_name = ['./data/rects_', test_name, '.mat'];

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
    title(['Draw rectangles on FIXED ANCHORS in order from 1 to ', num2str(n_anchors)], 'FontSize', 15);
    hold on;
    for i = 1:n_anchors
        h = drawrectangle('LineWidth', 2);
        wait(h);  % Wait for user input
        pos = h.Position;  % Get rectangle position [x, y, width, height]

        % Ensure the rectangle does not exceed the image bounds
        pt1 = [max(1, pos(1)), max(1, pos(2))];
        pt2 = [min(video_width, pos(1) + pos(3)), min(video_height, pos(2) + pos(4))];

        anchors_rects(i, :) = [int32(pt1) int32(pt2)];
    end

    imshow(frame);
    title(['Draw rectangles on ROBOTS in order from 1 to ', num2str(n_robots)], 'FontSize', 15);
    if n_robots == 3
        disp('R1 has two UWBs, R2 has green parts, R3 has white parts')
    end
    hold on;
    for i = 1:n_robots
        h = drawrectangle('LineWidth', 2);
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
    cla;
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
    if make_gif
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
        frame = insertShape(frame, 'Rectangle', rects_xywh, 'LineWidth', 10, 'Color', robot_colors_255, 'Opacity', 1);
    
        angles = robots_poses_frame(:, 3);
        x1 = robots_poses_frame(:, 1) + len_line * cos(angles);
        y1 = robots_poses_frame(:, 2) + len_line * sin(angles);
        x2 = robots_poses_frame(:, 1) - len_line * cos(angles);
        y2 = robots_poses_frame(:, 2) - len_line * sin(angles);
    
        centers = [robots_poses_frame(:, 1), robots_poses_frame(:, 2), 15*ones(n_robots, 1)];
        frame = insertShape(frame, 'FilledCircle', centers, 'Color', robot_colors_255, 'Opacity', 1);
        frame = insertShape(frame, 'Line', [x2, y2, x1, y1], 'LineWidth', 15, 'Color', robot_colors_255);
    
        if mod(n_frame, skip_n_frames) ~= 0
            continue
        end
        imshow(frame); hold on;
    
        if n_frame == 0
            title('Press ENTER', 'FontSize', 15)
            pause()
        else
            title([num2str(n_frame)], 'FontSize', 15)
    
            % Plot trajectories
            if anchors_poses_frame(1, 1) > size(frame, 1) / 2
                K = [0 -1; -1 0];
            else
                K = [0 1; 1 0];
            end
            meters_to_pixels = @(p_pixels) (((p_pixels - c1) / s) * R' + c2) * K + anchors_poses_frame(1, :);
            for robot = 1:n_robots
                trajectories = meters_to_pixels(squeeze(robots_poses_world_history(:, robot, 1:2)));
                plot(trajectories(1:10:end, 1), trajectories(1:10:end, 2), 'Color', robot_colors(robot, :), 'LineStyle', '--');
                % set(gcf, 'Renderer', 'Painters');
                % pause(0.05)
                % set(gcf, 'Renderer', 'OpenGL');
            end

            pause(pause_time)

            set(gca, 'Visible', 'off');
            drawnow
            frame = getframe(gca);
            im = frame2im(frame);
            im = imresize(im, 0.5);
            [imind, cm] = rgb2ind(im, 256);
            if frame_idx == 1
                imwrite(imind, cm, gif_filename, 'gif', 'LoopCount', inf, 'DelayTime', 0.05);
            else
                imwrite(imind, cm, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
            end

            frame_idx = frame_idx + 1;
        end
    end
end

%% Save last frame and last trajectories
figure;
ax1 = axes('Position', [0, 0, 1, 1]); % Creazione di un axes che occupa tutta la figura
imshow(frame, 'Parent', ax1); % Mostra l'immagine nel primo axes

hold on;

if anchors_poses_frame(1, 1) > size(frame, 1) / 2
    K = [0 -1; -1 0];
else
    K = [0 1; 1 0];
end
meters_to_pixels = @(p_pixels) (((p_pixels - c1) / s) * R' + c2) * K + anchors_poses_frame(1, :);
last_trajectories = zeros(size(robots_poses_world_history, 1), n_robots, 2);
for robot = 1:n_robots
    trajectories = meters_to_pixels(squeeze(robots_poses_world_history(:, robot, 1:2)));
    plot(trajectories(1:10:end, 1), trajectories(1:10:end, 2), 'Color', robot_colors(robot, :), 'LineStyle', '--');
    last_trajectories(:, robot, :) = trajectories;
end

% set(gcf, 'Renderer', 'Painters');
% pause(0.5)
set(gcf, 'Renderer', 'OpenGL');

last_frame = frame;

%% Compute RMSE
if test_case == 1
    robots_last_pose = squeeze(robots_last_poses(n_test, :, :));
    rmse_real_pos = mean(vecnorm(robots_last_pose - robots_poses_world(:, 1:2), 2, 2));
else
    robots_last_pose = robot_last_poses(n_test, :);
    rmse_real_pos = mean(vecnorm(robots_last_pose - robots_poses_world(1:2), 2, 2));
end
fprintf("RMSE = %.3f m\n", rmse_real_pos);

%% Save variables
save(['./data/gt_', test_name, '.mat'], ...
      'anchors_min1', ...
      'anchors_min2', ...
      'anchors_max1', ...
      'anchors_max2', ...
      'anchors_poses_real_all', ...
      'anchors_rect_size', ...
      'last_frame', ...
      'last_trajectories', ...
      'n_anchors', ...
      'n_frame', ...
      'n_robots', ...
      'n_test', ...
      'rmse_real_pos', ...
      'rmse_history', ...
      'robot_colors', ...
      'robots_last_pose', ...
      'robots_poses_world_history', ...
      'test_case', ...
      'total_frames', ...
      'video_height', ...
      'video_path', ...
      'video_width' ...
    );

%% Plot poses
N = size(robots_poses_world_history, 1);

font_size = 13;
line_width = 1.5;

if test_case == 1
    figure('Position', [100, 100, 1500, 900]);
    n_cols = 5;
    tiledlayout(n_robots, n_cols, 'TileSpacing', 'compact', 'Padding', 'tight');

    color_x = 'r';
    color_y = 'g';
    color_theta = 'b';

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
        text(anchors_poses_real(i, 1), anchors_poses_real(i, 2) + 0.05, ...
             ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
             'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
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
else
    figure;
    n_cols = 4;
    n_rows = 3;
    tiledlayout(n_rows, n_cols, 'TileSpacing', 'compact', 'Padding', 'tight');

    % Extract data for the current robot
    x = squeeze(robots_poses_world_history(:, 1, 1));
    y = squeeze(robots_poses_world_history(:, 1, 2));
    theta = squeeze(robots_poses_world_history(:, 1, 3));

    % Unwrap theta to avoid jumps between π and -π
    theta = unwrap(theta);

    % Subplot 1: X coordinate
    nexttile(n_cols*0+1, [1, 2]);
    plot(1:N, x, 'r', 'LineWidth', line_width);
    % title('Robot 1 - X');
    xlabel('Samples');
    ylabel('x [m]');
    grid on;
    axis padded;
    xlim([0, inf]);
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

    % Subplot 2: Y coordinate
    nexttile(n_cols*1+1, [1,2]);
    plot(1:N, y, 'g', 'LineWidth', line_width);
    % title('Robot 1 - Y');
    xlabel('Samples');
    ylabel('y [m]');
    grid on;
    axis padded;
    xlim([0, inf]);
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

    % Subplot 3: Theta (unwrapped)
    nexttile(n_cols*2+1, [1,2]);
    plot(1:N, theta, 'b', 'LineWidth', line_width);
    % title('Robot 1 - Theta');
    xlabel('Samples');
    ylabel('\theta [rad]');
    grid on;
    axis padded;
    xlim([0, inf]);
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold');


    % Plot all robots together in a bigger tile
    nexttile(n_cols-1, [3, 2]);
    hold on;

    plot(x, y, 'Color', 'k', 'LineWidth', line_width, 'HandleVisibility', 'off');

    plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
    plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End

    % Plot anchors as black 'X' markers
    plot(anchors_poses_real_all(:, 1), anchors_poses_real_all(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

    % Add anchor labels
    for i = 1:size(anchors_poses_real_all, 1)
        text(anchors_poses_real_all(i, 1), anchors_poses_real_all(i, 2) + 0.12, ...
             ['A', num2str(i)], 'HorizontalAlignment', 'Center', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');
    end

    title('XY Trajectory');
    xlabel('x [m]');
    ylabel('y [m]');
    legend('Start', 'End', 'Location', 'northeast');
    xlim([min(anchors_poses_real_all(:,1))-0.2, max(anchors_poses_real_all(:,1))+0.2]);
    ylim([min(anchors_poses_real_all(:,2))-0.2, max(anchors_poses_real_all(:,2))+0.2]);
    axis equal;
    grid on;
    hold off;
    set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');
end

set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, ['./figures/', test_name, '.pdf'], 'ContentType', 'vector');
savefig(gcf, ['./figures/', test_name, '.fig']);
