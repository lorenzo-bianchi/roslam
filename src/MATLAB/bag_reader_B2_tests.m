%% Cleanup
clc; close all
bag_field_B2

%% Plot style
line_width = 1;
marker_size = 20;

%% Params
wheel_radius = 0.033;
wheels_separation = 0.16;

tests_pre = [1, 4, 7, 9, 3, 6];
pos_anchors = [    0.0,    0.0;
                1.7903,    0.0;
                1.7241, 3.6934;
               -0.1471, 3.7211;
                2.0991, 1.8572;
                0.7446, 3.1161;
               -0.3386, 2.1962;
                0.8853, 1.2402];
n_anchors = size(pos_anchors, 1);

dkeys = ["44AE", "08B0", "8192", "5723", ...         % 2 and 4 are swapped
         "5107", "139C", "9128", "4A21"];
dvalues = ["A1", "A2", "A3", "A4", ...
           "A5", "A6", "A7", "A8"];

uwb_fixed_names = dictionary(dkeys(1:n_anchors), dvalues(1:n_anchors));

path_to_data = './data/';
files = dir([path_to_data, 'gt_B2_test*.mat']);
gt_data = struct();
for i = 1:length(tests_pre)
    file_name = [path_to_data, 'gt_B2_test', num2str(tests_pre(i)), '.mat'];
    temp = load(file_name);
    fn = fieldnames(temp);
    for j = 1:length(fn)
        gt_data(i).(fn{j}) = temp.(fn{j});
    end
end
clear temp
t_first_move_gt = [5.97, 11.11, 5.91, 5.84, 6.41, 6.11];

%% Read bag and topics names
test = 3;
bag_name = ['20250408_test', num2str(test)];

% bag_name = '20250314_test5';
% bag_name = '20250314_test6';
% bag_name = '20250320_test3';
% bag_name = '20250320_test4';
% bag_name = '20250320_test5';

t0 = 1;
tf = 120;
path_prefix = '/home/lorenzo/Github/University/turtlebot3-utv-fork/logs/';
full_path = strcat(path_prefix, bag_name, '/', bag_name, '_0.db3');
bag = ros2bagreader(full_path);
all_topics = string(bag.AvailableTopics.Properties.RowNames);
t_start = bag.StartTime;
t_end = bag.EndTime;

%% Get number of robots and topics names
n_robots = length(tests_pre);
topics_names = strings(0);
for i = 1:length(all_topics)
    topic = all_topics(i);

    id_str = regexp(topic, '/robot(\d+)/', 'tokens');
    if isempty(id_str)

    else
        topic_split = split(topic, '/');
         if length(topic_split) > 2
            topic_name = topic_split(3);

            if ~any(topics_names == topic_name)
                topics_names(end+1) = topic_name;
            end
         end
    end
end

%% Summary
fprintf('Duration: %.4f s\n', t_end-t_start);
fprintf('Found: %d robots, %d anchors\n', n_robots, n_anchors);
fprintf('Anchors in positions:\n');
for i = 1:length(pos_anchors)
    fprintf('\t- (%.4f, %.4f)\n', pos_anchors(i,1), pos_anchors(i,2));
end
fprintf('Available topics:\n');
for name = topics_names
    fprintf('\t- %s\n', name);
end

%% Topics analysis
% Ground truth
%gt_data = struct('initial_pose', [], 'final_distances_anchors', []);
for i = 1:length(tests_pre)
    starting_pose = char(starting_poses(tests_pre(i)));
    pnt0_char = starting_pose(1);
    angle0_char = starting_pose(3:end);
    pnt0 = pts(pnt0_char);
    angle0 = angles(angle0_char);
    gt_data(i).fps = 29.97;
    gt_data(i).initial_pose = [pnt0(1), pnt0(2), angle0];
    gt_data(i).final_distances_anchors = squeeze(final_distances_anchors(tests_pre(i), :))';
end

%% UWB
if any(contains(topics_names, 'uwb_tag'))
    uwb_topics_names = all_topics(contains(all_topics, 'uwb_tag'));
    uwb_anchors_data = cell(length(uwb_topics_names), 1);
    for i = 1:length(uwb_topics_names)
        % robot i
        uwb_topic = select(bag, 'Topic', uwb_topics_names(i));
        num_msgs = uwb_topic.NumMessages;
        uwb_msg = readMessages(uwb_topic);
        t_start_uwb = double(uwb_msg{1}.header.stamp.sec) + double(uwb_msg{1}.header.stamp.nanosec)/1e9;
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, uwb_msg);
        times = times - t_start_uwb;

        for j = 1:num_msgs
            % message at time j
            uwb_dist = nan*ones(1, n_anchors);
            for k = 1:size(uwb_msg{j}.uwbs, 1)
                id_str = uwb_msg{j}.uwbs(k).id_str;
                anchor_num = str2double(extractBetween(uwb_fixed_names(id_str), 2, 2));
                single_dist = uwb_msg{j}.uwbs(k).dist;
                if single_dist < 0.1
                    single_dist = nan;
                end
                uwb_dist(anchor_num) = single_dist;
            end
            uwb_anchors_data{i}(end+1, :) = [times(j), uwb_dist];
        end
    end
end
clear uwb_msg

colors = lines(8);

% Plot anchors distances
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(uwb_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');

for i = 1:length(uwb_topics_names)
    data = uwb_anchors_data{i};

    times = data(:, 1);
    distances = data(:, 2:end);
    timesA = [];
    timesB = [];
    distancesA = [];
    distancesB = [];

    for j = 1:size(distances, 1)
        if all(isnan(distances(j, 5:8)))
            timesA = [timesA, times(j)];
            distancesA = [distancesA; distances(j, 1:4)];
        else
            timesB = [timesB, times(j)];
            distancesB = [distancesB; distances(j, 5:8)];
        end
    end

    t = tiledlayout(T, 1, 2, 'TileSpacing', 'Compact', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t,1);
    for k = 1:size(distancesA, 2)
        plot(timesA, distancesA(:,k), 'LineWidth', line_width, 'Color', colors(k, :));
        hold on;
    end
    grid on;
    hold on;
    for j = 1:size(distancesA, 2)
        nan_indices = isnan(distancesA(:, j));
        valid_indices = ~nan_indices;
        distancesA(:, j) = interp1(timesA(valid_indices), distancesA(valid_indices, j), timesA, 'linear', 'extrap');
        plot(timesA(nan_indices), distancesA(nan_indices, j), 'rx', 'MarkerSize', marker_size, 'LineWidth', line_width);
    end
    xlim([t0 inf]);
    ylim([0, 5]);
    xlabel('time [s]');
    ylabel('distance [m]');
    legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Location', 'northwest', 'NumColumns', 4);
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t,2);
    for k = 1:size(distancesB, 2)
        plot(timesB, distancesB(:,k), 'LineWidth', line_width, 'Color', colors(k+4,:));
        hold on;
    end
    grid on;
    hold on;
    for j = 1:size(distancesB, 2)
        nan_indices = isnan(distancesB(:, j));
        valid_indices = ~nan_indices;
        distancesB(:, j) = interp1(timesB(valid_indices), distancesB(valid_indices, j), timesB, 'linear', 'extrap');
        plot(timesB(nan_indices), distancesB(nan_indices, j), 'rx', 'MarkerSize', marker_size, 'LineWidth', line_width);
    end
    xlim([t0 inf]);
    ylim([0, 3.5]);
    xlabel('time [s]');
    ylabel('distance [m]');
    legend('Anchor 5', 'Anchor 6', 'Anchor 7', 'Anchor 8', 'Location', 'northwest', 'NumColumns', 4);
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Joint states
if any(contains(topics_names, 'joint_states'))
    js_topics_names = all_topics(contains(all_topics, 'joint_states'));
    js_data = cell(length(js_topics_names), 1);
    for i = 1:length(js_topics_names)
        % robot i
        js_topic = select(bag, 'Topic', js_topics_names(i));
        js_msg = readMessages(js_topic);

        if contains(js_msg{1}.name{1}, 'right')
            idx_right = 1;
            idx_left = 2;
        else
            idx_right = 2;
            idx_left = 1;
        end

        t_start_js = double(js_msg{1}.header.stamp.sec) + double(js_msg{1}.header.stamp.nanosec)/1e9;
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, js_msg);
        times = times - t_start_js;

        vel_left = cellfun(@(m) m.velocity(idx_left), js_msg);

        vel_right = cellfun(@(m) m.velocity(idx_right), js_msg);

        js_data{i} = [times, vel_right, vel_left];
    end
end

% Plot wheels velocities
figure;
tiledlayout(length(js_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1000, 1600]);

t_first_move_bag_tv = zeros(1, n_robots);

for i = 1:length(js_topics_names)
    data = js_data{i};

    idx = find(data(200:-1:1, 2) == 0 & data(200:-1:1, 3) == 0, 1, 'first');
    t_first_move_bag_tv(i) = data(200-idx+2, 1);

    times = data(:, 1);
    velocities = data(:, 2:end);

    nexttile;
    plot(times, velocities, 'LineWidth', line_width);
    grid on;
    hold on;

    title(['Robot ', num2str(i)]);
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    legend('Right wheel', 'Left wheel', 'Location', 'southeast');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
    xlim([t0 tf]);
    ytickformat('%.2f');
end

%% Odom
if any(contains(topics_names, 'odom'))
    odom_topics_names = all_topics(contains(all_topics, 'odom'));
    odom_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    odom_data_cell = cell(length(odom_topics_names), 1);
    for i = 1:length(odom_topics_names)
        odom_topic = select(bag, 'Topic', odom_topics_names(i));
        odom_msg = readMessages(odom_topic);

        t_start_odom = double(odom_msg{1}.header.stamp.sec) + double(odom_msg{1}.header.stamp.nanosec)/1e9;
        odom_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, odom_msg);
        odom_data(i).times = odom_data(i).times - t_start_odom;
        odom_data(i).x = cellfun(@(m) m.pose.pose.position.x, odom_msg);
        odom_data(i).y = cellfun(@(m) m.pose.pose.position.y, odom_msg);
        odom_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, odom_msg);
        odom_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, odom_msg);
        for k = 1:length(odom_data(i).v_ang)
            if abs(odom_data(i).v_ang(k)) > 10
                odom_data(i).v_ang(k) = odom_data(i).v_ang(k-1);
            end
        end

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, odom_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, odom_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, odom_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, odom_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        odom_data(i).theta = rpy(:, 1);

        odom_data_cell{i} = [odom_data(i).times(:), odom_data(i).x(:), odom_data(i).y(:), odom_data(i).theta(:), odom_data(i).v_lin(:), odom_data(i).v_ang(:)];
    end
end

% % Plot odometry
% figure;
% set(gcf, 'Position', [100, 100, 1200, 800]);
% 
% for i = 1:length(odom_topics_names)
%     data = odom_data_cell{i};
% 
%     positions = data(:, 2:3);
% 
%     nexttile;
%     plot(positions(:, 1), positions(:, 2), 'LineWidth', line_width);
%     grid on;
%     hold on;
% 
%     title(['Robot ', num2str(i)]);
%     xlabel('x [m]');
%     ylabel('y [m]');
%     set(gca, 'FontSize', 12, 'FontWeight', 'bold');
%     axis equal
% end

% Plot robot velocities
figure;
tiledlayout(length(odom_topics_names), 2, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1200, 800]);

for i = 1:length(odom_topics_names)
    data = odom_data_cell{i};

    times = data(:, 1);
    velocities = data(:, end-1:end);

    nexttile;
    plot(times, velocities(:, 1), 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 inf]);

    title(['Robot ', num2str(i), ' - Linear velocity']);
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');

    nexttile;
    plot(times, velocities(:, 2), 'Color', '#D95319', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 inf]);

    title(['Robot ', num2str(i), ' - Angular velocity']);
    xlabel('time [s]');
    ylabel('velocity [rad/s]');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Ground truth
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(n_robots, 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

for i = 1:n_robots
    data = squeeze(gt_data(i).robots_poses_world_history);

    times = linspace(0, gt_data(i).n_frame / gt_data(i).fps, size(gt_data(i).robots_poses_world_history, 1));
    x = data(:, 1);
    y = data(:, 2);
    theta = unwrap(data(:, 3));

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d GT', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times, x, 'Color', 'r', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times, y, 'Color', 'g', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times, theta, 'Color', 'b', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');
end

figure
tiledlayout(2, 3, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1100, 1200]);
for i = 1:n_robots
    data_gt = squeeze(gt_data(i).robots_poses_world_history);
    xy = data_gt(:, 1:2);
    theta = unwrap(data_gt(:, 3));

    data_odom = odom_data_cell{i};
    positions = rototranslation(data_odom(:, 2:3), xy(1, 1:2), mean(theta(1:80)));

    nexttile;
    plot(xy(:, 1), xy(:, 2), 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(positions(:, 1), positions(:, 2), 'b', 'LineWidth', line_width);
    xlabel('x [m]');
    ylabel('y [m]');
    axis equal
    title(['Robot ', num2str(i)]);
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');

    legend('Ground truth', 'Odometry')

    xlim([-0.5 2])
    ylim([-0.1 3.9])
end

%% Estimated poses
if any(contains(topics_names, 'estimated_pose'))
    est_pose_topics_names = all_topics(contains(all_topics, 'estimated_pose'));
    est_pose_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'var_x', [], 'var_y', [], 'var_theta', []);
    est_pose_data_cell = cell(length(est_pose_topics_names), 1);
    t_first_move_bag_ros = zeros(1, n_robots);
    for i = 1:length(est_pose_topics_names)
        est_pose_topic = select(bag, 'Topic', est_pose_topics_names(i));
        est_pose_msg = readMessages(est_pose_topic);

        t_start_est_pose = double(est_pose_msg{1}.header.stamp.sec) + double(est_pose_msg{1}.header.stamp.nanosec)/1e9;
        est_pose_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, est_pose_msg);
        est_pose_data(i).times = est_pose_data(i).times - t_start_est_pose - (uwb_topic.MessageList.Time(1) - t_start);
        est_pose_data(i).x = cellfun(@(m) m.pose.pose.position.x, est_pose_msg);
        est_pose_data(i).y = cellfun(@(m) m.pose.pose.position.y, est_pose_msg);
        est_pose_data(i).var_x = cellfun(@(m) m.pose.covariance(1), est_pose_msg);
        est_pose_data(i).var_y = cellfun(@(m) m.pose.covariance(8), est_pose_msg);
        est_pose_data(i).var_theta = cellfun(@(m) m.pose.covariance(36), est_pose_msg);

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, est_pose_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, est_pose_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, est_pose_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, est_pose_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        est_pose_data(i).theta = rpy(:, 1);

        est_pose_data_cell{i} = [est_pose_data(i).times(:), est_pose_data(i).x(:), est_pose_data(i).y(:), est_pose_data(i).theta(:), est_pose_data(i).var_x(:), est_pose_data(i).var_y(:), est_pose_data(i).var_theta(:)];

        idx_array = find(est_pose_data(i).x == 0 & est_pose_data(i).x == 0);
        if idx_array(end) - idx_array(end-1) == 1
            idx = idx_array(end);
        else
            idx = idx_array(end-1);
        end
        t_first_move_bag_ros(i) = est_pose_data(i).times(idx+1);
    end
end
clear est_pose_msg

resets_wrt_est = zeros(n_robots, 1);
t_resets_wrt_est = zeros(n_robots, 1);
% Check if some robots reset
for robot = 1:n_robots
    xy = est_pose_data_cell{robot}(:, 2:3);
    times = est_pose_data_cell{robot}(:, 1);
    for i = 200:size(xy, 1)
        xy_prec = xy(i-1, :);
        xy_curr = xy(i, :);
        dist = norm(xy_curr - xy_prec);
        if dist > 0.41
            resets_wrt_est(robot) = i;
            t_resets_wrt_est(robot) = times(i);
        end
    end
end

% Plot robots poses
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

for i = 1:length(est_pose_topics_names)
    data = est_pose_data_cell{i};

    times = data(:, 1);
    x = data(:, 2);
    y = data(:, 3);

    if resets_wrt_est(i) > 0
        theta = [unwrap(data(1:resets_wrt_est(i)-1, 4)); unwrap(data(resets_wrt_est(i):end, 4))];
    else
        theta = unwrap(data(:, 4));
    end

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times, x, 'Color', 'r', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 tf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times, y, 'Color', 'g', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 tf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times, theta, 'Color', 'b', 'LineWidth', line_width);
    grid on;
    hold on;
    xlim([t0 tf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Estimated landmarks
if any(contains(topics_names, 'estimated_landmarks'))
    est_lnds_topics_names = all_topics(contains(all_topics, 'estimated_landmarks'));
    est_lnds_data = cell(length(est_lnds_topics_names), 1);
    
    for i = 1:length(est_lnds_topics_names)
        est_lnds_topic = select(bag, 'Topic', est_lnds_topics_names(i));
        est_lnds_msg = readMessages(est_lnds_topic);
    
        t_start_est_pose = double(est_lnds_msg{1}.header.stamp.sec) + double(est_lnds_msg{1}.header.stamp.nanosec)/1e9;
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, est_lnds_msg);
        times = times - t_start_est_pose - (uwb_topic.MessageList.Time(1) - t_start);
    
        x = cellfun(@(m) reshape([m.landmarks.x], [], 1), est_lnds_msg, 'UniformOutput', false);
        y = cellfun(@(m) reshape([m.landmarks.y], [], 1), est_lnds_msg, 'UniformOutput', false);
        var_x = cellfun(@(m) reshape([m.landmarks.var_x], [], 1), est_lnds_msg, 'UniformOutput', false);
        var_y = cellfun(@(m) reshape([m.landmarks.var_y], [], 1), est_lnds_msg, 'UniformOutput', false);
        cov_xy = cellfun(@(m) reshape([m.landmarks.cov_xy], [], 1), est_lnds_msg, 'UniformOutput', false);
    
        max_landmarks = max(cellfun(@numel, x));
    
        pad_fun = @(v) padarray(v(:)', [0, max_landmarks - numel(v)], NaN, 'post');
    
        x = cell2mat(cellfun(pad_fun, x, 'UniformOutput', false));
        y = cell2mat(cellfun(pad_fun, y, 'UniformOutput', false));
        var_x = cell2mat(cellfun(pad_fun, var_x, 'UniformOutput', false));
        var_y = cell2mat(cellfun(pad_fun, var_y, 'UniformOutput', false));
        cov_xy = cell2mat(cellfun(pad_fun, cov_xy, 'UniformOutput', false));
    
        est_lnds_data{i} = struct('times', times, 'x', x, 'y', y, 'var_x', var_x, 'var_y', var_y, 'cov_xy', cov_xy);
    end
end
clear est_lnds_msg

%%
initial_poses_gt = zeros(length(tests_pre), 3);
ending_poses_gt = zeros(length(tests_pre), 3);
initial_poses_gt_reset = zeros(length(tests_pre), 3);
for i = 1:length(tests_pre)
    initial_poses_gt(i, :) = squeeze(gt_data(i).robots_poses_world_history(1, 1, :));
    ending_poses_gt(i, :) = squeeze(gt_data(i).robots_poses_world_history(end, 1, :));
end

% figure;
% hold on;
% grid on;
% pts_keys = keys(pts);
% for idx = 1:num_anchors
%     key = pts_keys{idx};
%     xy = pts(key);
% 
%     num = key(2);
%     plot(xy(1), xy(2), 'rh', 'LineWidth', line_width, 'MarkerSize', marker_size);
%     text(xy(1), xy(2)+0.1, num2str(num), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
% end
% 
% for idx = 1:length(tests_pre)
%     x = initial_poses_gt(idx, 1);
%     y = initial_poses_gt(idx, 2);
%     theta = initial_poses_gt(idx, 3);
%     L = 0.2;
%     dx = L * cos(theta);
%     dy = L * sin(theta);
%     quiver(x, y, dx, dy, 'LineWidth', line_width, 'MaxHeadSize', 1);
% end
% 
% colors = lines(length(tests_pre));
% for i = 1:length(tests_pre)
%     gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
%     plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', colors(i, 1:end))
% end
% 
% theta = -0.85;
% for i = 1:length(tests_pre)
%     data = est_pose_data_cell{i};
% 
%     xy = data(:, 2:3) - data(end, 2:3);
%     xy_w = rototranslation(xy, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
%     xy_w = rototranslation(xy, ending_poses_gt(i, 1:2), ending_poses_gt(i, 3)-theta);
% 
%     plot(xy_w(:, 1), xy_w(:, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end))
% end
% 
% for i = 1:length(tests_pre)
%     data = est_lnds_data{i};
% 
%     xy = [data.x(end, :)', data.y(end, :)'];
%     xy_w = rototranslation(xy, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
%     xy_w = rototranslation(xy, ending_poses_gt(i, 1:2), ending_poses_gt(i, 3)-theta);
% 
%     plot(xy_w(:, 1), xy_w(:, 2), 'kh', 'LineWidth', line_width, 'MarkerSize', marker_size);
% end
% 
% xlabel('x [m]')
% ylabel('y [m]')
% axis equal;
% xlim([min(z_opt(:,1))-1, max(z_opt(:,1))+1]);
% ylim([min(z_opt(:,2))-1, max(z_opt(:,2))+1]);
% set(gca, 'FontSize', 12, 'FontWeight', 'bold');

%% Only estimated pose
% for i = 1:length(tests_pre)
%     pose_robot = est_pose_data_cell{i}(:, 2:4);
% 
%     figure
%     plot(pose_robot(:, 1), pose_robot(:, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end))
%     hold on
%     grid on
%     axis equal
% 
%     for j = 1:10:length(pose_robot)
%         x = pose_robot(j, 1);
%         y = pose_robot(j, 2);
%         theta = pose_robot(j, 3);
%         L = 0.1;
%         dx = L * cos(theta);
%         dy = L * sin(theta);
%         quiver(x, y, dx, dy, 'LineWidth', line_width, 'MaxHeadSize', 1, 'Color', 'r');
%     end
% 
%     xlim([-1 2.7])
%     ylim([-0.5 4.3])
% end

%%
% for i = 1:length(tests_pre)
%     xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];
%     [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 100, 0.15, 7);
%     xy_lnds_w = (R * xy_lnds' + t)';
% 
%     figure
%     plot(pos_anchors(:, 1), pos_anchors(:, 2), 'rh', 'LineWidth', line_width, 'MarkerSize', marker_size);
%     hold on
%     for idx = 1:num_anchors
%         text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
%     end
%     plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'kh', 'LineWidth', line_width, 'MarkerSize', marker_size);
%     for idx = 1:num_anchors
%         text(xy_lnds_w(idx, 1), xy_lnds_w(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'black');
%     end
% 
%     if length(tests_pre) == 1
%         plot(ending_poses_gt(1), ending_poses_gt(2), 'ro', 'LineWidth', line_width, 'MarkerSize', marker_size);
%     else
%         plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ro', 'LineWidth', line_width, 'MarkerSize', marker_size);
%     end
% 
%     xy_robot = est_pose_data_cell{i}(end, 2:3);
%     xy_robot_w = (R * xy_robot' + t)';
%     plot(xy_robot_w(1), xy_robot_w(2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size);
% 
%     gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
%     plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', colors(i, 1:end))
% 
%     data = est_pose_data_cell{i};
%     xy = data(:, 2:3);
%     xy_w = (R * xy' + t)';
%     plot(xy_w(:, 1), xy_w(:, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end))
% 
%     grid on
%     axis equal
% end

%% Ground truth + Estimated poses (WRT anchors)
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

gt_errors_pos = cell(n_robots, 1);
gt_errors_angle = cell(n_robots, 1);
times = cell(n_robots, 1);
idx_convergence = zeros(n_robots, 1);

deltas = t_first_move_gt - t_first_move_bag_ros;

for i = 1:n_robots
    data_gt = squeeze(gt_data(i).robots_poses_world_history);

    new_points = 1000;
    points_end = repmat(data_gt(end, :), new_points, 1);
    data_gt = [data_gt; points_end];
    times_gt = linspace(0, (gt_data(i).n_frame + new_points) / gt_data(i).fps, size(data_gt, 1));

    x_gt = data_gt(:, 1);
    y_gt = data_gt(:, 2);
    theta_gt = unwrap(data_gt(:, 3));
    
    data_est = est_pose_data_cell{i};
    times_est = data_est(:, 1);

    t_start_gt_x = deltas(i);
    t_start_gt_y = deltas(i);
    t_start_gt_theta = deltas(i);

    idx_gt_start_x = find(times_gt > t_start_gt_x, 1);
    idx_gt_start_y = find(times_gt > t_start_gt_y, 1);
    idx_gt_start_theta = find(times_gt > t_start_gt_theta, 1);

    x_gt = x_gt(idx_gt_start_x:end);
    y_gt = y_gt(idx_gt_start_y:end);
    theta_gt = theta_gt(idx_gt_start_theta:end);

    times_gt_x = times_gt(idx_gt_start_x:end) - t_start_gt_x;
    x_gt = interp1(times_gt_x, x_gt, times_est, 'linear');

    times_gt_y = times_gt(idx_gt_start_y:end) - t_start_gt_y;
    y_gt = interp1(times_gt_y, y_gt, times_est, 'linear');

    times_gt_theta = times_gt(idx_gt_start_theta:end) - t_start_gt_theta;
    theta_gt = interp1(times_gt_theta, theta_gt, times_est, 'linear');

    % remove nans
    times_gt = times_est;
    x_gt(isnan(x_gt)) = x_gt(find(~isnan(x_gt), 1, 'first'));
    y_gt(isnan(y_gt)) = y_gt(find(~isnan(y_gt), 1, 'first'));
    theta_gt(isnan(theta_gt)) = theta_gt(find(~isnan(theta_gt), 1, 'first'));

    if resets_wrt_est(i) > 0
        initial_poses_gt_reset(i, :) = [x_gt(resets_wrt_est(i)), y_gt(resets_wrt_est(i)), theta_gt(resets_wrt_est(i))];
    end

    len_bag = size(est_pose_data_cell{i}, 1);
    xy_lnds_w = zeros(len_bag, 8, 2);
    xy_robot_w = zeros(len_bag, 2);
    theta_robot_w = zeros(len_bag, 1);
    first_time = true;
    for k = 1:len_bag
        xy_lnds = [est_lnds_data{i}.x(k, :)', est_lnds_data{i}.y(k, :)'];
        xy_robot = est_pose_data_cell{i}(k, 2:3);
        theta_robot = est_pose_data_cell{i}(k, 4);
        % if k < len_bag / 3
        %     n_inl = 5;
        % elseif k < 2 * len_bag / 3
        %     n_inl = 6;
        % else
        %     n_inl = 7;
        % end
        [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.4-0.2*k/len_bag, 7);
        if resets_wrt_est(i) > 0
            if k < resets_wrt_est(i)
                xy_0 = initial_poses_gt(i, 1:2);
                theta_0 = initial_poses_gt(i, 3);
            else
                xy_0 = [x_gt(resets_wrt_est(i)), y_gt(resets_wrt_est(i))];
                theta_0 = theta_gt(resets_wrt_est(i));
            end
        else
            xy_0 = initial_poses_gt(i, 1:2);
            theta_0 = initial_poses_gt(i, 3);
        end
        if isempty(inl)
            first_time = true;
            xy_robot_w(k, :) = rototranslation(xy_robot, xy_0, theta_0);
            theta_robot_w(k, 1) = theta_robot + theta_0;
        else
            if first_time
                first_time = false;
                idx_convergence(i) = k;
            end
            xy_robot_w(k, :) = (R * xy_robot' + t)';
            theta_robot_w(k, 1) = theta_robot + atan2(R(2,1), R(1,1));
        end
    end

    x_est = xy_robot_w(:, 1);
    y_est = xy_robot_w(:, 2);

    if resets_wrt_est(i) > 0
        theta_est = [unwrap(theta_robot_w(1:resets_wrt_est(i)-1)); unwrap(theta_robot_w(resets_wrt_est(i):end))];
    else
        theta_est = unwrap(theta_robot_w(:, 1));
    end

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times_gt, x_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, x_est, 'r', 'LineWidth', line_width);
    if idx_convergence(i) > 0
        xline(times_est(idx_convergence(i)), 'k', 'LineWidth', line_width);
    end
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times_gt, y_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, y_est, 'g', 'LineWidth', line_width);
    if idx_convergence(i) > 0
        xline(times_est(idx_convergence(i)), 'k', 'LineWidth', line_width);
    end
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times_gt, theta_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, theta_est, 'b', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    if idx_convergence(i) > 0
        xline(times_est(idx_convergence(i)), 'k', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');

    gt_errors_pos{i} = sqrt((x_gt - x_est).^2 + (y_gt- y_est).^2);
    gt_errors_angle{i} = abs(theta_gt - theta_est);
    times{i} = times_gt;

    diff_xy = diff(gt_errors_pos{i});
    idx_stop = find(abs(diff_xy) < 1e-4);
    if isempty(idx_stop)
        idx_stop = length(x_gt);
    else
        last_one = find(diff(idx_stop) ~= 1, 1, 'last') + 1;
        if isempty(last_one)
            last_one = 1;
        end
        idx_stop = idx_stop(last_one);
    end

    gt_errors_pos{i} = gt_errors_pos{i}(1:idx_stop);
    gt_errors_angle{i} = gt_errors_angle{i}(1:idx_stop);
    times{i} = times{i}(1:idx_stop);
end

%% Ground truth + Estimated poses errors (WRT anchors)
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

tf = 120;
for i = 1:n_robots
    t = tiledlayout(T, 1, 2, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    err_pos = gt_errors_pos{i};
    plot(times{i}, err_pos, 'r', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, repmat(mean(err_pos), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
    if idx_convergence(i) > 0
        plot(times_est, repmat(mean(err_pos(idx_convergence(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xline(times_est(idx_convergence(i)), 'k', 'LineWidth', line_width);
    end
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('e_{xy} [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    if resets_wrt_est(i) > 0
        ax2 = axes('Position', [0.261, 0.0735, 0.185, 0.072]);
        plot(times{i}, err_pos, 'r', 'LineWidth', line_width);
        hold on
        plot(times_est, repmat(mean(err_pos), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
        plot(times_est, repmat(mean(err_pos(idx_convergence(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xlim([times{i}(idx_convergence(i)) tf]);
        ylim([0, max(err_pos(idx_convergence(i):end-10))])
        set(ax2, 'Color', 'none');
        set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
    end

    ax2 = nexttile(t, 2);
    err_angle = gt_errors_angle{i};
    plot(times{i}, err_angle, 'b', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, repmat(mean(err_angle), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
    if idx_convergence(i) > 0
        plot(times_est, repmat(mean(err_angle(idx_convergence(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xline(times_est(idx_convergence(i)), 'k', 'LineWidth', line_width);
    end
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('e_\theta [rad]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    if resets_wrt_est(i) > 0
        ax2 = axes('Position', [0.7515, 0.0735, 0.185, 0.072]);
        plot(times{i}, err_angle, 'b', 'LineWidth', line_width);
        hold on
        plot(times_est, repmat(mean(err_angle), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
        plot(times_est, repmat(mean(err_angle(idx_convergence(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xlim([times{i}(idx_convergence(i)) tf]);
        ylim([0, max(err_angle(idx_convergence(i):end-10))])
        set(ax2, 'Color', 'none');
        set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
    end
end

%% WRT anchors
ending_poses_est = zeros(n_robots, 2);
xy_lnds_w_cell = cell(n_robots, 1);
for i = 1:length(tests_pre)
    len_bag = length(est_lnds_data{i}.times);
    xy_lnds_w = zeros(len_bag, 8, 2);
    xy_robot_w = zeros(len_bag, 2);
    for k = 1:len_bag
        xy_lnds = [est_lnds_data{i}.x(k, :)', est_lnds_data{i}.y(k, :)'];
        xy_robot = est_pose_data_cell{i}(k, 2:3);
        theta_robot = est_pose_data_cell{i}(k, 4);
        [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.2, 7);
        if resets_wrt_est(i) > 0
            if k < resets_wrt_est(i)
                xy_0 = initial_poses_gt(i, 1:2);
                theta_0 = initial_poses_gt(i, 3);
            else
                xy_0 = [x_gt(resets_wrt_est(i)), y_gt(resets_wrt_est(i))];
                theta_0 = theta_gt(resets_wrt_est(i));
            end
        else
            xy_0 = initial_poses_gt(i, 1:2);
            theta_0 = initial_poses_gt(i, 3);
        end
        if isempty(inl)
            xy_robot_w(k, :) = rototranslation(xy_robot, xy_0, theta_0);
            xy_lnds_w(k, :, :) = rototranslation(xy_lnds, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
            theta_robot_w(k, 1) = theta_robot + theta_0;
        else
            xy_robot_w(k, :) = (R * xy_robot' + t)';
            xy_lnds_w(k, :, :) = (R * xy_lnds' + t)';
            theta_robot_w(k, 1) = theta_robot + atan2(R(2,1), R(1,1));
        end
    end
    ending_poses_est(i, :) = xy_robot_w(end, :);
    xy_lnds_w_cell{i} = xy_lnds_w;

    figure
    set(gcf, 'Position', [100, 100, 1000, 1600]);

    h_gt_anchors = plot(pos_anchors(:, 1), pos_anchors(:, 2), 'kh', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT anchors');
    hold on
    grid on
    axis equal
    gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
    h_gt_poses = plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', 'k', 'DisplayName', 'GT trajectory');

    for idx = 1:num_anchors
        text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    end
    h_est_anchors = plot(xy_lnds_w(end, :, 1), xy_lnds_w(end, :, 2), 'h', 'LineWidth', line_width, 'MarkerSize', marker_size, 'Color', colors(i, 1:end), 'DisplayName', 'Est. anchors');
    for idx = 1:num_anchors
        text(xy_lnds_w(end, idx, 1), xy_lnds_w(end, idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', colors(i, 1:end));
    end

    if isscalar(tests_pre)
        h_gt_end_point = plot(ending_poses_gt(1), ending_poses_gt(2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT end');
        h_gt_start_point = plot(initial_poses_gt(1), initial_poses_gt(2), 'kx', 'LineWidth', line_width, 'MarkerSize', 18, 'DisplayName', 'GT start');
    else
        h_gt_end_point = plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT end');
        h_gt_start_point = plot(initial_poses_gt(i, 1), initial_poses_gt(i, 2), 'kx', 'LineWidth', line_width, 'MarkerSize', 18, 'DisplayName', 'GT start');
    end

    h_est_end_point = plot(xy_robot_w(end, 1), xy_robot_w(end, 2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size, 'Color', colors(i, 1:end), 'DisplayName', 'Est. end');
    % h_est_start_point = plot(xy_robot_w(1, 1), xy_robot_w(1, 2), 'kx', 'LineWidth', line_width, 'MarkerSize', marker_size, 'Color', colors(i, 1:end), 'DisplayName', 'Est. start');

    xlim([-1 2.7])
    ylim([-0.4 4.2])

    xlabel("x [m]")
    ylabel("y [m]")
    
    h_after_conv = plot(xy_robot_w(idx_convergence(i):end, 1), xy_robot_w(idx_convergence(i):end, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end), 'DisplayName', 'Est. traj. after convergence');

    if resets_wrt_est(i) > 0
        h_before_conv_ = plot(xy_robot_w(1:resets_wrt_est(i)-1, 1), xy_robot_w(1:resets_wrt_est(i)-1, 2), 'LineWidth', line_width, 'LineStyle', '--', 'Color', colors(i, 1:end), 'DisplayName', 'Est. traj. before convergence');
        h_before_conv =  plot(xy_robot_w(resets_wrt_est(i):idx_convergence(i), 1), xy_robot_w(resets_wrt_est(i):idx_convergence(i), 2), 'LineWidth', line_width, 'LineStyle', '--', 'Color', colors(i, 1:end), 'DisplayName', 'Est. traj. before convergence');
        h_reset = plot([xy_robot_w(resets_wrt_est(i)-1, 1), xy_robot_w(resets_wrt_est(i), 1)], [xy_robot_w(resets_wrt_est(i)-1, 2), xy_robot_w(resets_wrt_est(i), 2)], 'rx', 'LineWidth', 3, 'MarkerSize', marker_size, 'DisplayName', 'Reset');
        legend([h_gt_anchors, h_est_anchors, h_gt_start_point, h_gt_poses, h_gt_end_point, h_est_end_point, h_before_conv, h_after_conv, h_reset], 'NumColumns', 5, 'Location', 'south')
        xlim([-0.8 3.0])
        ylim([-0.4 4.2])
    else
        h_before_conv = plot(xy_robot_w(1:idx_convergence(i), 1), xy_robot_w(1:idx_convergence(i), 2), 'LineWidth', line_width, 'LineStyle', '--', 'Color', colors(i, 1:end), 'DisplayName', 'Est. traj. before convergence');
        legend([h_gt_anchors, h_est_anchors, h_gt_start_point, h_gt_poses, h_gt_end_point, h_est_end_point, h_before_conv, h_after_conv], 'NumColumns', 4, 'Location', 'south')
        xlim([-1 2.7])
        ylim([-0.4 4.2])
    end

    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

%% WRT initial position
colors = lines(length(tests_pre));
for i = 1:length(tests_pre)
    xy_robot = est_pose_data_cell{i}(:, 2:3);
    xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];

    figure
    set(gcf, 'Position', [100, 100, 1000, 1600]);

    if resets_wrt_est(i) > 0
        % xy_robot_w1 = rototranslation(xy_robot(1:resets_wrt_est(i)-1, :), initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        xy_robot_w1 = rototranslation(xy_robot(1:resets_wrt_est(i)-15, :), initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));     % to have the X inside the plot
        xy_robot_w2 = rototranslation(xy_robot(resets_wrt_est(i):end, :), initial_poses_gt_reset(i, 1:2), initial_poses_gt_reset(i, 3));
        xy_lnds_w = rototranslation(xy_lnds, initial_poses_gt_reset(i, 1:2), initial_poses_gt_reset(i, 3));
        h_est_poses_pre = plot(xy_robot_w1(:, 1), xy_robot_w1(:, 2), 'LineWidth', line_width, 'LineStyle', '--', 'Color', colors(i, 1:end), 'DisplayName', 'Est. trajectory before reset');
        hold on
        h_est_poses_post = plot(xy_robot_w2(:, 1), xy_robot_w2(:, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end), 'DisplayName', 'Est. trajectory after reset');
        h_reset = plot([xy_robot_w1(end, 1), xy_robot_w2(1, 1)], [xy_robot_w1(end, 2), xy_robot_w2(1, 2)], 'rx', 'LineWidth', 3, 'MarkerSize', marker_size, 'DisplayName', 'Reset');
        xy_robot_w = [xy_robot_w1; xy_robot_w2];
    else
        xy_robot_w = rototranslation(xy_robot, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        xy_lnds_w = rototranslation(xy_lnds, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        h_est_poses = plot(xy_robot_w(:, 1), xy_robot_w(:, 2), 'LineWidth', line_width, 'LineStyle', '-', 'Color', colors(i, 1:end), 'DisplayName', 'Est. trajectory');
    end

    hold on
    grid on
    axis equal

    h_est_anchors = plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'h', 'LineWidth', line_width, 'MarkerSize', marker_size, 'Color', colors(i, 1:end), 'DisplayName', 'Est. anchors');

    h_gt_anchors = plot(pos_anchors(:, 1), pos_anchors(:, 2), 'kh', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT anchors');
    for idx = 1:num_anchors
        text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'black');
    end
    % plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'kh', 'LineWidth', line_width, 'MarkerSize', marker_size);
    for idx = 1:num_anchors
        text(xy_lnds_w(idx, 1), xy_lnds_w(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', colors(i, 1:end));
    end

    if isscalar(tests_pre)
        h_gt_end_point = plot(ending_poses_gt(1), ending_poses_gt(2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT end');
    else
        h_gt_end_point = plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ko', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT end');
    end

    gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
    h_gt_poses = plot(gt_poses(:, 1), gt_poses(:, 2), 'k--', 'DisplayName', 'GT trajectory');

    h_gt_start_point = plot(initial_poses_gt(i, 1), initial_poses_gt(i, 2), 'kx', 'LineWidth', line_width, 'MarkerSize', marker_size, 'DisplayName', 'GT start');
    h_est_end_point = plot(xy_robot_w(end, 1), xy_robot_w(end, 2), 'o', 'LineWidth', line_width, 'MarkerSize', marker_size, 'Color', colors(i, 1:end), 'DisplayName', 'Est. end');

    xlim([-1 2.7])
    ylim([-0.5 4.3])

    if resets_wrt_est(i) > 0
        legend([h_gt_anchors, h_est_anchors, h_gt_start_point, h_gt_poses, h_gt_end_point, h_est_end_point, h_est_poses_pre, h_est_poses_post, h_reset], 'NumColumns', 5, 'Location', 'south')
    else
        legend([h_gt_anchors, h_est_anchors, h_gt_start_point, h_gt_poses, h_gt_end_point, h_est_end_point, h_est_poses], 'NumColumns', 5, 'Location', 'south')
    end

    xlabel('x [m]');
    ylabel('y [m]');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Ground truth + Estimated poses (WRT initial pose)
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

gt_errors_pos_wrt_initial = cell(n_robots, 1);
gt_errors_angle_wrt_initial = cell(n_robots, 1);
times = cell(n_robots, 1);

deltas = t_first_move_gt - t_first_move_bag_ros;

for i = 1:n_robots
    data_gt = squeeze(gt_data(i).robots_poses_world_history);

    new_points = 1000;
    points_end = repmat(data_gt(end, :), new_points, 1);
    data_gt = [data_gt; points_end];
    times_gt = linspace(0, (gt_data(i).n_frame + new_points) / gt_data(i).fps, size(data_gt, 1));

    x_gt = data_gt(:, 1);
    y_gt = data_gt(:, 2);
    theta_gt = unwrap(data_gt(:, 3));
    
    data_est = est_pose_data_cell{i};
    times_est = data_est(:, 1);

    t_start_gt_x = deltas(i);
    t_start_gt_y = deltas(i);
    t_start_gt_theta = deltas(i);

    idx_gt_start_x = find(times_gt > t_start_gt_x, 1);
    idx_gt_start_y = find(times_gt > t_start_gt_y, 1);
    idx_gt_start_theta = find(times_gt > t_start_gt_theta, 1);

    x_gt = x_gt(idx_gt_start_x:end);
    y_gt = y_gt(idx_gt_start_y:end);
    theta_gt = theta_gt(idx_gt_start_theta:end);

    times_gt_x = times_gt(idx_gt_start_x:end) - t_start_gt_x;
    x_gt = interp1(times_gt_x, x_gt, times_est, 'linear');

    times_gt_y = times_gt(idx_gt_start_y:end) - t_start_gt_y;
    y_gt = interp1(times_gt_y, y_gt, times_est, 'linear');

    times_gt_theta = times_gt(idx_gt_start_theta:end) - t_start_gt_theta;
    theta_gt = interp1(times_gt_theta, theta_gt, times_est, 'linear');

    % remove nans
    times_gt = times_est;
    x_gt(isnan(x_gt)) = x_gt(find(~isnan(x_gt), 1, 'first'));
    y_gt(isnan(y_gt)) = y_gt(find(~isnan(y_gt), 1, 'first'));
    theta_gt(isnan(theta_gt)) = theta_gt(find(~isnan(theta_gt), 1, 'first'));

    if resets_wrt_est(i) > 0
        initial_poses_gt_reset(i, :) = [x_gt(resets_wrt_est(i)), y_gt(resets_wrt_est(i)), theta_gt(resets_wrt_est(i))];
    end

    xy_robot = est_pose_data_cell{i}(:, 2:3);
    theta_robot = est_pose_data_cell{i}(:, 4);
    xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];

    if resets_wrt_est(i) > 0
        xy_robot_w1 = rototranslation(xy_robot(1:resets_wrt_est(i)-1, :), initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        xy_robot_w2 = rototranslation(xy_robot(resets_wrt_est(i):end, :), initial_poses_gt_reset(i, 1:2), initial_poses_gt_reset(i, 3));
        xy_lnds_w = rototranslation(xy_lnds, initial_poses_gt_reset(i, 1:2), initial_poses_gt_reset(i, 3));
        hold on
        xy_robot_w = [xy_robot_w1; xy_robot_w2];
        theta_robot_w = [theta_robot(1:resets_wrt_est(i)-1, :) + initial_poses_gt(i, 3); theta_robot(resets_wrt_est(i):end, :) + initial_poses_gt_reset(i, 3)];
    else
        xy_robot_w = rototranslation(xy_robot, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        xy_lnds_w = rototranslation(xy_lnds, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        theta_robot_w = theta_robot + initial_poses_gt(i, 3);
    end

    x_est = xy_robot_w(:, 1);
    y_est = xy_robot_w(:, 2);

    if resets_wrt_est(i) > 0
        theta_est = [unwrap(theta_robot_w(1:resets_wrt_est(i)-1)); unwrap(theta_robot_w(resets_wrt_est(i):end))];
    else
        theta_est = unwrap(theta_robot_w(:, 1));
    end

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times_gt, x_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, x_est, 'r', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times_gt, y_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, y_est, 'g', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times_gt, theta_gt, 'k--', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, theta_est, 'b', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');

    gt_errors_pos_wrt_initial{i} = sqrt((x_gt - x_est).^2 + (y_gt- y_est).^2);
    gt_errors_angle_wrt_initial{i} = abs(theta_gt - theta_est);
    times{i} = times_gt;

    diff_xy = diff(gt_errors_pos_wrt_initial{i});
    idx_stop = find(abs(diff_xy) < 1e-4);
    if isempty(idx_stop)
        idx_stop = length(x_gt);
    else
        last_one = find(diff(idx_stop) ~= 1, 1, 'last') + 1;
        if isempty(last_one)
            last_one = 1;
        end
        idx_stop = idx_stop(last_one);
    end

    gt_errors_pos_wrt_initial{i} = gt_errors_pos_wrt_initial{i}(1:idx_stop);
    gt_errors_angle_wrt_initial{i} = gt_errors_angle_wrt_initial{i}(1:idx_stop);
    times{i} = times{i}(1:idx_stop);
end

%% Ground truth + Estimated poses errors (WRT initial pose)
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

tf = 120;
for i = 1:n_robots
    t = tiledlayout(T, 1, 2, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    err_pos = gt_errors_pos_wrt_initial{i};
    plot(times{i}, err_pos, 'r', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, repmat(mean(err_pos), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('e_{xy} [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    if resets_wrt_est(i) > 0
        ax2 = axes('Position', [0.261, 0.0735, 0.185, 0.072]);
        plot(times{i}, err_pos, 'r', 'LineWidth', line_width);
        hold on
        plot(times_est, repmat(mean(err_pos), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
        plot(times_est, repmat(mean(err_pos(resets_wrt_est(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xlim([times{i}(resets_wrt_est(i)) tf]);
        ylim([0, max(err_pos(resets_wrt_est(i):end-10))])
        set(ax2, 'Color', 'none');
        set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
    end

    ax2 = nexttile(t, 2);
    err_angle = gt_errors_angle_wrt_initial{i};
    plot(times{i}, err_angle, 'b', 'LineWidth', line_width);
    grid on;
    hold on;
    plot(times_est, repmat(mean(err_angle), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
    if resets_wrt_est(i) > 0
        xline(t_resets_wrt_est(i), 'k-.', 'LineWidth', line_width);
    end
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('e_\theta [rad]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    if resets_wrt_est(i) > 0
        ax2 = axes('Position', [0.7515, 0.0735, 0.185, 0.072]);
        plot(times{i}, err_angle, 'b', 'LineWidth', line_width);
        hold on
        plot(times_est, repmat(mean(err_angle), 1, size(times_est, 1)), 'k--', 'LineWidth', line_width);
        plot(times_est, repmat(mean(err_angle(resets_wrt_est(i):end)), 1, size(times_est, 1)), 'k:', 'LineWidth', line_width);
        xlim([times{i}(resets_wrt_est(i)) tf]);
        ylim([0, max(err_angle(resets_wrt_est(i):end-10))])
        set(ax2, 'Color', 'none');
        set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
    end
end

%% Landmarks
robot = 2;
figure
set(gcf, 'Position', [100, 100, 1000, 400]);

x = est_lnds_data{robot}.x';
y = est_lnds_data{robot}.y';
len_time = size(x, 2);

time = est_pose_data_cell{robot}(:, 1);
colors = {'#0072BD', '#D95319', '#EDB120', '#7E2F8E', '#77AC30', '#4DBEEE', '#A2142F', '#1010FF'};
names = {'Anchor1', 'Anchor2', 'Anchor3', 'Anchor4', 'Anchor5', 'Anchor6', 'Anchor7', 'Anchor8'};

tiledlayout(1, 2, 'TileSpacing', 'compact')
for idx = 1:2
    nexttile
    for tag = 1:n_anchors
        posLoc = [x(tag, :); y(tag, :)];
        if resets_wrt_est(robot) > 0
            % posGlob = zeros(3, nPassi);
            % 
            % tResetVect = [1, tResets{robot}, nPassi];
            % for t_idx = 1:length(tResetVect)-1
            %     t0 = tResetVect(t_idx);
            %     t1 = tResetVect(t_idx+1);
            % 
            %     T = TsGL{robot}(:, :, t_idx);
            %     posGlob(:, t0:t1) = T*posLoc(:, t0:t1);
            % end
        else
            posGlob = rototranslation(posLoc', initial_poses_gt(robot, 1:2), initial_poses_gt(robot, 3));
        end
        if idx == 1
            plot(time, posGlob(:, 1), 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, pos_anchors(tag, 1)*ones(1, len_time), '--', 'LineWidth', line_width, 'Color', colors{tag}, 'DisplayName', '')
        else
            plot(time, posGlob(:, 2), 'LineWidth', 1, 'Color', colors{tag}, 'DisplayName', names{tag})
            hold on
            plot(time, pos_anchors(tag, 2)*ones(1, len_time), '--', 'LineWidth', line_width, 'Color', colors{tag}, 'DisplayName', '')
        end
    end
    % if pruning
    %     for i = 1:length(stepStartPruning{robot})
    %         if stepStartPruning{robot}(i) > t_max
    %             continue
    %         end
    %         if i == 1
    %             xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
    %         else
    %             xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'HandleVisibility', 'off');
    %         end
    %     end
    % end
    % if sharing
    %     first = 1;
    %     for t = startSharing
    %         if t ~= -1
    %             if first
    %                 first = 0;
    %                 xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
    %             else
    %                 xline(t, '-.k', 'LineWidth', 1, 'HandleVisibility', 'off');
    %             end
    %         end
    %     end
    % end
    % if ~isempty(tResets{robot})
    %     for i = 1:length(tResets{robot})
    %         if i == 1
    %             xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'DisplayName', 'Reset');
    %         else
    %             xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    %         end
    %     end
    % end
    
    grid on
    xlabel('time [s]');
    if idx == 1
        ylabel('x [m]');
    else
        ylabel('y [m]');
        legend('location', 'eastoutside');
    end
    yticks(-4:0.5:4);
    xlim([t0 tf]);

    ax = gca;
    ax.FontSize = 12;
end

sgtitle(sprintf('Absolute errors robot %d', robot), 'FontSize', 16)

set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');

% if length(dbstack) == 1
%     saveas(gcf, sprintf('./Article/robot%d.eps', robot), 'epsc');
% end

%% Print results
fd = fopen(['data/errors_', bag_name, '.txt'], 'w');

fprintf("Position error robots (WRT initial pose):\n")
fprintf(fd, "Position error robots (WRT initial pose):\n");
for robot = 1:n_robots
    error_pos = mean(gt_errors_pos_wrt_initial{robot});
    fprintf(" - Robot %d: %.4f m\n", robot, error_pos)
    fprintf(fd, " - Robot %d: %.4f m\n", robot, error_pos);
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Angle error robots (WRT initial pose):\n")
fprintf(fd, "Angle error robots (WRT initial pose):\n");
for robot = 1:n_robots
    error_angle = mean(gt_errors_angle_wrt_initial{robot});
    fprintf(" - Robot %d: %.4f rad\n", robot, error_angle)
    fprintf(fd, " - Robot %d: %.4f rad\n", robot, error_angle);
end
fprintf("\n")
fprintf(fd, "\n");


fprintf("Position error robots (WRT anchors):\n")
fprintf(fd, "Position error robots (WRT anchors):\n");
for robot = 1:n_robots
    error_pos = mean(gt_errors_pos{robot});
    fprintf(" - Robot %d: %.4f m\n", robot, error_pos)
    fprintf(fd, " - Robot %d: %.4f m\n", robot, error_pos);
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Position error robots (WRT anchors, after convergence):\n")
fprintf(fd, "Position error robots (WRT anchors, after convergence):\n");
for robot = 1:n_robots
    if idx_convergence(robot) > 0
        error_pos = mean(gt_errors_pos{robot}(idx_convergence(robot):end));
        fprintf(" - Robot %d: %.4f m\n", robot, error_pos)
        fprintf(fd, " - Robot %d: %.4f m\n", robot, error_pos);
    else
        fprintf(" - Robot %d: --------\n", robot)
        fprintf(fd, " - Robot %d: --------\n", robot);
    end
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Angle error robots (WRT anchors):\n")
fprintf(fd, "Angle error robots (WRT anchors):\n");
for robot = 1:n_robots
    error_angle = mean(gt_errors_angle{robot});
    fprintf(" - Robot %d: %.4f rad\n", robot, error_angle)
    fprintf(fd, " - Robot %d: %.4f rad\n", robot, error_angle);
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Angle error robots (WRT anchors, after convergence):\n")
fprintf(fd, "Angle error robots (WRT anchors, after convergence):\n");
for robot = 1:n_robots
    if idx_convergence(robot) > 0
        error_angle = mean(gt_errors_angle{robot}(idx_convergence(robot):end));
        fprintf(" - Robot %d: %.4f rad\n", robot, error_angle)
        fprintf(fd, " - Robot %d: %.4f rad\n", robot, error_angle);
    else
        fprintf(" - Robot %d: ----------\n", robot)
        fprintf(fd, " - Robot %d: ----------\n", robot);
    end
end
fprintf("\n")
fprintf(fd, "\n");

% fprintf("Final position error robots:\n")
% fprintf(fd, "Final position error robots:\n");
% for robot = 1:n_robots
%     error_final_pos = norm(ending_poses_gt(robot, 1:2) - ending_poses_est(robot, :));
%     fprintf(" - Robot %d: %.4f m\n", robot, error_final_pos)
%     fprintf(fd, " - Robot %d: %.4f m\n", robot, error_final_pos);
% end
% fprintf("\n")
% fprintf(fd, "\n");

fprintf("Landmark errors after RoMa:\n")
fprintf(fd, "Landmark errors after RoMa:\n");
for robot = 1:n_robots
    xy_lnds = [est_lnds_data{robot}.x(end, :)', est_lnds_data{robot}.y(end, :)'];
    [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.3, 7);
    xy_lnds_post_roma = (R * xy_lnds' + t)';
    error_post_roma = mean(vecnorm(xy_lnds_post_roma - pos_anchors, 2, 2));
    fprintf(" - Robot %d: %.4f m\n", robot, error_post_roma)
    fprintf(fd, " - Robot %d: %.4f m\n", robot, error_post_roma);
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Final distances robot-landmark:\n")
fprintf(fd, "Final distances robot-landmark:\n");
for robot = 1:n_robots
    fprintf("\tRobot %d:\n", robot)
    fprintf(fd, "\tRobot %d:\n", robot);
    distances_est = zeros(1, n_anchors);
    distances_gt = zeros(1, n_anchors);
    for lnd = 1:n_anchors
        % Estimates
        xy_robot_est = est_pose_data_cell{robot}(end, 2:3);
        xy_lnd_est = [est_lnds_data{robot}.x(end, lnd), est_lnds_data{robot}.y(end, lnd)];
        distances_est(lnd) = norm(xy_lnd_est - xy_robot_est);

        % GT
        xy_robot_gt = squeeze(gt_data(robot).robots_poses_world_history(end, 1, 1:2))';
        xy_lnd_gt = pos_anchors(lnd, :);
        distances_gt(lnd) = norm(xy_lnd_gt - xy_robot_gt);
    end
    % Print results
    fprintf("\t\tGT:   ")
    fprintf(fd, "\t\tGT:   ");
    for lnd = 1:n_anchors
        fprintf("%.4f ", distances_gt(lnd))
        fprintf(fd, "%.4f ", distances_gt(lnd));
    end
    fprintf("\n")
    fprintf(fd, "\n");
    fprintf("\t\tEST:  ")
    fprintf(fd, "\t\tEST:  ");
    for lnd = 1:n_anchors
        fprintf("%.4f ", distances_est(lnd))
        fprintf(fd, "%.4f ", distances_est(lnd));
    end
    fprintf("\n")
    fprintf(fd, "\n");
    fprintf("\t\t----------------------------------------------\n")
    fprintf(fd, "\t\t----------------------------------------------\n");
    fprintf("\t\tDIFF: ")
    fprintf(fd, "\t\tDIFF: ");
    for lnd = 1:n_anchors
        fprintf("%.4f ", abs(distances_gt(lnd) - distances_est(lnd)))
        fprintf(fd, "%.4f ", abs(distances_gt(lnd) - distances_est(lnd)));
    end
    fprintf(" --> MEAN: %.4f ", mean(abs(distances_gt - distances_est)))
    fprintf(fd, " --> MEAN: %.4f ", mean(abs(distances_gt - distances_est)));
    fprintf("\n")
    fprintf(fd, "\n");
end
fprintf("\n")
fprintf(fd, "\n");

fprintf("Final distances landmark-landmark:\n")
fprintf(fd, "Final distances landmark-landmark:\n");
for robot = 1:n_robots
    fprintf("\tRobot %d:\n", robot)
    fprintf(fd, "\tRobot %d:\n", robot);
    distances_est = [];
    distances_gt = [];
    for lnd1 = 1:n_anchors-1
        for lnd2 = lnd1+1:n_anchors
            % Estimates
            xy_lnd1_est = [est_lnds_data{robot}.x(end, lnd1), est_lnds_data{robot}.y(end, lnd1)];
            xy_lnd2_est = [est_lnds_data{robot}.x(end, lnd2), est_lnds_data{robot}.y(end, lnd2)];
            distances_est = [distances_est, norm(xy_lnd1_est - xy_lnd2_est)];

            % GT
            xy_lnd1_gt = pos_anchors(lnd1, :);
            xy_lnd2_gt = pos_anchors(lnd2, :);
            distances_gt = [distances_gt, norm(xy_lnd1_gt - xy_lnd2_gt)];
        end
    end
    % Print results
    fprintf("\t\tGT:   ")
    fprintf(fd, "\t\tGT:   ");
    for lnd = 1:length(distances_gt)
        fprintf("%.4f ", distances_gt(lnd))
        fprintf(fd, "%.4f ", distances_gt(lnd));
    end
    fprintf("\n")
    fprintf(fd, "\n");
    fprintf("\t\tEST:  ")
    fprintf(fd, "\t\tEST:  ");
    for lnd = 1:length(distances_est)
        fprintf("%.4f ", distances_est(lnd))
        fprintf(fd, "%.4f ", distances_est(lnd));
    end
    fprintf("\n")
    fprintf(fd, "\n");
    fprintf("\t\t-------------------------------------------------------------------------------------------------------------------------------------------------\n")
    fprintf(fd, "\t\t-------------------------------------------------------------------------------------------------------------------------------------------------\n");
    fprintf("\t\tDIFF: ")
    fprintf(fd, "\t\tDIFF: ");
    for lnd = 1:length(distances_est)
        fprintf("%.4f ", abs(distances_gt(lnd) - distances_est(lnd)))
        fprintf(fd, "%.4f ", abs(distances_gt(lnd) - distances_est(lnd)));
    end
    fprintf("\n\t\t      --> MEAN: %.4f ", mean(abs(distances_gt - distances_est)))
    fprintf(fd, "\n\t\t      --> MEAN: %.4f ", mean(abs(distances_gt - distances_est)));
    fprintf("\n")
    fprintf(fd, "\n");
end

fclose(fd);

%% Save workspace
roslam_data = struct();

roslam_data.wheel_radius = wheel_radius;
roslam_data.wheels_separation = wheels_separation;

roslam_data.duration = t_end-t_start;
roslam_data.num_robots = n_robots;
roslam_data.num_anchors_fixed = n_anchors;
roslam_data.anchors_positions = pos_anchors;

roslam_data.ground_truth = gt_data;
roslam_data.robot_odometry = est_lnds_data;
roslam_data.wheels_odometry = js_data;
roslam_data.uwb_anchors_distances = uwb_anchors_data;

save('roslam_data.mat', 'roslam_data');

save(['./data/postprocess_', bag_name, '.mat'])