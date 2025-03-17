%% Cleanup
clc; close all
bag_field_B2

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

%% Read bag and topics names
test = 6;
bag_name = ['20250314_test', num2str(test)];
path_prefix = '/home/lorenzo/Github/University/turtlebot3-utv-fork/logs/';
full_path = strcat(path_prefix, bag_name, '/', bag_name, '_0.db3');
bag = ros2bagreader(full_path);
all_topics = string(bag.AvailableTopics.Properties.RowNames);
t_start = bag.StartTime;
t_end = bag.EndTime;

%% Get number of robots and topics names
n_robots = -1;
topics_names = strings(0);
for i = 1:length(all_topics)
    topic = all_topics(i);

    id_str = regexp(topic, '/robot(\d+)/', 'tokens');
    if isempty(id_str)

    else
        id = str2double(id_str{1}{1});
        if id > n_robots
            n_robots = id;
        end
    
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
fprintf('Duration: %.2f s\n', t_end-t_start);
fprintf('Found: %d robots, %d anchors\n', n_robots, n_anchors);
fprintf('Anchors in positions:\n');
for i = 1:length(pos_anchors)
    fprintf('\t- (%.2f, %.2f)\n', pos_anchors(i,1), pos_anchors(i,2));
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
    gt_data(i).fps = 30.0;
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

% Plot anchors distances
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(uwb_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');

colors = lines(8);

t0 = 1;
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
        plot(timesA, distancesA(:,k), 'LineWidth', 1, 'Color', colors(k, :));
        hold on;
    end
    grid on;
    hold on;
    for j = 1:size(distancesA, 2)
        nan_indices = isnan(distancesA(:, j));
        valid_indices = ~nan_indices;
        distancesA(:, j) = interp1(timesA(valid_indices), distancesA(valid_indices, j), timesA, 'linear', 'extrap');
        plot(timesA(nan_indices), distancesA(nan_indices, j), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    xlim([t0 inf]);
    ylim([0, 5]);
    xlabel('time [s]');
    ylabel('distance [m]');
    legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Location', 'northwest', 'NumColumns', 4);
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t,2);
    for k = 1:size(distancesB, 2)
        plot(timesB, distancesB(:,k), 'LineWidth', 1, 'Color', colors(k+4,:));
        hold on;
    end
    grid on;
    hold on;
    for j = 1:size(distancesB, 2)
        nan_indices = isnan(distancesB(:, j));
        valid_indices = ~nan_indices;
        distancesB(:, j) = interp1(timesB(valid_indices), distancesB(valid_indices, j), timesB, 'linear', 'extrap');
        plot(timesB(nan_indices), distancesB(nan_indices, j), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    xlim([t0 inf]);
    ylim([0, 3.5]);
    xlabel('time [s]');
    ylabel('distance [m]');
    legend('Anchor 5', 'Anchor 6', 'Anchor 7', 'Anchor 8', 'Location', 'northwest', 'NumColumns', 4);
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');
end


%% Joint states
% if any(contains(topics_names, 'joint_states'))
%     js_topics_names = all_topics(contains(all_topics, 'joint_states'));
%     js_data = cell(length(js_topics_names), 1);
%     for i = 1:length(js_topics_names)
%         % robot i
%         js_topic = select(bag, 'Topic', js_topics_names(i));
%         js_msg = readMessages(js_topic);
% 
%         if contains(js_msg{1}.name{1}, 'right')
%             idx_right = 1;
%             idx_left = 2;
%         else
%             idx_right = 2;
%             idx_left = 1;
%         end
% 
%         t_start_js = double(js_msg{1}.header.stamp.sec) + double(js_msg{1}.header.stamp.nanosec)/1e9;
%         times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, js_msg);
%         times = times - t_start_js;
% 
%         pos_left = cellfun(@(m) m.position(idx_left), js_msg);
%         pos_left = [0; diff(pos_left) * wheel_radius];
% 
%         pos_right = cellfun(@(m) m.position(idx_right), js_msg);
%         pos_right = [0; diff(pos_right) * wheel_radius];
% 
%         js_data{i} = [times, pos_right, pos_left];
%     end
% end
% 
% % Plot wheels velocities
% figure;
% tiledlayout(length(js_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
% set(gcf, 'Position', [100, 100, 1200, 800]);
% 
% t0 = 1;
% for i = 1:length(js_topics_names)
%     data = js_data{i};
% 
%     times = data(:, 1);
%     velocities = data(:, 2:end);
% 
%     nexttile;
%     plot(times, velocities, 'LineWidth', 1);
%     grid on;
%     hold on;
% 
%     xlim([t0 inf]);
% 
%     title(['Robot ', num2str(i)]);
%     xlabel('time [s]');
%     ylabel('velocity [m/s]');
%     legend('Right wheel', 'Left wheel', 'Location', 'northwest');
%     set(gca, 'FontSize', 12, 'FontWeight', 'bold');
% end

%% Odom
% if any(contains(topics_names, 'odom'))
%     odom_topics_names = all_topics(contains(all_topics, 'odom'));
%     odom_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
%     odom_data_cell = cell(length(odom_topics_names), 1);
%     for i = 1:length(odom_topics_names)
%         odom_topic = select(bag, 'Topic', odom_topics_names(i));
%         odom_msg = readMessages(odom_topic);
% 
%         t_start_odom = double(odom_msg{1}.header.stamp.sec) + double(odom_msg{1}.header.stamp.nanosec)/1e9;
%         odom_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, odom_msg);
%         odom_data(i).times = odom_data(i).times - t_start_odom;
%         odom_data(i).x = cellfun(@(m) m.pose.pose.position.x, odom_msg);
%         odom_data(i).y = cellfun(@(m) m.pose.pose.position.y, odom_msg);
%         odom_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, odom_msg);
%         odom_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, odom_msg);
%         for k = 1:length(odom_data(i).v_ang)
%             if abs(odom_data(i).v_ang(k)) > 10
%                 odom_data(i).v_ang(k) = odom_data(i).v_ang(k-1);
%             end
%         end
% 
%         quat_w = cellfun(@(m) m.pose.pose.orientation.w, odom_msg);
%         quat_x = cellfun(@(m) m.pose.pose.orientation.x, odom_msg);
%         quat_y = cellfun(@(m) m.pose.pose.orientation.y, odom_msg);
%         quat_z = cellfun(@(m) m.pose.pose.orientation.z, odom_msg);
%         rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
%         odom_data(i).theta = rpy(:, 1);
% 
%         odom_data_cell{i} = [odom_data(i).times(:), odom_data(i).x(:), odom_data(i).y(:), odom_data(i).theta(:), odom_data(i).v_lin(:), odom_data(i).v_ang(:)];
%     end
% end
% 
% % Plot robot velocities
% figure;
% tiledlayout(length(odom_topics_names), 2, 'TileSpacing', 'Compact', 'Padding', 'Compact');
% set(gcf, 'Position', [100, 100, 1200, 800]);
% 
% t0 = 1;
% for i = 1:length(odom_topics_names)
%     data = odom_data_cell{i};
% 
%     times = data(:, 1);
%     velocities = data(:, end-1:end);
% 
%     nexttile;
%     plot(times, velocities(:, 1), 'LineWidth', 1);
%     grid on;
%     hold on;
%     xlim([t0 inf]);
% 
%     title(['Robot ', num2str(i), ' - Linear velocity']);
%     xlabel('time [s]');
%     ylabel('velocity [m/s]');
%     set(gca, 'FontSize', 12, 'FontWeight', 'bold');
% 
%     nexttile;
%     plot(times, velocities(:, 2), 'Color', '#D95319', 'LineWidth', 1);
%     grid on;
%     hold on;
%     xlim([t0 inf]);
% 
%     title(['Robot ', num2str(i), ' - Angular velocity']);
%     xlabel('time [s]');
%     ylabel('velocity [rad/s]');
%     set(gca, 'FontSize', 12, 'FontWeight', 'bold');
% end

%% Ground truth
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(n_robots, 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

t0 = 1;
for i = 1:n_robots
    data = squeeze(gt_data(i).robots_poses_world_history);

    times = linspace(1, gt_data(i).n_frame / gt_data(i).fps, size(gt_data(i).robots_poses_world_history, 1));
    x = data(:, 1);
    y = data(:, 2);
    theta = unwrap(data(:, 3));



    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d GT', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times, x, 'Color', 'r', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times, y, 'Color', 'g', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times, theta, 'Color', 'b', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Estimated poses
if any(contains(topics_names, 'estimated_pose'))
    est_pose_topics_names = all_topics(contains(all_topics, 'estimated_pose'));
    est_pose_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'var_x', [], 'var_y', [], 'var_theta', []);
    est_pose_data_cell = cell(length(est_pose_topics_names), 1);
    for i = 1:length(est_pose_topics_names)
        est_pose_topic = select(bag, 'Topic', est_pose_topics_names(i));
        est_pose_msg = readMessages(est_pose_topic);
        
        t_start_est_pose = double(est_pose_msg{1}.header.stamp.sec) + double(est_pose_msg{1}.header.stamp.nanosec)/1e9;
        est_pose_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, est_pose_msg);
        est_pose_data(i).times = est_pose_data(i).times - t_start_est_pose;
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
    end
end
clear est_pose_msg

% Plot robots poses
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

t0 = 1;
for i = 1:length(est_pose_topics_names)
    data = est_pose_data_cell{i};

    times = data(:, 1);
    x = data(:, 2);
    y = data(:, 3);
    theta = unwrap(data(:, 4));

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times, x, 'Color', 'r', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times, y, 'Color', 'g', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times, theta, 'Color', 'b', 'LineWidth', 1);
    grid on;
    hold on;
    xlim([t0 inf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Estimated landmarks
if any(contains(topics_names, 'estimated_landmarks'))
    est_lnds_topics_names = all_topics(contains(all_topics, 'estimated_landmarks'));
    est_lnds_topics_names = all_topics(contains(all_topics, 'estimated_landmarks'));
    est_lnds_data = cell(length(est_lnds_topics_names), 1);
    
    for i = 1:length(est_lnds_topics_names)
        est_lnds_topic = select(bag, 'Topic', est_lnds_topics_names(i));
        est_lnds_msg = readMessages(est_lnds_topic);
    
        t_start_est_pose = double(est_lnds_msg{1}.header.stamp.sec) + double(est_lnds_msg{1}.header.stamp.nanosec)/1e9;
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, est_lnds_msg);
        times = times - t_start_est_pose;
    
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
for i = 1:length(tests_pre)
    initial_poses_gt(i, :) = squeeze(gt_data(i).robots_poses_world_history(1, 1, :));
    ending_poses_gt(i, :) = squeeze(gt_data(i).robots_poses_world_history(end, 1, :));
end
% 
% figure;
% hold on;
% grid on;
% pts_keys = keys(pts);
% for idx = 1:num_anchors
%     key = pts_keys{idx};
%     xy = pts(key);
% 
%     num = key(2);
%     plot(xy(1), xy(2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
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
%     quiver(x, y, dx, dy, 'LineWidth', 1, 'MaxHeadSize', 1);
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
%     plot(xy_w(:, 1), xy_w(:, 2), 'LineWidth', 1, 'LineStyle', '-', 'Color', colors(i, 1:end))
% end
% 
% for i = 1:length(tests_pre)
%     data = est_lnds_data{i};
% 
%     xy = [data.x(end, :)', data.y(end, :)'];
%     xy_w = rototranslation(xy, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
%     xy_w = rototranslation(xy, ending_poses_gt(i, 1:2), ending_poses_gt(i, 3)-theta);
% 
%     plot(xy_w(:, 1), xy_w(:, 2), 'kh', 'Linewidth', 2, 'MarkerSize', 12);
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
%     plot(pose_robot(:, 1), pose_robot(:, 2), 'LineWidth', 1, 'LineStyle', '-', 'Color', colors(i, 1:end))
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
%         quiver(x, y, dx, dy, 'LineWidth', 1, 'MaxHeadSize', 1, 'Color', 'r');
%     end
% 
%     xlim([-1 2.7])
%     ylim([-0.5 4.3])
% end

%% WRT initial position
for i = 1:length(tests_pre)
    xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];
    [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 100, 0.15, 7);
    xy_lnds_w = (R * xy_lnds' + t)';

    xy_robot = est_pose_data_cell{i}(:, 2:3);
    xy_robot_w = rototranslation(xy_robot, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));

    figure
    plot(xy_robot_w(:, 1), xy_robot_w(:, 2), 'LineWidth', 1, 'LineStyle', '-', 'Color', colors(i, 1:end))
    hold on
    grid on
    axis equal

    xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];
    xy_lnds_w = rototranslation(xy_lnds, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));

    plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'kh', 'Linewidth', 2, 'MarkerSize', 12);

    plot(pos_anchors(:, 1), pos_anchors(:, 2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
    for idx = 1:num_anchors
        text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
    end
    plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'kh', 'Linewidth', 2, 'MarkerSize', 12);
    for idx = 1:num_anchors
        text(xy_lnds_w(idx, 1), xy_lnds_w(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'black');
    end

    if length(tests_pre) == 1
        plot(ending_poses_gt(1), ending_poses_gt(2), 'ro', 'Linewidth', 2, 'MarkerSize', 12);
    else
        plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ro', 'Linewidth', 2, 'MarkerSize', 12);
    end

    gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
    plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', colors(i, 1:end))

    xlim([-1 2.7])
    ylim([-0.5 4.3])
end

%%
% for i = 1:length(tests_pre)
%     xy_lnds = [est_lnds_data{i}.x(end, :)', est_lnds_data{i}.y(end, :)'];
%     [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 100, 0.15, 7);
%     xy_lnds_w = (R * xy_lnds' + t)';
% 
%     figure
%     plot(pos_anchors(:, 1), pos_anchors(:, 2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
%     hold on
%     for idx = 1:num_anchors
%         text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
%     end
%     plot(xy_lnds_w(:, 1), xy_lnds_w(:, 2), 'kh', 'Linewidth', 2, 'MarkerSize', 12);
%     for idx = 1:num_anchors
%         text(xy_lnds_w(idx, 1), xy_lnds_w(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'black');
%     end
% 
%     if length(tests_pre) == 1
%         plot(ending_poses_gt(1), ending_poses_gt(2), 'ro', 'Linewidth', 2, 'MarkerSize', 12);
%     else
%         plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ro', 'Linewidth', 2, 'MarkerSize', 12);
%     end
% 
%     xy_robot = est_pose_data_cell{i}(end, 2:3);
%     xy_robot_w = (R * xy_robot' + t)';
%     plot(xy_robot_w(1), xy_robot_w(2), 'ko', 'Linewidth', 2, 'MarkerSize', 12);
% 
%     gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
%     plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', colors(i, 1:end))
% 
%     data = est_pose_data_cell{i};
%     xy = data(:, 2:3);
%     xy_w = (R * xy' + t)';
%     plot(xy_w(:, 1), xy_w(:, 2), 'LineWidth', 1, 'LineStyle', '-', 'Color', colors(i, 1:end))
% 
%     grid on
%     axis equal
% end

%% WRT anchors
ending_poses_est = zeros(n_robots, 2);
for i = 1:length(tests_pre)
    len_bag = length(est_lnds_data{i}.times);
    xy_lnds_w = zeros(len_bag, 8, 2);
    xy_robot_w = zeros(len_bag, 2);
    for k = 1:len_bag
        xy_lnds = [est_lnds_data{i}.x(k, :)', est_lnds_data{i}.y(k, :)'];
        xy_robot = est_pose_data_cell{i}(k, 2:3);
        if k < len_bag / 3
            n_inl = 5;
        elseif k < 2 * len_bag / 3
            n_inl = 6;
        else
            n_inl = 7;
        end
        [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.4-0.2*k/len_bag, 7);
        if isempty(inl)
            xy_lnds_w(k, :, :) = rototranslation(xy_lnds, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
            xy_robot_w(k, :) = rototranslation(xy_robot, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        else
            xy_lnds_w(k, :, :) = (R * xy_lnds' + t)';
            xy_robot_w(k, :) = (R * xy_robot' + t)';
        end
    end
    ending_poses_est(i, :) = xy_robot_w(end, :);

    figure
    plot(pos_anchors(:, 1), pos_anchors(:, 2), 'kh', 'Linewidth', 2, 'MarkerSize', 12);
    hold on
    grid on
    axis equal
    gt_poses = squeeze(gt_data(i).robots_poses_world_history(:, 1, :));
    plot(gt_poses(:, 1), gt_poses(:, 2), 'LineStyle', '--', 'Color', 'k')

    for idx = 1:num_anchors
        text(pos_anchors(idx, 1), pos_anchors(idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
    end
    plot(xy_lnds_w(end, :, 1), xy_lnds_w(end, :, 2), 'h', 'Linewidth', 2, 'MarkerSize', 12, 'Color', colors(i, 1:end));
    for idx = 1:num_anchors
        text(xy_lnds_w(end, idx, 1), xy_lnds_w(end, idx, 2)+0.05, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', colors(i, 1:end));
    end

    if length(tests_pre) == 1
        plot(ending_poses_gt(1), ending_poses_gt(2), 'ko', 'Linewidth', 2, 'MarkerSize', 12);
        plot(initial_poses_gt(1), initial_poses_gt(2), 'kx', 'Linewidth', 2, 'MarkerSize', 18);
    else
        plot(ending_poses_gt(i, 1), ending_poses_gt(i, 2), 'ko', 'Linewidth', 2, 'MarkerSize', 12);
        plot(initial_poses_gt(i, 1), initial_poses_gt(i, 2), 'kx', 'Linewidth', 2, 'MarkerSize', 18);
    end

    plot(xy_robot_w(end, 1), xy_robot_w(end, 2), 'ko', 'Linewidth', 2, 'MarkerSize', 12, 'Color', colors(i, 1:end));
    plot(xy_robot_w(1, 1), xy_robot_w(1, 2), 'kx', 'Linewidth', 2, 'MarkerSize', 12, 'Color', colors(i, 1:end));

    xlim([-1 2.7])
    ylim([-0.5 4.3])

    xlabel("x [m]")
    ylabel("y [m]")
    
    plot(xy_robot_w(:, 1), xy_robot_w(:, 2), 'LineWidth', 1, 'LineStyle', '-', 'Color', colors(i, 1:end))
end

%% Ground truth + Estimated poses
figure;
set(gcf, 'Position', [100, 100, 1000, 1600]);

T = tiledlayout(length(est_pose_topics_names), 1, 'TileSpacing', 'Loose', 'Padding', 'Compact');

tf = 120;
for i = 1:n_robots
    data_gt = squeeze(gt_data(i).robots_poses_world_history);

    new_points = 600;
    inizio = repmat(data_gt(1, :), new_points / 2, 1);
    fine = repmat(data_gt(end, :), new_points / 2, 1);
    data_gt = [inizio; data_gt; fine];
    times_gt = linspace(1, (gt_data(i).n_frame + new_points) / gt_data(i).fps, size(data_gt, 1));

    x_gt = data_gt(:, 1);
    y_gt = data_gt(:, 2);
    theta_gt = unwrap(data_gt(:, 3));

    len_bag = size(est_pose_data_cell{i}, 1);
    xy_lnds_w = zeros(len_bag, 8, 2);
    xy_robot_w = zeros(len_bag, 2);
    for k = 1:len_bag
        xy_lnds = [est_lnds_data{i}.x(k, :)', est_lnds_data{i}.y(k, :)'];
        xy_robot = est_pose_data_cell{i}(k, 2:3);
        if k < len_bag / 3
            n_inl = 5;
        elseif k < 2 * len_bag / 3
            n_inl = 6;
        else
            n_inl = 7;
        end
        [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.4-0.2*k/len_bag, 7);
        if isempty(inl)
            xy_robot_w(k, :) = rototranslation(xy_robot, initial_poses_gt(i, 1:2), initial_poses_gt(i, 3));
        else
            xy_robot_w(k, :) = (R * xy_robot' + t)';
        end
    end

    data_est = est_pose_data_cell{i};
    times_est = data_est(:, 1);
    x_est = xy_robot_w(:, 1);
    y_est = xy_robot_w(:, 2);
    theta_est = unwrap(data_est(:, 4)) + theta_gt(1);

    vx_est = diff(x_est) ./ diff(times_est);
    vy_est = diff(y_est) ./ diff(times_est);
    v_est = sqrt(vx_est.^2 + vy_est.^2)';
    idx_est = find(v_est > 0.005, 1);
    t_start_est = times_est(idx_est) - 4;
    idx_est_start = find(times_est > t_start_est, 1);
    x_est = x_est(idx_est_start:end);
    y_est = y_est(idx_est_start:end);
    theta_est = theta_est(idx_est_start:end);
    times_est = times_est(idx_est_start:end) - t_start_est;

    error_function = @(dx) norm(interp1(times_gt, x_gt, times_est + dx, 'linear', 'extrap') - x_est);
    optimal_dx = fminsearch(error_function, 0);
    % vx_gt = diff(x_gt) ./ diff(times_gt)';
    % vy_gt = diff(y_gt) ./ diff(times_gt)';
    % v_gt = sqrt(vx_gt.^2 + vy_gt.^2)';
    % idx_gt = find(v_gt > 0.04, 1);
    t_start_gt = optimal_dx;
    idx_gt_start = find(times_gt > t_start_gt, 1);
    x_gt = x_gt(idx_gt_start:end);
    y_gt = y_gt(idx_gt_start:end);
    theta_gt = theta_gt(idx_gt_start:end);
    times_gt = times_gt(idx_gt_start:end) - t_start_gt;

    t = tiledlayout(T, 1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
    t.Layout.Tile = i;
    t.Title.String = sprintf('Robot %d', i);
    t.Title.FontWeight = 'bold';
    t.Title.FontSize = 12;

    ax1 = nexttile(t, 1);
    plot(times_gt, x_gt, 'k--', 'LineWidth', 1);
    grid on;
    hold on;
    plot(times_est, x_est, 'r', 'LineWidth', 1);
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('x [m]');
    set(ax1, 'FontSize', 12, 'FontWeight', 'bold');

    ax2 = nexttile(t, 2);
    plot(times_gt, y_gt, 'k--', 'LineWidth', 1);
    grid on;
    hold on;
    plot(times_est, y_est, 'g', 'LineWidth', 1);
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('y [m]');
    set(ax2, 'FontSize', 12, 'FontWeight', 'bold');

    ax3 = nexttile(t, 3);
    plot(times_gt, theta_gt, 'k--', 'LineWidth', 1);
    grid on;
    hold on;
    plot(times_est, theta_est, 'b', 'LineWidth', 1);
    xlim([1 tf]);
    xlabel('time [s]');
    ylabel('\theta [rad]');
    set(ax3, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Print results
fprintf("Final position error robots:\n")
for robot = 1:n_robots
    error_final_pos = norm(ending_poses_gt(robot, 1:2) - ending_poses_est(robot, :));
    fprintf(" - Robot %d: %.2f m\n", robot, error_final_pos)
end
fprintf("\n") 

fprintf("Landmarks errors after RoMa:\n")
for robot = 1:n_robots
    xy_lnds = [est_lnds_data{robot}.x(end, :)', est_lnds_data{robot}.y(end, :)'];
    [R, t, inl] = ransacRototranslation(xy_lnds, pos_anchors, 1000, 0.3, 7);
    xy_lnds_post_roma = (R * xy_lnds' + t)';
    error_post_roma = mean(vecnorm(xy_lnds_post_roma - pos_anchors, 2, 2));
    fprintf(" - Robot %d: %.2f m\n", robot, error_post_roma)
end
fprintf("\n")

fprintf("Final distances robot-landmark:\n")
for robot = 1:n_robots
    fprintf("\tRobot %d:\n", robot)
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
    for lnd = 1:n_anchors
        fprintf("%.2f ", distances_gt(lnd))
    end
    fprintf("\n")
    fprintf("\t\tEST:  ")
    for lnd = 1:n_anchors
        fprintf("%.2f ", distances_est(lnd))
    end
    fprintf("\n")
    fprintf("\t\t----------------------------------------------\n")
    fprintf("\t\tDIFF: ")
    for lnd = 1:n_anchors
        fprintf("%.2f ", abs(distances_gt(lnd) - distances_est(lnd)))
    end
    fprintf(" --> MEAN: %.2f ", mean(abs(distances_gt - distances_est)))
    fprintf("\n")
end
fprintf("\n")

fprintf("Final distances landmark-landmark:\n")
for robot = 1:n_robots
    fprintf("\tRobot %d:\n", robot)
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
    for lnd = 1:length(distances_gt)
        fprintf("%.2f ", distances_gt(lnd))
    end
    fprintf("\n")
    fprintf("\t\tEST:  ")
    for lnd = 1:length(distances_est)
        fprintf("%.2f ", distances_est(lnd))
    end
    fprintf("\n")
    fprintf("\t\t-------------------------------------------------------------------------------------------------------------------------------------------------\n")
    fprintf("\t\tDIFF: ")
    for lnd = 1:length(distances_est)
        fprintf("%.2f ", abs(distances_gt(lnd) - distances_est(lnd)))
    end
    fprintf("\n\t\t      --> MEAN: %.4f ", mean(abs(distances_gt - distances_est)))
    fprintf("\n")
end

%% Save workspace
% roslam_data = struct();
% 
% roslam_data.wheel_radius = wheel_radius;
% roslam_data.wheels_separation = wheels_separation;
% 
% roslam_data.duration = t_end-t_start;
% roslam_data.num_robots = n_robots;
% roslam_data.num_anchors_fixed = n_anchors;
% roslam_data.anchors_positions = pos_anchors;
% 
% roslam_data.ground_truth = gt_data;
% roslam_data.robot_odometry = est_lnds_data;
% % roslam_data.wheels_odometry = js_data;
% roslam_data.uwb_anchors_distances = uwb_anchors_data;
% 
% save('roslam_data.mat', 'roslam_data');

save(['./data/postprocess_', bag_name, '.mat'])