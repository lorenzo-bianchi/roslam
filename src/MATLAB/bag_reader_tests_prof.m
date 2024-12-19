%% Cleanup
clc; clear; close all

%% Params
wheel_radius = 0.033;
wheels_separation = 0.16;

keys = ["44AE", "08B0", "8192", "5723", ...         % 2 and 4 are swapped
        "139C", "9128", "4A21"];
values = ["A1", "A2", "A3", "A4", ...
          "R2-M2-A2", "R3-M2-A3", "R3-M3-A3"];

uwb_fixed_names = dictionary(keys(1:4), values(1:4));
uwb_moving_names = dictionary(keys(5:end), values(5:end));

pos_anchors = [    0.0    0.0;
                1.7903    0.0;
                1.7241 3.6934;
               -0.1471 3.7211];

%% Read bag and topics names
bag_name = '20241218_test7';
path_prefix = '/home/lorenzo/Github/turtlebot3-utv-fork/logs/';
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

%% Get number of tags and positions
inter_robots_distances = false;
n_anchors = -1;
for i = 1:length(all_topics)
    topic = all_topics(i);
    if contains(topic, 'uwb')
        uwb_topic = select(bag, 'Topic', topic);
        uwb_msgs = readMessages(uwb_topic);
        for k = 1:length(uwb_msgs)
            % id_strs = string({uwb_msgs{k}.uwbs.id_str});
            if isKey(uwb_moving_names, uwb_msgs{k}.uwbs(1).id_str)
                inter_robots_distances = true;
            else
                num = uwb_msgs{k}.anchor_num;
                if num > n_anchors
                    n_anchors = num;
                end
            end
        end
        break
    end
end

%% Summary
fprintf('Duration: %.2f s\n', t_end-t_start);
fprintf('Found: %d robots, %d anchors\n', n_robots, n_anchors);
if inter_robots_distances
    fprintf('Inter-robots distances AVAILABLE\n');
else
    fprintf('Inter-robots distances NOT AVAILABLE\n');
end
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
% if any(contains(topics_names, 'ground_truth'))
%     gt_topics_names = all_topics(contains(all_topics, 'ground_truth'));
%     gt_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
%     for i = 1:length(gt_topics_names)
%         gt_topic = select(bag, 'Topic', gt_topics_names(i));
%         gt_msg = readMessages(gt_topic);
%         gt_times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, gt_msg);
%         gt_times = gt_times - gt_times(1);
%         gt_pos_x = cellfun(@(m) double(m.pose.pose.position.x), gt_msg);
%         gt_pos_y = cellfun(@(m) double(m.pose.pose.position.y), gt_msg);
% 
%         quat_w = cellfun(@(m) m.pose.pose.orientation.w, gt_msg);
%         quat_x = cellfun(@(m) m.pose.pose.orientation.x, gt_msg);
%         quat_y = cellfun(@(m) m.pose.pose.orientation.y, gt_msg);
%         quat_z = cellfun(@(m) m.pose.pose.orientation.z, gt_msg);
%         rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
%         gt_data(i).theta = rpy(:, 1);
% 
%         gt_data(i).times = gt_times;
%         gt_data(i).x = gt_pos_x;
%         gt_data(i).y = gt_pos_y;
%         gt_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, gt_msg);
%         gt_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, gt_msg);
%     end
% end

%% UWB
if any(contains(topics_names, 'uwb_tag'))
    uwb_topics_names = all_topics(contains(all_topics, 'uwb_tag'));
    uwb_anchors_data = cell(length(uwb_topics_names), 1);
    uwb_robots_data = cell(length(uwb_topics_names), 1);
    for i = 1:length(uwb_topics_names)
        % robot i
        uwb_topic = select(bag, 'Topic', uwb_topics_names(i));
        num_msgs = uwb_topic.NumMessages;
        uwb_msg = readMessages(uwb_topic);
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, uwb_msg);
        times = times - t_start;

        for j = 1:num_msgs
            % message at time j
            if isKey(uwb_fixed_names, uwb_msg{j}.uwbs(1).id_str)
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
            else
                uwb_dist_inter = nan*ones(1, n_robots);
                uwb_dist_inter(i) = 0.0;
                for k = 1:size(uwb_msg{j}.uwbs, 1)
                    id_str = uwb_msg{j}.uwbs(k).id_str;
                    other_robot_num = str2double(extractBetween(uwb_moving_names(id_str), 2, 2));
                    single_dist = uwb_msg{j}.uwbs(k).dist;
                    if single_dist < 0.1
                        single_dist = nan;
                    end
                    uwb_dist_inter(other_robot_num) = single_dist;
                end
                uwb_robots_data{i}(end+1, :) = [times(j), uwb_dist_inter];
            end
        end
    end
end

% Plot anchors distances
figure;
tiledlayout(length(uwb_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1200, 800]);

t0 = 1;
for i = 1:length(uwb_topics_names)
    data = uwb_anchors_data{i};

    times = data(:, 1);
    distances = data(:, 2:end);

    nexttile;
    plot(times, distances, 'LineWidth', 2);
    grid on;
    hold on;

    for j = 1:size(distances, 2)
        nan_indices = isnan(distances(:, j));

        valid_indices = ~nan_indices;
        distances(:, j) = interp1(times(valid_indices), distances(valid_indices, j), times, 'linear', 'extrap');

        plot(times(nan_indices), distances(nan_indices, j), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
        hold on;
    end

    xlim([t0 inf]);

    title(['Robot ', num2str(i)]);
    xlabel('time [s]');
    ylabel('distance [m]');
    legend('Anchor 1', 'Anchor 2', 'Anchor 3', 'Anchor 4', 'Location', 'southwest');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

% Plot robots distances
figure;
set(gcf, 'Position', [100, 100, 600, 400]);

data = uwb_robots_data{i};

times_12_13 = uwb_robots_data{1}(:, 1);
distances_12 = uwb_robots_data{1}(:, 3);
distances_13 = uwb_robots_data{1}(:, 4);

times_23 = uwb_robots_data{2}(:, 1);
distances_23 = uwb_robots_data{2}(:, 4);

%
plot(times_12_13, distances_12, 'LineWidth', 2);
hold on

nan_indices = isnan(distances_12);
valid_indices = ~nan_indices;
distances_12 = interp1(times_12_13(valid_indices), distances_12(valid_indices), times_12_13, 'linear', 'extrap');

plot(times_12_13(nan_indices), distances_12(nan_indices), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

%
plot(times_12_13, distances_13, 'LineWidth', 2);
hold on

nan_indices = isnan(distances_13);
valid_indices = ~nan_indices;
distances_13 = interp1(times_12_13(valid_indices), distances_13(valid_indices), times_12_13, 'linear', 'extrap');

plot(times_12_13(nan_indices), distances_13(nan_indices), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

%
plot(times_23, distances_23, 'LineWidth', 2);
hold on

nan_indices = isnan(distances_23);
valid_indices = ~nan_indices;
distances_23 = interp1(times_23(valid_indices), distances_23(valid_indices), times_23, 'linear', 'extrap');

plot(times_23(nan_indices), distances_23(nan_indices), 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');


grid on
xlim([t0 inf]);
xlabel('time [s]');
ylabel('distance [m]');
legend('d_{12}', 'd_{13}', 'd_{23}', 'Location', 'northeast');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');


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

        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, js_msg);
        times = times - t_start;

        pos_left = cellfun(@(m) m.position(idx_left), js_msg);
        pos_left = [0; diff(pos_left) * wheel_radius];

        pos_right = cellfun(@(m) m.position(idx_right), js_msg);
        pos_right = [0; diff(pos_right) * wheel_radius];

        js_data{i} = [times, pos_right, pos_left];
    end
end

% Plot wheels velocities
figure;
tiledlayout(length(js_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1200, 800]);

t0 = 1;
for i = 1:length(js_topics_names)
    data = js_data{i};

    times = data(:, 1);
    velocities = data(:, 2:end);

    nexttile;
    plot(times, velocities, 'LineWidth', 2);
    grid on;
    hold on;

    xlim([t0 inf]);

    title(['Robot ', num2str(i)]);
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    legend('Right wheel', 'Left wheel', 'Location', 'northwest');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Odom
if any(contains(topics_names, 'odom'))
    odom_topics_names = all_topics(contains(all_topics, 'odom'));
    odom_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    odom_data_cell = cell(length(odom_topics_names), 1);
    for i = 1:length(odom_topics_names)
        odom_topic = select(bag, 'Topic', odom_topics_names(i));
        odom_msg = readMessages(odom_topic);
        
        odom_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, odom_msg);
        odom_data(i).times = odom_data(i).times - t_start;
        odom_data(i).x = cellfun(@(m) m.pose.pose.position.x, odom_msg);
        odom_data(i).y = cellfun(@(m) m.pose.pose.position.y, odom_msg);
        odom_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, odom_msg);
        odom_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, odom_msg);

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, odom_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, odom_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, odom_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, odom_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        odom_data(i).theta = rpy(:, 1);

        odom_data_cell{i} = [odom_data(i).times(:), odom_data(i).x(:), odom_data(i).y(:), odom_data(i).theta(:), odom_data(i).v_lin(:), odom_data(i).v_ang(:)];
    end
end

% Plot robot velocities
figure;
tiledlayout(length(odom_topics_names), 1, 'TileSpacing', 'Compact', 'Padding', 'Compact');
set(gcf, 'Position', [100, 100, 1200, 800]);

t0 = 1;
for i = 1:length(odom_topics_names)
    data = odom_data_cell{i};

    times = data(:, 1);
    velocities = data(:, end-1:end);

    nexttile;
    plot(times, velocities, 'LineWidth', 2);
    grid on;
    hold on;

    xlim([t0 inf]);

    title(['Robot ', num2str(i)]);
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    legend('Linear', 'Angular', 'Location', 'northwest');
    set(gca, 'FontSize', 12, 'FontWeight', 'bold');
end

%% Save workspace
roslam_data = struct();

roslam_data.wheel_radius = wheel_radius;
roslam_data.wheels_separation = wheels_separation;

roslam_data.duration = t_end-t_start;
roslam_data.num_robots = n_robots;
roslam_data.num_anchors_fixed = n_anchors;
roslam_data.inter_robots_distances = inter_robots_distances;
roslam_data.anchors_positions = pos_anchors;

% roslam_data.ground_truth = gt_data;
roslam_data.robot_odometry = odom_data;
roslam_data.wheels_odometry = js_data;
roslam_data.uwb_anchors_distances = uwb_anchors_data;
roslam_data.uwb_inter_robots_distances = uwb_robots_data;

save('roslam_data.mat', 'roslam_data');
