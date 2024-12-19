%% Cleanup
clc; clear; close all

%% Params
wheel_radius = 0.033;
wheels_separation = 0.16;

keys = ["44AE", "5723", "8192", "08B0", ...
        "4A21", "860B", "52B0", "5107", ...
        "139C", "54B3", "9128", "4A21"];
values = ["A1", "A2", "A3", "A4", ...
          "M1-T1", "M2-T1", "M1-T2", "M3-T2", ...
          "M2-A2", "M1-T3", "M2-A3", "M3-A3"];

uwb_fixed_names = dictionary(keys(1:4), values(1:4));
uwb_moving_names = dictionary(keys(5:end), values(5:end));

pos_anchors = [    0.0    0.0;
                1.7903    0.0;
                1.7241 3.6934;
               -0.1471 3.7211];

%% Read bag and topics names
bag_name = '20241218_test1';
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
for i = 1:length(all_topics)
    topic = all_topics(i);
    if contains(topic, 'uwb')
        uwb_topic = select(bag, 'Topic', topic);
        uwb_msgs = readMessages(uwb_topic);
        n_anchors_tot = -1;
        inter_robots_distances = false;
        for k = 1:length(uwb_msgs)
            id_strs = string({uwb_msgs{k}.uwbs.id_str});
            if isKey(uwb_moving_names, id_strs(1))
                inter_robots_distances = true;
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
fprintf('Available topicjs_msg = readMessages(js_topic)s:\n');
for name = topics_names
    fprintf('\t- %s\n', name);
end
fprintf('Anchors in positions:\n');
for i = 1:length(pos_anchors)
    fprintf('\t- (%d, %d)\n', pos_anchors(i,1), pos_anchors(i,2));
end

%% Topics analysis
% Ground truth 
if any(contains(topics_names, 'ground_truth'))
    gt_topics_names = all_topics(contains(all_topics, 'ground_truth'));
    gt_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    for i = 1:length(gt_topics_names)
        gt_topic = select(bag, 'Topic', gt_topics_names(i));
        gt_msg = readMessages(gt_topic);
        gt_times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, gt_msg);
        gt_times = gt_times - gt_times(1);
        gt_pos_x = cellfun(@(m) double(m.pose.pose.position.x), gt_msg);
        gt_pos_y = cellfun(@(m) double(m.pose.pose.position.y), gt_msg);

        quat_w = cellfun(@(m) m.pose.pose.orientation.w, gt_msg);
        quat_x = cellfun(@(m) m.pose.pose.orientation.x, gt_msg);
        quat_y = cellfun(@(m) m.pose.pose.orientation.y, gt_msg);
        quat_z = cellfun(@(m) m.pose.pose.orientation.z, gt_msg);
        rpy = quat2eul([quat_w, quat_x, quat_y, quat_z], 'ZYX');
        gt_data(i).theta = rpy(:, 1);

        gt_data(i).times = gt_times;
        gt_data(i).x = gt_pos_x;
        gt_data(i).y = gt_pos_y;
        gt_data(i).v_lin = cellfun(@(m) m.twist.twist.linear.x, gt_msg);
        gt_data(i).v_ang = cellfun(@(m) m.twist.twist.angular.z, gt_msg);
    end
end

%% UWB
if any(contains(topics_names, 'uwb_tag'))
    uwb_topics_names = all_topics(contains(all_topics, 'uwb_tag'));
    uwb_anchors_data = cell(length(uwb_topics_names), 1);
    uwb_robots_data = cell(length(uwb_topics_names), 1);
    for i = 1:length(uwb_topics_names)
        uwb_topic = select(bag, 'Topic', uwb_topics_names(i));
        num_msgs = uwb_topic.NumMessages;
        uwb_msg = readMessages(uwb_topic);
        times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, uwb_msg);
        times = times - times(1);
        uwb_dist = nan*ones(num_msgs, n_anchors);
        uwb_dist_inter = nan*ones(num_msgs, n_anchors_tot-n_anchors+1);
        for j = 1:num_msgs
            uwb_dist_inter(j, i) = 0.0;
            for k = 1:size(uwb_msg{j}.uwbs, 1)
                id = uwb_msg{j}.uwbs(k).id;
                if id > 100
                    uwb_dist_inter(j, id-100) = uwb_msg{j}.uwbs(k).dist;
                    continue
                end
                uwb_dist(j, id+1) = uwb_msg{j}.uwbs(k).dist;
            end
        end
        uwb_anchors_data{i} = [times, uwb_dist];
        uwb_robots_data{i} = [times, uwb_dist_inter];
    end
end

%% Joint states
if any(contains(topics_names, 'joint_states'))
    js_topics_names = all_topics(contains(all_topics, 'joint_states'));
    js_data = cell(length(js_topics_names), 1);
    for i = 1:length(js_topics_names)
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
        times = times - times(1);

        pos_left = cellfun(@(m) m.position(idx_left), js_msg);
        pos_left = [0; diff(pos_left) * wheel_radius];

        pos_right = cellfun(@(m) m.position(idx_right), js_msg);
        pos_right = [0; diff(pos_right) * wheel_radius];

        js_data{i} = [times, pos_right, pos_left];
    end
end

%% Odom
if any(contains(topics_names, 'odom'))
    odom_topics_names = all_topics(contains(all_topics, 'odom'));
    odom_data = struct('times', [], 'x', [], 'y', [], 'theta', [], 'v_lin', [], 'v_ang', []);
    for i = 1:length(odom_topics_names)
        odom_topic = select(bag, 'Topic', odom_topics_names(i));
        odom_msg = readMessages(odom_topic);
        
        odom_data(i).times = cellfun(@(m) double(m.header.stamp.sec) + double(m.header.stamp.nanosec)/1e9, odom_msg);
        odom_data(i).times = odom_data(i).times - odom_data(i).times(1);
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
    end
end

%% Save workspace
roslam_data = struct();

roslam_data.wheel_radius = wheel_radius;
roslam_data.wheels_separation = wheels_separation;

roslam_data.duration = t_end-t_start;
roslam_data.num_robots = n_robots;
roslam_data.num_anchors_fixed = n_anchors;
roslam_data.num_anchors_total = n_anchors_tot;
roslam_data.inter_robots_distances = inter_robots_distances;
roslam_data.anchors_positions = pos_anchors;

roslam_data.ground_truth = gt_data;
roslam_data.robot_odometry = odom_data;
roslam_data.wheels_odometry = js_data;
roslam_data.uwb_anchors_distances = uwb_anchors_data;
roslam_data.uwb_inter_robots_distances = uwb_robots_data;

save('roslam_data.mat', 'roslam_data');
