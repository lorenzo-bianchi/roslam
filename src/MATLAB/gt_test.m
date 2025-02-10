q%%
clear; close all; clc;

%%
n_anchors = 4;
n_robots = 3;
video_name = 'test5';               % test1 no / test2 yes / test3 no / test4 yes / test5 fix / test6 ? / test7 yes / test8 fix?
min_time = 2;
max_time = 86; % in seconds % 86
pause_time = 0.05;
skip_n_frames = 20;

area_threshold = 50;
robots_rect_size = 200;
anchors_rect_size = 300;
len_line = 100;

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

%%
video_path = ['./videos/', video_name, '.MP4'];
video = VideoReader(video_path);
video.CurrentTime = min_time;
video_height = video.Height;
video_width = video.Width;

anchors_min1 = [anchors_h_min1, anchors_s_min, anchors_v_min] ./ [180, 255, 255];
anchors_max1 = [anchors_h_max1, anchors_s_max, anchors_v_max] ./ [180, 255, 255];
anchors_min2 = [anchors_h_min2, anchors_s_min, anchors_v_min] ./ [180, 255, 255];
anchors_max2 = [anchors_h_max2, anchors_s_max, anchors_v_max] ./ [180, 255, 255];

robots_min1 = [robots_h_min1, robots_s_min, robots_v_min] ./ [180, 255, 255];
robots_max1 = [robots_h_max1, robots_s_max, robots_v_max] ./ [180, 255, 255];
robots_min2 = [robots_h_min2, robots_s_min, robots_v_min] ./ [180, 255, 255];
robots_max2 = [robots_h_max2, robots_s_max, robots_v_max] ./ [180, 255, 255];

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
end

%% Main loop
total__frames = floor(video.Duration * video.FrameRate);

anchors_poses_frame = zeros(n_anchors, 2);
robots_poses_frame = zeros(n_robots, 3);

n_frame = -1;
figure
while hasFrame(video) && video.CurrentTime <= max_time
    frame = readFrame(video);
    n_frame = n_frame + 1;

    if n_frame == 80
        a = 1;
    end

    % Anchors
    [anchors_poses_frame, anchors_rects] = localize_anchors(frame, anchors_poses_frame, anchors_rects, area_threshold, anchors_min1, anchors_min2, anchors_max1, anchors_max2, anchors_rect_size, false);

    % Robots
    [robots_poses_frame, robots_rects] = localize_robots(frame, robots_poses_frame, robots_rects, area_threshold, robots_min1, robots_min2, robots_max1, robots_max2, robots_rect_size, false);

    %% Plots
    % Anchors
    rects_positions = [anchors_rects(:, 1), anchors_rects(:, 2), ...
                       anchors_rects(:, 3) - anchors_rects(:, 1), ...
                       anchors_rects(:, 4) - anchors_rects(:, 2)];
    frame = insertShape(frame, 'Rectangle', rects_positions, 'LineWidth', 10, 'Color', 'yellow', 'Opacity', 1);

    centers = [anchors_poses_frame(:, 1), anchors_poses_frame(:, 2), 15*ones(n_anchors, 1)];
    frame = insertShape(frame, 'FilledCircle', centers, 'Color', 'yellow', 'Opacity', 1);

    % Robots
    rects_positions = [robots_rects(:, 1), robots_rects(:, 2), ...
                       robots_rects(:, 3) - robots_rects(:, 1), ...
                       robots_rects(:, 4) - robots_rects(:, 2)];
    frame = insertShape(frame, 'Rectangle', rects_positions, 'LineWidth', 10, 'Color', 'green', 'Opacity', 1);

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

%%
%anchors_poses_frame = get_anchors_poses(frame, anchors_rects, area_threshold, anchors_min1, anchors_min2, anchors_max1, anchors_max2, true);
%robots_poses_frame = get_robots_poses(frame, robots_rects, area_threshold, robots_min1, robots_min2, robots_max1, robots_max2, true);