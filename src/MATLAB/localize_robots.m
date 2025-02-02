function [robots_poses_frame, robots_rects] = localize_robots(frame, robots_poses_frame, robots_rects, ...
        area_threshold, robots_min1, robots_min2, robots_max1, robots_max2, rect_size, debug)
    n_robots = size(robots_rects, 1);
    [height, width, ~] = size(frame);

    for robot = 1:n_robots
        rect = int32(robots_rects(robot, :));
        x1 = max(1, min(rect(1), width));
        y1 = max(1, min(rect(2), height));
        x2 = max(1, min(rect(3), width));
        y2 = max(1, min(rect(4), height));

        patch = frame(y1:y2, x1:x2, :);
        hsv_patch = rgb2hsv(patch);
    
        % Create masks for red color
        mask1 = (hsv_patch(:,:,1) >= robots_min1(1)) & (hsv_patch(:,:,1) <= robots_max1(1)) & ...
                (hsv_patch(:,:,2) >= robots_min1(2)) & (hsv_patch(:,:,2) <= robots_max1(2)) & ...
                (hsv_patch(:,:,3) >= robots_min1(3)) & (hsv_patch(:,:,3) <= robots_max1(3));
        mask2 = (hsv_patch(:,:,1) >= robots_min2(1)) & (hsv_patch(:,:,1) <= robots_max2(1)) & ...
                (hsv_patch(:,:,2) >= robots_min2(2)) & (hsv_patch(:,:,2) <= robots_max2(2)) & ...
                (hsv_patch(:,:,3) >= robots_min2(3)) & (hsv_patch(:,:,3) <= robots_max2(3));
        mask = mask1 | mask2;

        if debug
            figure(201);
            imshow(patch);
            figure(202);
            imshow(mask);
        end
    
        % Find contours
        stats = regionprops(mask, 'BoundingBox', 'Area', 'Centroid');
        if size(stats, 1) == 0
            %robots_poses_frame(robot, :) = robots_poses_frame(robot, :) + robots_rects(robot, 1:2);
            robots_rects(robot, :) = [robots_poses_frame(robot, 1:2) - rect_size, ...
                                      robots_poses_frame(robot, 1:2) + rect_size];
            continue
        end
    
        % Iterate through the detected contours
        robot_centroids = [];
        areas = [];
        for j = 1:length(stats)
            if stats(j).Area > area_threshold            
                robot_centroids = [robot_centroids; stats(j).Centroid];
                areas = [areas; stats(j).Area, stats(j).Centroid];
            end
        end

        area_size = size(areas, 1);
        if area_size < 3 || area_size > 5 
            disp("Error")
            continue
        end

        areas = sortrows(areas, 1);
        if area_size == 4    
            robot_centroid_back  = [(areas(3,2) + areas(4,2)) / 2, (areas(3,3) + areas(4,3)) / 2];
        elseif area_size  == 3
            robot_centroid_back = [areas(3,2), areas(3,3)];
        else
            disp("Error")
            continue
        end
        robot_centroid_front = [(areas(1,2) + areas(2,2)) / 2, (areas(1,3) + areas(2,3)) / 2];
        angle = atan2(robot_centroid_front(2) - robot_centroid_back(2), robot_centroid_front(1) - robot_centroid_back(1));

        robots_poses_frame(robot, :) = [robot_centroid_front + robots_rects(robot, 1:2), angle];
        robots_poses_frame(robot, 1) = max(1, min(robots_poses_frame(robot, 1), width));
        robots_poses_frame(robot, 2) = max(1, min(robots_poses_frame(robot, 2), height));
        robots_rects(robot, :) = [max(1, min(robots_poses_frame(robot, 1) - rect_size / 2, width)), ...
                                  max(1, min(robots_poses_frame(robot, 2) - rect_size / 2, height)), ...
                                  max(1, min(robots_poses_frame(robot, 1) + rect_size / 2, width)), ...
                                  max(1, min(robots_poses_frame(robot, 2) + rect_size / 2, height))];

        if debug
            figure(203);
            nexttile;
            imshow(patch);
            hold on;
            plot(robot_centroid_front(1), robot_centroid_front(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
            plot(robot_centroid_back(1), robot_centroid_back(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
            line([robot_centroid_front(1), robot_centroid_back(1)], [robot_centroid_front(2), robot_centroid_back(2)], 'Color', 'b', 'LineWidth', 2);
            axis equal
            title(sprintf('Robot %d', robot));
            axis off;
        end
    end
end