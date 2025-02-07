classdef RobotLocalizer < handle
    properties
        area_threshold
        forward_skip
        hsv_min1
        hsv_min2
        hsv_max1
        hsv_max2
        n_robots
        poses_frame
        rect
        rect_size
        use_debug
    end
    
    methods
        function obj = RobotLocalizer(n_robots, forward_skip, area_threshold, hsv_min1, hsv_min2, hsv_max1, hsv_max2, rect_size, use_debug)
            % Constructor to initialize parameters
            obj.area_threshold = area_threshold;
            obj.forward_skip = forward_skip;
            obj.hsv_min1 = hsv_min1;
            obj.hsv_min2 = hsv_min2;
            obj.hsv_max1 = hsv_max1;
            obj.hsv_max2 = hsv_max2;
            obj.n_robots = n_robots;
            obj.rect_size = rect_size;
            obj.use_debug = use_debug;

            obj.poses_frame = zeros(n_robots, 3);
        end
        
        function [poses_frame, rects] = localize(obj, frame, rects)
            [height, width, ~] = size(frame);
            poses_frame = obj.poses_frame;
            
            for robot = 1:obj.n_robots
                [x1, y1, x2, y2] = obj.clamp_coordinates(int32(rects(robot, :)), width, height);

                patch = frame(y1:y2, x1:x2, :);
                mask = obj.create_color_mask(patch);
                
                if obj.use_debug
                    obj.debug_display(patch, mask);
                end
                
                stats = regionprops(mask, 'BoundingBox', 'Area', 'Centroid');
                if isempty(stats)
                    angle = poses_frame(robot, 3);
                    robot_center = poses_frame(robot, 1:2) + obj.forward_skip * [cos(angle), sin(angle)];
                    [robot_center(1), robot_center(2)] = obj.clamp_coordinates_point(robot_center, width, height);
                    rects(robot, :) = [robot_center - obj.rect_size, robot_center + obj.rect_size];
                    continue
                end
                
                [centroid_front, centroid_back, valid] = obj.find_centroids(stats);
                if ~valid
                    disp("Error");
                    continue
                end
                
                angle = atan2(centroid_front(2) - centroid_back(2), ...
                              centroid_front(1) - centroid_back(1));
                
                poses_frame(robot, :) = [centroid_front + rects(robot, 1:2), angle];
                poses_frame(robot, 1) = max(1, min(poses_frame(robot, 1), width));
                poses_frame(robot, 2) = max(1, min(poses_frame(robot, 2), height));
                rects(robot, :) = obj.update_bounding_box(poses_frame(robot, :), width, height);
                
                if obj.use_debug
                    obj.debug_visualize(patch, centroid_front, centroid_back, robot);
                end
            end

            obj.poses_frame = poses_frame;
        end

        function [x, y] = clamp_coordinates_point(~, point, width, height)
            x = max(1, min(point(1), width));
            y = max(1, min(point(2), height));
        end
    
        function [x1, y1, x2, y2] = clamp_coordinates(obj, rect, width, height)
            [x1, y1] = obj.clamp_coordinates_point(rect(1:2), width, height);
            [x2, y2] = obj.clamp_coordinates_point(rect(3:4), width, height);
        end
        
        function mask = create_color_mask(obj, patch)
            hsv_patch = rgb2hsv(patch);
            mask1 = (hsv_patch(:,:,1) >= obj.hsv_min1(1)) & (hsv_patch(:,:,1) <= obj.hsv_max1(1)) & ...
                    (hsv_patch(:,:,2) >= obj.hsv_min1(2)) & (hsv_patch(:,:,2) <= obj.hsv_max1(2)) & ...
                    (hsv_patch(:,:,3) >= obj.hsv_min1(3)) & (hsv_patch(:,:,3) <= obj.hsv_max1(3));
            mask2 = (hsv_patch(:,:,1) >= obj.hsv_min2(1)) & (hsv_patch(:,:,1) <= obj.hsv_max2(1)) & ...
                    (hsv_patch(:,:,2) >= obj.hsv_min2(2)) & (hsv_patch(:,:,2) <= obj.hsv_max2(2)) & ...
                    (hsv_patch(:,:,3) >= obj.hsv_min2(3)) & (hsv_patch(:,:,3) <= obj.hsv_max2(3));
            mask = mask1 | mask2;
        end
        
        function [front, back, valid] = find_centroids(obj, stats)
            valid = false;
            centroids = [];
            areas = [];
            for j = 1:length(stats)
                if stats(j).Area > obj.area_threshold
                    centroids = [centroids; stats(j).Centroid];
                    areas = [areas; stats(j).Area, stats(j).Centroid];
                end
            end
            
            area_size = size(areas, 1);
            if area_size < 3 || area_size > 5
                front = [];
                back = [];
                return;
            end
            
            valid = true;
            areas = sortrows(areas, 1);
            if area_size == 4
                back = [(areas(3,2) + areas(4,2)) / 2, (areas(3,3) + areas(4,3)) / 2];
            else
                back = [areas(3,2), areas(3,3)];
            end
            front = [(areas(1,2) + areas(2,2)) / 2, (areas(1,3) + areas(2,3)) / 2];
        end
        
        function rect = update_bounding_box(obj, pose, width, height)
            rect = [max(1, min(pose(1) - obj.rect_size / 2, width)), ...
                    max(1, min(pose(2) - obj.rect_size / 2, height)), ...
                    max(1, min(pose(1) + obj.rect_size / 2, width)), ...
                    max(1, min(pose(2) + obj.rect_size / 2, height))];
        end
        
        function debug_display(~, patch, mask)
            figure(201); imshow(patch);
            figure(202); imshow(mask);
        end
        
        function debug_visualize(~, patch, front, back, robot)
            figure(203);
            nexttile;
            imshow(patch);
            hold on;
            plot(front(1), front(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
            plot(back(1), back(2), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
            line([front(1), back(1)], [front(2), back(2)], 'Color', 'b', 'LineWidth', 2);
            axis equal;
            title(sprintf('Robot %d', robot));
            axis off;
        end
    end
end
