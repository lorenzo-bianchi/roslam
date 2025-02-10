classdef AnchorLocalizer < handle
    properties
        area_threshold
        hsv_min1
        hsv_min2
        hav_max1
        hsv_max2
        n_anchors
        poses_frame
        rect
        rect_size
        use_debug
    end

    methods
        function obj = AnchorLocalizer(n_anchors, area_threshold, hsv_min1, hsv_min2, hsv_max1, hsv_max2, rect_size, use_debug)
            % Constructor to initialize parameters
            obj.area_threshold = area_threshold;
            obj.hsv_min1 = hsv_min1;
            obj.hsv_min2 = hsv_min2;
            obj.hav_max1 = hsv_max1;
            obj.hsv_max2 = hsv_max2;
            obj.n_anchors = n_anchors;
            obj.rect_size = rect_size;
            obj.use_debug = use_debug;

            obj.poses_frame = zeros(n_anchors, 2);
        end

        function [poses_frame, rects] = localize(obj, frame, rects)
            [height, width, ~] = size(frame);
            poses_frame = obj.poses_frame;

            for anchor = 1:obj.n_anchors
                [x1, y1, x2, y2] = obj.clamp_coordinates(int32(rects(anchor, :)), width, height);

                patch = frame(y1:y2, x1:x2, :);
                mask = obj.create_color_mask(patch);

                stats = regionprops(mask, 'BoundingBox', 'Area', 'Centroid');
                if isempty(stats)
                    rects(anchor, :) = [poses_frame(anchor, 1:2) - 2 * obj.rect_size, ...
                                        poses_frame(anchor, 1:2) + 2 * obj.rect_size];
                    continue
                end

                for j = 1:length(stats)
                    if stats(j).Area > obj.area_threshold
                        poses_frame(anchor, :) = stats(j).Centroid;
                        poses_frame(anchor, :) = poses_frame(anchor, :) + rects(anchor, 1:2);
                        poses_frame(anchor, 1) = max(1, min(poses_frame(anchor, 1), width));
                        poses_frame(anchor, 2) = max(1, min(poses_frame(anchor, 2), height));
                        rects(anchor, :) = obj.update_bounding_box(poses_frame(anchor, :), width, height);

                        break
                    end
                end

                if obj.use_debug
                    figure(101);
                    nexttile;
                    imshow(patch);
                    hold on;
                    rectangle('Position', stats(j).BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
                    plot(poses_frame(anchor, 1), poses_frame(anchor, 2), 'ro', 'MarkerFaceColor', 'r', 'LineWidth', 1);
                    title(sprintf('Anchor %d', anchor));
                    axis off;
                end
            end

           obj.poses_frame = poses_frame;
        end

        function [x1, y1, x2, y2] = clamp_coordinates(~, rect, width, height)
            x1 = max(1, min(rect(1), width));
            y1 = max(1, min(rect(2), height));
            x2 = max(1, min(rect(3), width));
            y2 = max(1, min(rect(4), height));
        end

        function mask = create_color_mask(obj, patch)
            hsv_patch = rgb2hsv(patch);
            mask1 = (hsv_patch(:,:,1) >= obj.hsv_min1(1)) & (hsv_patch(:,:,1) <= obj.hav_max1(1)) & ...
                    (hsv_patch(:,:,2) >= obj.hsv_min1(2)) & (hsv_patch(:,:,2) <= obj.hav_max1(2)) & ...
                    (hsv_patch(:,:,3) >= obj.hsv_min1(3)) & (hsv_patch(:,:,3) <= obj.hav_max1(3));
            mask2 = (hsv_patch(:,:,1) >= obj.hsv_min2(1)) & (hsv_patch(:,:,1) <= obj.hsv_max2(1)) & ...
                    (hsv_patch(:,:,2) >= obj.hsv_min2(2)) & (hsv_patch(:,:,2) <= obj.hsv_max2(2)) & ...
                    (hsv_patch(:,:,3) >= obj.hsv_min2(3)) & (hsv_patch(:,:,3) <= obj.hsv_max2(3));
            mask = mask1 | mask2;
        end

        function rect = update_bounding_box(obj, pose, width, height)
            rect = [max(1, min(pose(1) - obj.rect_size / 2, width)), ...
                    max(1, min(pose(2) - obj.rect_size / 2, height)), ...
                    max(1, min(pose(1) + obj.rect_size / 2, width)), ...
                    max(1, min(pose(2) + obj.rect_size / 2, height))];
        end
    end
end
