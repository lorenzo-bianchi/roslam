function [anchors_poses_frame, anchors_rects] = localize_anchors(frame, anchors_poses_frame, anchors_rects, ...
        area_threshold, anchors_min1, anchors_min2, anchors_max1, anchors_max2, rect_size, debug)
    n_anchors = size(anchors_rects, 1);
    [height, width, ~] = size(frame);

    for anchor = 1:n_anchors
        rect = int32(anchors_rects(anchor, :));
        x1 = max(1, min(rect(1), width));
        y1 = max(1, min(rect(2), height));
        x2 = max(1, min(rect(3), width));
        y2 = max(1, min(rect(4), height));

        patch = frame(y1:y2, x1:x2, :);
        hsv_patch = rgb2hsv(patch);

        % Create masks for red color
        mask1 = (hsv_patch(:,:,1) >= anchors_min1(1)) & (hsv_patch(:,:,1) <= anchors_max1(1)) & ...
                (hsv_patch(:,:,2) >= anchors_min1(2)) & (hsv_patch(:,:,2) <= anchors_max1(2)) & ...
                (hsv_patch(:,:,3) >= anchors_min1(3)) & (hsv_patch(:,:,3) <= anchors_max1(3));
        mask2 = (hsv_patch(:,:,1) >= anchors_min2(1)) & (hsv_patch(:,:,1) <= anchors_max2(1)) & ...
                (hsv_patch(:,:,2) >= anchors_min2(2)) & (hsv_patch(:,:,2) <= anchors_max2(2)) & ...
                (hsv_patch(:,:,3) >= anchors_min2(3)) & (hsv_patch(:,:,3) <= anchors_max2(3));
        mask = mask1 | mask2;

        % Find contours
        stats = regionprops(mask, 'BoundingBox', 'Area', 'Centroid');
        if size(stats, 1) == 0
            %anchors_poses_frame(anchor, :) = anchors_poses_frame(anchor, :) + anchors_rects(anchor, 1:2);
            anchors_rects(anchor, :) = [anchors_poses_frame(anchor, 1:2) - 2 * rect_size, ...
                                        anchors_poses_frame(anchor, 1:2) + 2 * rect_size];
            continue
        end

        % Iterate through the detected contours
        for j = 1:length(stats)
            if stats(j).Area > area_threshold
                anchors_poses_frame(anchor, :) = stats(j).Centroid;
                break
            end
        end

        % Get coordinates in image reference system
        anchors_poses_frame(anchor, :) = anchors_poses_frame(anchor, :) + anchors_rects(anchor, 1:2);
        anchors_poses_frame(anchor, 1) = max(1, min(anchors_poses_frame(anchor, 1), width));
        anchors_poses_frame(anchor, 2) = max(1, min(anchors_poses_frame(anchor, 2), height));
        anchors_rects(anchor, :) = [max(1, min(anchors_poses_frame(anchor, 1) - rect_size / 2, width)), ...
                                    max(1, min(anchors_poses_frame(anchor, 2) - rect_size / 2, height)), ...
                                    max(1, min(anchors_poses_frame(anchor, 1) + rect_size / 2, width)), ...
                                    max(1, min(anchors_poses_frame(anchor, 2) + rect_size / 2, height))];

        if debug
            figure(101);
            nexttile;
            imshow(patch);
            hold on;
            rectangle('Position', stats(j).BoundingBox, 'EdgeColor', 'g', 'LineWidth', 2);
            plot(anchors_poses_frame(anchor, 1), anchors_poses_frame(anchor, 2), 'ro', 'MarkerFaceColor', 'r', 'LineWidth', 1);
            title(sprintf('Anchor %d', anchor));
            axis off;
        end
    end
end