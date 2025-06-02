close all

load('gt_test7.mat')

anchors_poses_real = [    0.0,    0.0;
                       1.7903,    0.0;
                       1.7241, 3.6934;
                      -0.1471, 3.7211];

N = size(robots_poses_world_history, 1);

font_size = 10;
line_width = 2;

robot1 = 1;
robot2 = 2;

x1 = squeeze(robots_poses_world_history(:, robot1, 1));
y1 = squeeze(robots_poses_world_history(:, robot1, 2));

x2 = squeeze(robots_poses_world_history(:, robot2, 1));
y2 = squeeze(robots_poses_world_history(:, robot2, 2));


%%
figure('Position', [100, 100, 400, 600]);

% Subplot 4: XY Position
% nexttile(robot);
plot(x1, y1, 'Color', "#D95319", 'LineWidth', 2); % Trajectory
hold on;
plot(x2, y2, 'Color', "#4DBEEE", 'LineWidth', 2); % Trajectory

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

hold on;
plot(x1(1), y1(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x1(end), y1(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
plot(x2(1), y2(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x2(end), y2(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End

hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

% Add anchor labels
for i = 1:size(anchors_poses_real, 1)
    text(anchors_poses_real(i, 1), anchors_poses_real(i, 2) + 0.05, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-0.5, 2.1]);
ylim([min(anchors_poses_real(:,2))-0.2, max(anchors_poses_real(:,2))+0.2]);
grid on;
hold off;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');
legend('Test 1', 'Test 2', 'Start', 'End', 'Location', 'south');

% set(gcf, 'Renderer', 'Painters');
% pause(0.2)
% set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, 'gt_article_equal.pdf', 'ContentType', 'vector');
savefig(gcf, 'gt_article.fig');

%%
figure('Position', [100, 100, 500, 400]);

% Subplot 4: XY Position
% nexttile(robot);
plot(x1, y1, 'Color', "#D95319", 'LineWidth', 2); % Trajectory
hold on;
plot(x2, y2, 'Color', "#4DBEEE", 'LineWidth', 2); % Trajectory

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

hold on;
plot(x1(1), y1(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x1(end), y1(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
plot(x2(1), y2(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x2(end), y2(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End

hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

% Add anchor labels
for i = 1:size(anchors_poses_real, 1)
    text(anchors_poses_real(i, 1), anchors_poses_real(i, 2) + 0.05, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

xlabel('x [m]');
ylabel('y [m]');
% axis equal;
xlim([-0.5, 2.1]);
ylim([min(anchors_poses_real(:,2))-0.2, max(anchors_poses_real(:,2))+0.3]);
grid on;
hold off;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');
legend('Test 1', 'Test 2', 'Start', 'End', 'Location', 'south');

% set(gcf, 'Renderer', 'Painters');
% pause(0.2)
% set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, 'gt_article_not_equal.pdf', 'ContentType', 'vector');
savefig(gcf, 'gt_article_not_equal.fig');

%%
figure('Position', [100, 100, 400, 600]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'tight');

% Subplot 4: XY Position
nexttile(1);
plot(x1, y1, 'Color', "#D95319", 'LineWidth', 2, 'HandleVisibility', 'off'); % Trajectory
hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

hold on;
plot(x1(1), y1(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x1(end), y1(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End

hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

% Add anchor labels
for i = 1:size(anchors_poses_real, 1)
    text(anchors_poses_real(i, 1), anchors_poses_real(i, 2) + 0.05, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

xlabel('x [m]');
ylabel('y [m]');
% axis equal;
xlim([-0.5, 2.1]);
ylim([min(anchors_poses_real(:,2))-0.2, max(anchors_poses_real(:,2))+0.3]);
grid on;
hold off;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');
legend('Start', 'End', 'Location', 'south');

% Subplot 4: XY Position
nexttile(2);
plot(x2, y2, 'Color', "#4DBEEE", 'LineWidth', 2, 'HandleVisibility', 'off'); % Trajectory
hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

hold on;
plot(x2(1), y2(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x2(end), y2(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End

hold on;

% Plot anchors as black 'X' markers
plot(anchors_poses_real(:, 1), anchors_poses_real(:, 2), 'rh', 'MarkerSize', 12, 'LineWidth', 2, 'HandleVisibility', 'off');

% Add anchor labels
for i = 1:size(anchors_poses_real, 1)
    text(anchors_poses_real(i, 1), anchors_poses_real(i, 2) + 0.05, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

xlabel('x [m]');
ylabel('y [m]');
% axis equal;
xlim([-0.5, 2.1]);
ylim([min(anchors_poses_real(:,2))-0.2, max(anchors_poses_real(:,2))+0.3]);
grid on;
hold off;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold', 'Box', 'on');
legend('Start', 'End', 'Location', 'south');

% set(gcf, 'Renderer', 'Painters');
% pause(0.2)
% set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, 'gt_article_tiled_not_equal.pdf', 'ContentType', 'vector');
savefig(gcf, 'gt_article_tiled_not_equal.fig');