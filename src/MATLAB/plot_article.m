N = size(robots_poses_world_history, 1);

font_size = 13;
line_width = 2;

% figure('Position', [100, 100, 1200, 600]);
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'tight');

color_x = 'r';
color_y = 'b';

robot = 2;

% Extract data for the current robot
x = squeeze(robots_poses_world_history(:, robot, 1));
y = squeeze(robots_poses_world_history(:, robot, 2));

% Subplot 1: X coordinate
nexttile(1);
plot(1:N, x, color_x, 'LineWidth', line_width);
xlabel('Frame number');
ylabel('m');
grid on;
hold on;

% Subplot 2: Y coordinate
plot(1:N, y, color_y, 'LineWidth', line_width);
grid on;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
legend('x', 'y')

% Subplot 4: XY Position
nexttile(2, [2, 1]);
plot(x, y, 'k', 'LineWidth', 2,'HandleVisibility', 'off'); % Trajectory
hold on;
plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot(x(end), y(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % End
legend('Start', 'End', 'Location', 'south');

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

set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');

exportgraphics(gcf, './figures/gt_article.pdf', 'ContentType', 'vector');
savefig(gcf, './figures/gt_article.fig');
