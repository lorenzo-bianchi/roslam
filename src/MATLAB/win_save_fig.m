imshow(last_frame);
hold on
for robot = 1: n_robots
    plot(last_trajectories(:, robot, 1), last_trajectories(:, robot, 2), '--', 'Color', robot_colors(robot, :), 'LineWidth', 1.5)
end
exportgraphics(gcf, ['figures/', test_name, '.png'], 'Resolution', 300);

imshow(last_frame);
exportgraphics(gcf, ['figures/', test_name, '_empty.png'], 'Resolution', 300);