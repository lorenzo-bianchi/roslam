% Data from the table
GT = [0.50, 0.75, 1.00, 1.25, 1.50, 1.75, 2.00, 2.25, 2.50, 3.65]; % Ground truth (GT)
mu = [0.25, 0.57, 0.86, 1.16, 1.28, 1.55, 1.87, 2.03, 2.26, 3.48]; % Mean (mu)
b = [0.25, 0.18, 0.14, 0.09, 0.22, 0.20, 0.13, 0.22, 0.24, 0.17]; % Bias (b)
sigma = [0.017, 0.023, 0.016, 0.015, 0.013, 0.018, 0.018, 0.022, 0.018, 0.018]; % Standard deviation (sigma)

% Plotting
figure;
hold on;
% Plot the data with error bars
plot(GT, mu, 'bo-', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', '$\mu$');
plot(GT, GT, 'ro-', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', 'GT');
plot(GT, b,  'go-', 'LineWidth', 1, 'MarkerSize', 6, 'DisplayName', '$b$');

% Labels and title
xlabel('actual distance [m]');
ylabel('[m]');

% Adding legend
lgd = legend('show', 'Location', 'northwest');
set(lgd,'Interpreter','latex');
set(lgd,'FontSize', 13, 'FontWeight', 'bold');

% Grid and axis settings
grid on;
axis equal;
hold off;
axis tight;
xlim([0.45, 3.7])
ylim([0.0, 3.7])
yticks([0 0.1 0.2 0.3 0.5 1.0 1.5 2.0 2.5 3.0 3.5]);

set(gca, 'FontSize', 13, 'FontWeight', 'bold', 'Box', 'on');

exportgraphics(gcf, './figures/uwb_table.pdf', 'ContentType', 'vector');
savefig(gcf, './figures/uwb_table.fig');