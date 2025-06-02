rng(4)

font_size = 13;
line_width = 1.5;
marker_size = 10;

A = [-2.5257    1.9402;
   -2.1892   -7.7403;
    0.7103   -0.7110;
   -1.8062   -5.4765;
    2.8895   -7.9924;
    0.0521    0.3754;
    0.8739   -8.1434;
    1.3665   -0.3018;
    3.6489   -3.8663;
    1.4439   -8.0681] + [4, 4];

B = [-6.123    1.245;
    0.2685    0.6171;
   -7.0679    3.4119;
   -4.2302    4.0364;
    0.1669    5.7598;
   -8.1613    2.7375;
    0.3636    3.7533;
   -7.5057    4.0739;
   -3.9676    6.4261;
   -0.5740    4.3213];

%%
figure
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'tight');
set(gcf, 'Position', [100, 100, 900, 300]);

nexttile(1)
plot(A(:,1), A(:,2), 'r+', 'DisplayName', 'Set A', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A(i, 1), A(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-9 9])
ylim([-5 7])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

[bestR, bestT] = icp2D(A, B);

% Transform points A using the best rototranslation
A_transformed = (bestR * A' + bestT)';

nexttile(2)
plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Set A transformed', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed(i, 1), A_transformed(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
grid on;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)

set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

legend('location', 'northwest');

saveas(gcf, './figures/test_aeris_no_ransac.eps', 'epsc');
savefig(gcf, './figures/test_aeris_no_ransac.fig');

%%
figure
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'tight');
set(gcf, 'Position', [100, 100, 900, 300]);

nexttile(1)
plot(A(:,1), A(:,2), 'r+', 'DisplayName', 'Set A', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A(i, 1), A(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-9 9])
ylim([-5 7])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

numIterations = 1000;
distanceThreshold = 0.1;
minInliers = round(0.6 * size(A, 1));

[bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers);
if isempty(inliers)
    disp("No inliers")
    return
end

% Transform points A using the best rototranslation
A_transformed = (bestR * A' + bestT)';

nexttile(2)
plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Set A transformed', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed(i, 1), A_transformed(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
grid on;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)

set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

legend('location', 'northwest');

saveas(gcf, './figures/test_aeris.eps', 'epsc');
savefig(gcf, './figures/test_aeris.fig');

%%
figure
tiledlayout(1, 1, 'TileSpacing', 'compact', 'Padding', 'tight');
set(gcf, 'Position', [100, 100, 450, 300]);

nexttile(1)
plot(A(:,1), A(:,2), 'r+', 'DisplayName', 'Set A', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A(i, 1), A(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-9 9])
ylim([-5 7])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

saveas(gcf, './figures/test_aeris_orig.eps', 'epsc');
savefig(gcf, './figures/test_aeris_orig.fig');

%%
figure
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'tight');
set(gcf, 'Position', [100, 100, 900, 300]);

nexttile(1)

[bestR, bestT] = icp2D(A, B);
A_transformed = (bestR * A' + bestT)';

plot(A_transformed (:,1), A_transformed (:,2), 'r+', 'DisplayName', 'Set A transformed (ICP)', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed (i, 1), A_transformed (i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
legend('location', 'northwest');

numIterations = 1000;
distanceThreshold = 0.1;
minInliers = round(0.6 * size(A, 1));

[bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers);
if isempty(inliers)
    disp("No inliers")
    return
end

% Transform points A using the best rototranslation
A_transformed = (bestR * A' + bestT)';

nexttile(2)
plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Set A transformed (AERIS)', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed(i, 1), A_transformed(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

legend;
% title(sprintf('Tags alignment robot %d', robot));
xlabel('x [m]');
ylabel('y [m]');
axis equal;
grid on;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)

set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

legend('location', 'northwest');

saveas(gcf, './figures/test_aeris_both.eps', 'epsc');
savefig(gcf, './figures/test_aeris_both.fig');

%%
figure
t = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'tight');
set(gcf, 'Position', [100, 100, 900, 600]);  % più alta per fare spazio a 3 figure uguali

% --------- FIGURA IN ALTO (CENTRATA su 2 colonne)
nexttile([1 2])  % Figura 1
plot(A(:,1), A(:,2), 'r+', 'DisplayName', 'Set A', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

% Etichette A
for i = 1:size(A, 1)
    text(A(i, 1), A(i, 2) + 0.15, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

% Etichette B
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.2, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-9 9])
ylim([-5 7])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
legend;
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

% --------- FIGURA IN BASSO A SINISTRA
nexttile(3)  % Figura 2
[bestR, bestT] = icp2D(A, B);
A_transformed = (bestR * A' + bestT)';

plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Set A transformed (ICP)', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed(i, 1), A_transformed(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
legend('location', 'northwest');
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

% --------- FIGURA IN BASSO A DESTRA
nexttile(4)  % Figura 3

numIterations = 1000;
distanceThreshold = 0.1;
minInliers = round(0.6 * size(A, 1));
[bestR, bestT, inliers] = ransacRototranslation(A, B, numIterations, distanceThreshold, minInliers);

if isempty(inliers)
    disp("No inliers")
    return
end

A_transformed = (bestR * A' + bestT)';

plot(A_transformed(:,1), A_transformed(:,2), 'r+', 'DisplayName', 'Set A transformed (AERIS)', 'MarkerSize', marker_size, 'LineWidth', 2);
hold on;
plot(B(:,1), B(:,2), 'kx', 'DisplayName', 'Set B', 'MarkerSize', marker_size, 'LineWidth', 2);

for i = 1:size(A, 1)
    text(A_transformed(i, 1), A_transformed(i, 2) + 0.1, ...
         ['A', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
for i = 1:size(B, 1)
    text(B(i, 1), B(i, 2) - 0.12, ...
         ['B', num2str(i)], 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top');
end

xlabel('x [m]');
ylabel('y [m]');
axis equal;
xlim([-10.5 1.5])
ylim([-0.5 7.5])
xticks(-20:1:20)
yticks(-20:1:20)
grid on;
legend('location', 'northwest');
set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

% SALVATAGGIO
saveas(gcf, './figures/test_aeris_all.eps', 'epsc');
savefig(gcf, './figures/test_aeris_all.fig');
