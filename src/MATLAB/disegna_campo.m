% serve prima chiamare main

robot = 6;

font_size = 20;
line_width = 1;

figure('Position', [100, 100, 500, 500]);
plot([0 L], [0 0], 'k', 'Linewidth', 2)
hold on
plot([0 L], [L L], 'k', 'Linewidth', 2)
plot([0 0], [0 L], 'k', 'Linewidth', 2)
plot([L L], [0 L], 'k', 'Linewidth', 2)
for indTag = 1:nTag
    plot(cTag(indTag,1), cTag(indTag,2), 'rh', 'Linewidth', 2, 'MarkerSize', 14);
    text(cTag(indTag,1), cTag(indTag,2) + 0.16, ...
         num2str(indTag), 'FontSize', font_size, 'FontWeight', 'bold', ...
         'Color', 'r', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

xVett = percorsi(1:nPassi, 1, robot);
yVett = percorsi(1:nPassi, 2, robot);

plot(xVett, yVett, 'k-.', 'Linewidth', 1)    % traiettoria
plot(xVett(1), yVett(1), 'ko', 'MarkerSize', 10, 'Linewidth', 3)  % pos iniziale
plot(xVett(end), yVett(end), 'ks', 'MarkerSize', 10, 'Linewidth', 3) % pos finale

xlabel('x [m]')
ylabel('y [m]')
grid on
axis equal
delta = 0.1*L;
axis([-delta L+delta 0 L])

set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

exportgraphics(gcf, 'figures/B2_field.pdf', 'ContentType', 'vector');