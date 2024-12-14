clc; close all; clear

d12 = 1.80;
d23 = 3.70;
d34 = 1.88;
d41 = 3.74;
d13 = 4.08;
d24 = 4.19;

objective = @(P) (sqrt((P(1) - P(3))^2 + (P(2) - P(4))^2) - d12)^2 + ... % d12
                 (sqrt((P(3) - P(5))^2 + (P(4) - P(6))^2) - d23)^2 + ... % d23
                 (sqrt((P(5) - P(7))^2 + (P(6) - P(8))^2) - d34)^2 + ... % d34
                 (sqrt((P(7) - P(1))^2 + (P(8) - P(2))^2) - d41)^2 + ... % d41
                 (sqrt((P(1) - P(5))^2 + (P(2) - P(6))^2) - d13)^2 + ... % d13
                 (sqrt((P(3) - P(7))^2 + (P(4) - P(8))^2) - d24)^2;      % d24

% P = [x1, y1, x2, y2, x3, y3, x4, y4]
initial_guess = [0, 0, d12, 0, d12/2, sqrt(d13^2 - (d12/2)^2), 0, sqrt(d41^2 - d12^2)];

options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');

Aeq = [1 0 0 0 0 0 0 0
       0 1 0 0 0 0 0 0;
       0 0 0 1 0 0 0 0];
beq = [0 0 0];

A = [];
b = [];

lb = [];
ub = [];

[P_opt, fval] = fmincon(objective, initial_guess, A, b, Aeq, beq, lb, ub, [], options);

point1 = P_opt(1:2);
point2 = P_opt(3:4);
point3 = P_opt(5:6);
point4 = P_opt(7:8);

points = [point1; point2; point3; point4];
figure()
plot(points(:, 1), points(:, 2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
xlabel('x [m]')
ylabel('y [m]')
min_x = min(points(:, 1));
max_x = max(points(:, 1));
min_y = min(points(:, 2));
max_y = max(points(:, 2));
l_x = max_x - min_x;
l_y = max_y - min_y;
alpha = 0.1;
xlim([min_x-alpha*l_x max_x+alpha*l_x])
ylim([min_y-alpha*l_y max_y+alpha*l_y])
axis equal
grid on
hold on

%%
distances = [3.21 3.29 1.11 1.07;
             3.03 2.76 1.05 1.79;
             2.69 3.18 1.92 1.06;
             2.26 2.45 1.81 1.74;
             2.20 1.91 1.93 2.34;
             1.66 2.39 2.64 2.08;
             1.52 1.54 2.60 2.68;
             1.90 0.77 2.92 3.50;
             0.65 1.32 3.52 3.40];
names = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I'];

for idx = 1:length(names)
    d1N = distances(idx, 1);
    d2N = distances(idx, 2);
    d3N = distances(idx, 3);
    d4N = distances(idx, 4);
    
    objective = @(pN) (sqrt((pN(1) - point1(1))^2 + (pN(2) - point1(2))^2) - d1N)^2 + ...
                      (sqrt((pN(1) - point2(1))^2 + (pN(2) - point2(2))^2) - d2N)^2 + ...
                      (sqrt((pN(1) - point3(1))^2 + (pN(2) - point3(2))^2) - d3N)^2 + ...
                      (sqrt((pN(1) - point4(1))^2 + (pN(2) - point4(2))^2) - d4N)^2;
    
    initial_guess = [0, 0];

    options = optimoptions('fminunc', 'Display', 'iter', 'Algorithm', 'quasi-newton');

    [pointN, fval] = fminunc(objective, initial_guess, options);
    
    fprintf('Coordinate ottimizzate del nuovo punto: [%.2f, %.2f]\n', pointN(1), pointN(2));
    fprintf('Errore quadratico totale: %.4f\n', fval);
    
    plot(pointN(1, 1), pointN(1, 2), 'xk', 'Linewidth', 2, 'MarkerSize', 12);
    text(pointN(1, 1), pointN(1, 2)+0.05, names(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'FontWeight', 'bold');
end

xlim([min_x-alpha*l_x max_x+alpha*l_x])
ylim([min_y-alpha*l_y max_y+alpha*l_y])

set(gca, 'FontSize', 12, 'FontWeight', 'bold');