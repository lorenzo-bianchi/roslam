clc; close all; clear

%% Numbers
num_anchors = 4;
num_points = 11;
num_robots = 3;

%%
d12 = 1.80;
d23 = 3.70;
d34 = 1.88;
d14 = 3.74;
d13 = 4.08;
d24 = 4.19;

dist_anchors = [
    0, d12, d13, d14;
    d12, 0, d23, d24;
    d13, d23, 0, d34;
    d14, d24, d34, 0
];

dist_anchors_points = [
    3.21 3.29 1.11 1.07;      % A
    3.03 2.76 1.05 1.79;      % B      
    2.69 3.18 1.92 1.06;      % C
    2.26 2.45 1.81 1.74;      % D
    2.20 1.91 1.93 2.34;      % E
    1.66 2.39 2.64 2.08;      % F
    1.52 1.54 2.60 2.68;      % G
    1.90 0.77 2.92 3.50;      % H
    0.65 1.32 3.52 3.40;      % I
    2.80 1.89 1.87 2.92;      % J
    2.23 3.05 2.56 1.55;      % K
];

names_anchors = ["A1", "A2", "A3", "A4"];
names_points = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K"];
names = [names_anchors, names_points];
pairs = {
    'AK', 1.41;
    'AJ', 1.85;
    'AI', 2.74;
    'AH', 2.55;
    'AG', 1.88;
    'AF', 1.62;
    'AE', 1.37;
    'AD', 0.91;
    'AC', 0.78;
    'AB', 0.71;

    'BK', 1.75;
    'BJ', 1.17;
    'BI', 2.49;
    'BH', 2.00;
    'BG', 1.55;
    'BF', 1.67;
    'BE', 0.91;
    'BD', 0.82;
    'BC', 1.24;

    'CK', 0.65;
    'CJ', 2.17;
    'CI', 2.35;
    'CH', 2.52;
    'CG', 1.65;
    'CF', 1.05;
    'CE', 1.43;
    'CD', 0.79;

    'DK', 1.04;
    'DJ', 1.44;
    'DI', 1.84;
    'DH', 1.76;
    'DG', 0.98;
    'DF', 0.86;
    'DE', 0.64;

    'EK', 1.61;
    'EJ', 0.87;
    'EI', 1.62;
    'EH', 1.18;
    'EG', 0.68;
    'EF', 1.18;

    'FK', 0.69;
    'FJ', 2.05;
    'FI', 1.34;
    'FH', 1.88;
    'FG', 0.92;

    'GK', 1.56;
    'GJ', 1.36;
    'GI', 0.94;
    'GH', 0.97;

    'HK', 2.51;
    'HJ', 1.15;
    'HI', 1.27;

    'IK', 2.01;
    'IJ', 2.16;

    'JK', 2.46;
};

dist_points = zeros(num_points, num_points);
for i = 1:length(pairs)
    pair = pairs{i, 1};
    dist = pairs{i, 2};
    idx1 = find(names_points == pair(1));
    idx2 = find(names_points == pair(2));
    dist_points(idx1, idx2) = dist;
    dist_points(idx2, idx1) = dist;
end

dist = [dist_anchors dist_anchors_points'; dist_anchors_points dist_points];

%% Compute points coordinates
num_total = num_anchors+num_points;
dim_state = 2*num_total;

function f = nested_cost_fun(z, num_total, dist)
    f = 0;
    for i = 1:num_total-1
        for j = i:num_total
            f = f + (sqrt(  ( z(2*i-1) - z(2*j-1) )^2 + ( z(2*i) - z(2*j) )^2  ) - dist(i, j))^2;
        end
    end
end

cost_fun = @(z) nested_cost_fun(z, num_total, dist);

Aeq = zeros(3, dim_state);
Aeq(1, 1) = 1;
Aeq(2, 2) = 1;
Aeq(3, 4) = 1;
beq = [0 0 0];

A = [];
b = [];

lb = [];
ub = [];

z0 = zeros(dim_state, 1);
z0(1:2*num_anchors) = [0, 0, d12, 0, d12/2, sqrt(d13^2 - (d12/2)^2), 0, sqrt(d14^2 - d12^2)];

options = optimoptions('fmincon', 'MaxFunctionEvaluations', 1e5, 'Display', 'iter', 'Algorithm', 'sqp');
[z_opt, fval] = fmincon(cost_fun, z0, A, b, Aeq, beq, lb, ub, [], options);

z_opt = reshape(z_opt, 2, [])';

%% Plot optimized points
figure;
hold on;
grid on;
for idx = 1:num_anchors+num_points
    if idx <= num_anchors
        plot(z_opt(idx, 1), z_opt(idx, 2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
        text(z_opt(idx, 1), z_opt(idx, 2)+0.1, num2str(idx), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
    else
        plot(z_opt(idx, 1), z_opt(idx, 2), 'xk', 'Linewidth', 2, 'MarkerSize', 12);
        text(z_opt(idx, 1), z_opt(idx, 2)+0.05, names_points(idx-num_anchors), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'FontWeight', 'bold');
    end
end

xlabel('x [m]')
ylabel('y [m]')
axis equal;
xlim([min(z_opt(:,1))-1, max(z_opt(:,1))+1]);
ylim([min(z_opt(:,2))-1, max(z_opt(:,2))+1]);
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

%%
anchors = z_opt(1:num_anchors, :);
pts = containers.Map(names, num2cell(z_opt, 2));
angles = containers.Map(["right", "up", "left", "down"], [0.0, pi/2, pi, -pi]);

%% GT
starting_poses = [
    ["I_up", "E_left", "C_right"];
    ["H_up", "F_down", "B_down"];
    ["G_down", "A_down", "H_up"];
    ["F_right", "E_down", "I_up"];
    ["E_up", "C_up", "I_up"];
    ["D_down", "A_right", "I_right"];
    ["C_right", "F_down", "H_up"];
    ["B_left", "C_down", "G_left"];
];

final_distances_anchors = zeros(size(starting_poses, 1), size(starting_poses, 2), num_anchors);
final_distances_anchors(1, :, :) = [[0.34 1.64 3.73 3.45]; [2.05 2.65 2.30 1.69]; [2.17 1.99 1.93 2.25]];
final_distances_anchors(2, :, :) = [[1.33 0.58 3.43 3.72]; [2.36 2.90 2.09 1.39]; [2.63 2.34 1.47 2.06]];
final_distances_anchors(3, :, :) = [[1.81 2.08 2.28 2.16]; [2.59 2.45 1.49 1.87]; [0.99 1.85 3.11 2.78]];
final_distances_anchors(4, :, :) = [[1.81 2.50 2.50 1.92]; [1.98 1.53 2.27 2.72]; [1.13 1.78 2.95 2.71]];
final_distances_anchors(5, :, :) = [[1.48 2.76 3.33 2.45]; [2.52 2.74 1.69 1.47]; [2.93 2.66 1.15 1.86]];
final_distances_anchors(6, :, :) = [[3.64 3.57 0.69 1.25]; [1.21 1.96 2.91 2.57]; [1.60 1.10 2.77 3.11]];
final_distances_anchors(7, :, :) = [[1.14 2.27 3.20 2.59]; [2.05 2.01 2.02 2.21]; [2.59 2.59 1.52 1.68]];
final_distances_anchors(8, :, :) = [[2.93 2.91 1.21 1.45]; [1.78 2.24 2.36 2.06]; [2.12 1.35 2.36 2.98]];

final_distances_robots = [
    % d12  d13  d23
    [1.77 1.83 1.02];
    [3.39 1.99 1.20];
    [0.81 0.85 1.65];
    [0.34 0.82 1.06];
    [1.66 2.35 0.78];
    [2.45 2.54 1.07];
    [1.35 1.70 0.61];
    [1.18 1.62 1.32];
];





function f = nested_cost_fun2(z, num_robots, num_anchors, anchors, dist_robot_robot, dist_robot_anchors)
    f = 0;

    % Robot-robot distances
    for i = 1:num_robots-1
        for j = i:num_robots
            x_i = 2*i;
            y_i = 2*i - 1;
            x_j = 2*j;
            y_j = 2*j - 1;

            dist_computed = sqrt((z(x_i) - z(x_j))^2 + (z(y_i) - z(y_j))^2);
            f = f + (dist_computed - dist_robot_robot(i, j))^2;
        end
    end

    % Robot-anchor distances
    for r = 1:num_robots
        for a = 1:num_anchors
            x_r = 2*r - 1;
            y_r = 2*r;
            x_a = anchors(a, 1);
            y_a = anchors(a, 2);

            dist_computed = sqrt((z(x_r) - x_a)^2 + (z(y_r) - y_a)^2);
            f = f + (dist_computed - dist_robot_anchors(r, a))^2;
        end
    end
end

final_positions_robots = zeros(test, 2*num_robots);
for test = 1:size(final_distances_robots, 1)
    dist_robot_robot = zeros(num_robots);
    dist_robot_robot(1, 2) = final_distances_robots(test, 1); % d12
    dist_robot_robot(1, 3) = final_distances_robots(test, 2); % d13
    dist_robot_robot(2, 3) = final_distances_robots(test, 3); % d23
    dist_robot_robot = dist_robot_robot + dist_robot_robot';

    dist_robot_anchors = squeeze(final_distances_anchors(test, :, :));

    cost_fun = @(z) nested_cost_fun2(z, num_robots, num_anchors, anchors, dist_robot_robot, dist_robot_anchors);

    z0 = zeros(2*num_robots, 1);
    options = optimoptions('fminunc', 'MaxFunctionEvaluations', 1e5, 'Display', 'iter', 'Algorithm', 'quasi-newton');
    [z_opt2, ~] = fminunc(cost_fun, z0, options);

    final_positions_robots(test, :) = z_opt2';
end

%% Print results
for test = 1:size(final_distances_robots, 1)
    fprintf('Test %d:\n', test);
    for robot = 1:num_robots
        fprintf('\tRobot%d:\n', robot);
        starting_pose = char(starting_poses(test, robot));
        pnt0_char = starting_pose(1);
        angle0_char = starting_pose(3:end);
        pnt0 = pts(pnt0_char);
        angle0 = angles(angle0_char);
        fprintf('\t\t%s %s --> (%.2f, %.2f, %.2f)\n', pnt0_char, angle0_char, pnt0(1), pnt0(2), angle0);
        fprintf('\t\t');
        for anchor = 1:num_anchors
            fprintf('dA%d = %.2f  ', anchor, final_distances_anchors(test, robot, anchor));
        end
        fprintf('\n');
    end
    fprintf('\tGround truth:\n');
    fprintf('\t\tDistances: d12 = %.2f  d13 = %.2f  d23 = %.2f\n', final_distances_robots(test, 1), final_distances_robots(test, 2), final_distances_robots(test, 3));
    fprintf('\t\tPosition: x = %.2f, y = %.2f\n', final_positions_robots(test, 1), final_positions_robots(test, 2));
    fprintf('\n');
end