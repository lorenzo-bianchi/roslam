clc; close all; clear

%% Numbers
num_anchors = 4;
num_points = 11;
num_robots = 1;
num_tests = 11;

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



new_names_anchors = ["A1", "A2", "A3", "A4"];
new_names_points = ["A6", "B", "C", "D", "E", "F", "A8", "H", "I", "A5", "A7"];
new_names = [new_names_anchors, new_names_points];

pts = containers.Map(new_names, num2cell(z_opt, 2));
anchors = [pts("A1"); pts("A2"); pts("A3"); pts("A4"); ...
           pts("A5"); pts("A6"); pts("A7"); pts("A8")];

num_anchors = num_anchors + 4;
num_points = num_points - 4;

angles = containers.Map(["right", "up", "left", "down"], [0.0, pi/2, pi, -pi/2]);

%% Plot optimized points
figure;
hold on;
grid on;
pts_keys = keys(pts);
for idx = 1:num_anchors+num_points
    key = pts_keys{idx};
    xy = pts(key);
    if length(key) == 2
        num = key(2);
        disp([xy(1), xy(2)])
        plot(xy(1), xy(2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
        text(xy(1), xy(2)+0.1, num2str(num), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'red');
    else
        plot(xy(1), xy(2), 'xk', 'Linewidth', 2, 'MarkerSize', 12);
        text(xy(1), xy(2)+0.05, key, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 12, 'FontWeight', 'bold');
    end
end

xlabel('x [m]')
ylabel('y [m]')
axis equal;
xlim([min(z_opt(:,1))-1, max(z_opt(:,1))+1]);
ylim([min(z_opt(:,2))-1, max(z_opt(:,2))+1]);
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

%% GT B2
starting_poses = [
    "I_left";
    "H_up";
    "F_right";
    "B_down";
    "D_right";
    "C_down";
    "E_up";
    "I_right";
    "C_up";
    "F_up";
    "D_left";
];

final_distances_anchors = zeros(size(starting_poses, 1), num_anchors);
final_distances_anchors(1, :)  = [0.62 1.33 3.56 3.45 2.20 2.78 2.04 0.99];
final_distances_anchors(2, :)  = [1.85 0.76 2.93 3.50 1.19 2.55 2.50 0.95];
final_distances_anchors(3, :)  = [0.85 1.97 3.32 2.89 2.31 2.38 1.39 0.97];
final_distances_anchors(4, :)  = [2.12 1.41 2.30 2.92 0.70 1.93 2.10 0.72];
final_distances_anchors(5, :)  = [2.27 2.43 1.85 1.78 1.43 0.96 1.04 0.93];
final_distances_anchors(6, :)  = [2.68 3.18 1.94 1.08 2.17 0.78 0.65 1.65];
final_distances_anchors(7, :)  = [2.19 1.95 1.93 2.31 0.91 1.36 1.57 0.66];
final_distances_anchors(8, :)  = [0.62 1.33 3.56 3.45 2.20 2.78 2.04 0.99];
final_distances_anchors(9, :)  = [1.92 1.27 2.48 2.99 0.89 2.04 2.07 0.59];
final_distances_anchors(10, :) = [1.04 1.70 3.04 2.82 1.93 2.19 1.45 0.59];
final_distances_anchors(11, :) = [2.35 2.48 1.77 1.74 1.41 0.89 1.07 0.99];


function f = nested_cost_fun2(z, num_anchors, anchors, dist_robot_anchors)
    f = 0;

    % Robot-anchor distances
    for a = 1:num_anchors
        x_r = 1;
        y_r = 2;
        x_a = anchors(a, 1);
        y_a = anchors(a, 2);

        dist_computed = sqrt((z(x_r) - x_a)^2 + (z(y_r) - y_a)^2);
        f = f + (dist_computed - dist_robot_anchors(1, a))^2;
    end
end

final_positions_robots = zeros(num_tests, 2);
for test = 1:num_tests
    dist_robot_anchors = squeeze(final_distances_anchors(test, :, :));

    cost_fun = @(z) nested_cost_fun2(z, num_anchors, anchors, dist_robot_anchors);

    z0 = zeros(2, 1);
    options = optimoptions('fminunc', 'MaxFunctionEvaluations', 1e5, 'Display', 'iter', 'Algorithm', 'quasi-newton');
    [z_opt2, ~] = fminunc(cost_fun, z0, options);

    final_positions_robots(test, :) = z_opt2';
end

%% Print results
for test = 1:num_tests
    fprintf('Test %d:\n', test);
    starting_pose = char(starting_poses(test, 1));
    pnt0_char = starting_pose(1);
    angle0_char = starting_pose(3:end);
    pnt0 = pts(pnt0_char);
    angle0 = angles(angle0_char);
    fprintf('\tStart: %s %s --> (%.2f, %.2f, %.2f)\n', pnt0_char, angle0_char, pnt0(1), pnt0(2), angle0);
    fprintf('\t');
    for anchor = 1:num_anchors
        fprintf('dA%d = %.2f  ', anchor, final_distances_anchors(test, anchor));
    end
    fprintf('\n');

    fprintf('\tEnd (GT): (%.2f, %.2f)\n', final_positions_robots(test, 1), final_positions_robots(test, 2));
    fprintf('\n');
end