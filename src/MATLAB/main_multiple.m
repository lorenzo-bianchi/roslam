% INITIAL VALUES
% nRobot = 6;
% nPassi = 2000;
% nPhi = 16;
% pruning = 1;
% minZerosStartPruning = ceil(nPhi*0.3);
% stepStartPruning0 = 10;
% sharing = 0;
% stepStartSharing = 600;
% resetThr = 100;
% sigmaDistanza = 0.2;
% sigmaDistanzaModello = sigmaDistanza;
% sigmaMisuraMedia = 0.2;
% numIterationsA = 120;
% distanceThresholdA = 0.2;
% percentMinInliersA = 0.4;
% numIterationsB = 120;
% distanceThresholdB = 0.1;
% percentMinInliersB = 0.6;
% Nstep = 1;
% possibiliPhi = linspace(-pi+2*pi/nPhi, pi, nPhi);
% sigmaPhi = 2*pi/(1.5*nPhi);
% L = 10;
% dVera = 0.16;
% deltaRvera = 1;
% deltaLvera = 1;
% deltaR = deltaRvera;
% deltaL = deltaLvera;
% KRvera = 0.0001;

% sims_100_reset: reset = 1;
% sims_100_no_reset: reset = 0;
% sims_100_pruning_100_no_reset: stepStartPruning0 = 100; reset = 0;
% sims_100_pruning_100_reset: stepStartPruning0 = 100; reset = 1;
% sims_100_8_hyp_reset: nPhi = 8; reset = 1;
% sims_100_8_hyp_no_reset: nPhi = 8; reset = 0;
% sims_100_8_hyp_sigma_up_reset: nPhi = 8; reset = 1; sigmaDistanza = 0.4;
% sims_100_8_hyp_sigma_up_no_reset: nPhi = 8; reset = 0; sigmaDistanza = 0.4;
% sims_100_8_hyp_sigma2_up_reset: nPhi = 8; reset = 1; sigmaMisuraMedia = 1.0;
% sims_100_8_hyp_sigma2_up_no_reset: nPhi = 8; reset = 0; sigmaMisuraMedia = 1.0;
% sims_100_8_hyp_pruning_100_no_reset: nPhi = 8; reset = 0; stepStartPruning0 = 100;
% sims_100_8_hyp_pruning_100_reset: nPhi = 8; reset = 1; stepStartPruning0 = 100;


clc; clear; close all;

perform_reset = 1;
dati_multiple

filename = 'data/sims_100_8_hyp_reset.mat';
disp("Check filename:")
disp(filename)
disp("num hyp:")
disp(nPhi)
disp("sigma:")
disp(sigmaDistanza)
disp("start pruning:")
disp(stepStartPruning0)
disp("reset:")
disp(perform_reset)
pause()

load('percorsi.mat', 'percorsi');
percorsi_init = percorsi;

ekfsTot = {};
TsGLTot = {};
TsLGTot = {};

erroreAssolutoRobotTot = {};
distanzeRobotVereTot = {};
distanzeRobotStimateTot = {};
distanzeInterTagVereTot = {};
distanzeInterTagStimateTot = {};
erroriAssolutiTagTot = {};
erroriMediPostICPTot = {};
erroriMediTagGlobaliTot = {};
erroriPosGlobaliTot = {};

v = 1:size(percorsi, 3);

for test = 1:100
    disp(test)
    seed = test + 42;
    rng(seed)
    dati_multiple
    data.reset = perform_reset;
    
    robot_numbers = v(randperm(length(v), 6));

    percorsi = zeros(nPassi, 5, nRobot);
    w = 0;
    for robot = robot_numbers
        w = w + 1;
        k0 = randi([1, size(percorsi_init, 1)-nPassi-100]);
        percorsi(:, :, w) = percorsi_init(k0:k0+nPassi-1, :, robot);
    end
    
    for i = 0:1
        sharing = i;
    
        ekfs = FedEkf.empty(nRobot, 0);
    
        TsGL = cell(1, nRobot);
        TsLG = cell(1, nRobot);
        for robot = 1:nRobot
            TsGL{robot} = zeros(3, 3, 0);
            TsLG{robot} = zeros(3, 3, 0);
        end
    
        rng(seed);
        for robot = 1:nRobot
            x0 = percorsi(1, 1, robot);
            y0 = percorsi(1, 2, robot);
            theta0 = percorsi(1, 3, robot);
            TsGL{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
            TsLG{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]]^-1;
        
            misureRange = sqrt((x0-cTag(:,1)).^2+(y0-cTag(:,2)).^2) + sigmaDistanza*randn;
        
            stato0 = [0, 0, 0];
            ekfs(robot) = FedEkf(data, robot, stato0, misureRange);
        end
        main_simulazione_multiple;
    
        ekfsTot{i+1} = ekfs;
        TsGLTot{i+1} = TsGL;
        TsLGTot{i+1} = TsLG;
    
        calcolo_errori_multiple;
        erroreAssolutoRobotTot{2*(test-1)+i+1} = erroreAssolutoRobot;
        distanzeRobotVereTot{2*(test-1)+i+1} = distanzeRobotVere;
        distanzeRobotStimateTot{2*(test-1)+i+1} = distanzeRobotStimate;
        distanzeInterTagVereTot{2*(test-1)+i+1} = distanzeInterTagVere;
        distanzeInterTagStimateTot{2*(test-1)+i+1} = distanzeInterTagStimate;
        erroriAssolutiTagTot{2*(test-1)+i+1} = erroriAssolutiTag;
        erroriMediPostICPTot{2*(test-1)+i+1} = erroriMediPostICP;
        erroriMediTagGlobaliTot{2*(test-1)+i+1} = erroriMediTagGlobali;
        % erroriPosGlobaliTot{2*(test-1)+i+1} = erroriPosGlobale;

        % DISEGNA_ANIMAZIONE = 0;
        % grafici;
    end
end

% %% Errori
% fprintf('Confronto\n')
% for robot = 1:nRobot
%     fprintf('Robot %d:\t\t\t\t\tNO SHARING\t SHARING\t MIGLIORAMENTO\n', robot)
% 
%     mean1 = mean(abs(distanzeRobotVereTot{1}(robot, :) - distanzeRobotStimateTot{1}(robot, :)));
%     mean2 = mean(abs(distanzeRobotVereTot{2}(robot, :) - distanzeRobotStimateTot{2}(robot, :)));
%     fprintf('\tMedia differenza distanze robot-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)
% 
%     mean1 = mean(abs(distanzeInterTagVereTot{1}(robot, :) - distanzeInterTagStimateTot{1}(robot, :)));
%     mean2 = mean(abs(distanzeInterTagVereTot{2}(robot, :) - distanzeInterTagStimateTot{2}(robot, :)));
%     fprintf('\tMedia differenza distanze tag-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)
% 
%     mean1 = mean(erroriAssolutiTagTot{1}(robot, :));
%     mean2 = mean(erroriAssolutiTagTot{2}(robot, :));
%     fprintf('\tMedia errori assoluti tag: \t\t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)
% 
%     err1 = erroreAssolutoRobotTot{1}(robot);
%     err2 = erroreAssolutoRobotTot{2}(robot);
%     fprintf('\tErrore assoluto robot: \t\t\t%.3f \t\t %.3f \t\t %.3f \n', err1, err2, err1-err2)
% 
%     mean_rot1 = erroriMediPostICPTot{1}(robot);
%     mean_rot2 = erroriMediPostICPTot{2}(robot);
%     if isnan(mean_rot1) && ~isnan(mean_rot2)
%         val = Inf;
%     elseif isnan(mean_rot2) && ~isnan(mean_rot1)
%         val = -Inf;
%     else
%         val = mean_rot1-mean_rot2;
%     end
% 
%     fprintf('\tErrore medio tag post ICP: \t\t%.3f \t\t %.3f \t\t %.3f \n', mean_rot1, mean_rot2, val)
% 
%     fprintf('\tErrore medio tag post globalize: \t --- \t\t %.3f \t\t  --- \n', erroriMediTagGlobaliTot{2}(robot))
% 
%     fprintf('\tErrore robot post globalize: \t\t --- \t\t %.3f \t\t  --- \n', erroriPosGlobaliTot{2}(robot))
%     fprintf('\n')
% end

%% Grafici
% DISEGNA_ANIMAZIONE = 0;
% grafici;

%% Save variables
save(filename, 'erroreAssolutoRobotTot', 'distanzeRobotVereTot', 'distanzeRobotStimateTot', ...
    'distanzeInterTagVereTot', 'distanzeInterTagStimateTot', 'erroriAssolutiTagTot', ...
    'erroriMediPostICPTot', 'erroriMediTagGlobaliTot');

%%
% Errore assoluto robot
single = erroreAssolutoRobotTot(1:2:end);
multi = erroreAssolutoRobotTot(2:2:end);

avg_single = zeros(1, 100);
avg_multi = zeros(1, 100);

for i = 1:100
    avg_single(i) = mean(single{i});
    avg_multi(i) = mean(multi{i});
end

figure
plot(1:100, avg_single)
grid on
hold on
plot(1:100, avg_multi)
legend('single robot', 'multi robot')
xlabel('simulation');
ylabel('absolute error [m]');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

sorted_avg_single = sort(avg_single);
sorted_multi = sort(avg_multi);

figure
plot(1:100, sorted_avg_single)
grid on
hold on
plot(1:100, sorted_multi)
legend('single robot', 'multi robot')
xlabel('simulation');
ylabel('absolute error [m]');
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

%% Errore assoluto robot
single = erroriMediPostICPTot(1:2:end);
multi = erroriMediPostICPTot(2:2:end);

avg_single = zeros(1, 100);
avg_multi = zeros(1, 100);

for i = 1:100
    avg_single(i) = mean(single{i});
    avg_multi(i) = mean(multi{i});
end

figure
plot(1:100, avg_single)
grid on
hold on
plot(1:100, avg_multi)
legend('single robot', 'multi robot')
xlabel('simulation');
ylabel('error post AERIS [m]');
xlim([1 100])
set(gca, 'FontSize', 12, 'FontWeight', 'bold');

sorted_avg_single = sort(avg_single);
sorted_multi = sort(avg_multi);

figure
plot(1:100, sorted_avg_single)
grid on
hold on
plot(1:100, sorted_multi)
legend('single robot', 'multi robot')
xlabel('simulation');
ylabel('error post AERIS [m]');
xlim([1 100])
set(gca, 'FontSize', 12, 'FontWeight', 'bold');