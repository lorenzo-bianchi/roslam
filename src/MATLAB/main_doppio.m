clc; clear; close all;

load('percorsi.mat', 'percorsi');

dati

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
    
for i = 0:1
    sharing = i;
    fprintf('Sharing: %d\n', sharing)

    ekfs = FedEkf.empty(nRobot, 0);

    TsGL = cell(1, nRobot);
    TsLG = cell(1, nRobot);
    for robot = 1:nRobot
        TsGL{robot} = zeros(3, 3, 0);
        TsLG{robot} = zeros(3, 3, 0);
    end

    main_simulazione;

    ekfsTot{i+1} = ekfs;
    TsGLTot{i+1} = TsGL;
    TsLGTot{i+1} = TsLG;

    %%
    calcolo_errori;

    erroreAssolutoRobotTot{i+1} = erroreAssolutoRobot;
    distanzeRobotVereTot{i+1} = distanzeRobotVere;
    distanzeRobotStimateTot{i+1} = distanzeRobotStimate;
    distanzeInterTagVereTot{i+1} = distanzeInterTagVere;
    distanzeInterTagStimateTot{i+1} = distanzeInterTagStimate;
    erroriAssolutiTagTot{i+1} = erroriAssolutiTag;
    erroriMediPostICPTot{i+1} = erroriMediPostICP;
    erroriMediTagGlobaliTot{i+1} = erroriMediTagGlobali;
    erroriPosGlobaliTot{i+1} = erroriPosGlobale;

    fprintf('\n')
end

%% Errori
fprintf('Confronto\n')
for robot = 1:nRobot
    fprintf('Robot %d:\t\t\t\t\tNO SHARING\t SHARING\t MIGLIORAMENTO\n', robot)

    mean1 = mean(abs(distanzeRobotVereTot{1}(robot, :) - distanzeRobotStimateTot{1}(robot, :)));
    mean2 = mean(abs(distanzeRobotVereTot{2}(robot, :) - distanzeRobotStimateTot{2}(robot, :)));
    fprintf('\tMedia differenza distanze robot-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    mean1 = mean(abs(distanzeInterTagVereTot{1}(robot, :) - distanzeInterTagStimateTot{1}(robot, :)));
    mean2 = mean(abs(distanzeInterTagVereTot{2}(robot, :) - distanzeInterTagStimateTot{2}(robot, :)));
    fprintf('\tMedia differenza distanze tag-tag: \t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    mean1 = mean(erroriAssolutiTagTot{1}(robot, :));
    mean2 = mean(erroriAssolutiTagTot{2}(robot, :));
    fprintf('\tMedia errori assoluti tag: \t\t%.3f \t\t %.3f \t\t %.3f \n', mean1, mean2, mean1-mean2)

    err1 = erroreAssolutoRobotTot{1}(robot);
    err2 = erroreAssolutoRobotTot{2}(robot);
    fprintf('\tErrore assoluto robot: \t\t\t%.3f \t\t %.3f \t\t %.3f \n', err1, err2, err1-err2)

    mean_rot1 = erroriMediPostICPTot{1}(robot);
    mean_rot2 = erroriMediPostICPTot{2}(robot);
    if isnan(mean_rot1) && ~isnan(mean_rot2)
        val = Inf;
    elseif isnan(mean_rot2) && ~isnan(mean_rot1)
        val = -Inf;
    else
        val = mean_rot1-mean_rot2;
    end
        
    fprintf('\tErrore medio tag post ICP: \t\t%.3f \t\t %.3f \t\t %.3f \n', mean_rot1, mean_rot2, val)

    fprintf('\tErrore medio tag post globalize: \t --- \t\t %.3f \t\t  --- \n', erroriMediTagGlobaliTot{2}(robot))

    fprintf('\tErrore robot post globalize: \t\t --- \t\t %.3f \t\t  --- \n', erroriPosGlobaliTot{2}(robot))
    fprintf('\n')
end

%% Grafici
DISEGNA_ANIMAZIONE = 0;
grafici;

%%
if true
    return
end


%%
figure('Position', [100, 100, 900, 1800]);
tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'tight');

font_size = 20;
line_width = 1;

for robot = 1:6
    nexttile
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
    
    exportgraphics(gcf, 'figures/abc.pdf', 'ContentType', 'vector');
end

%%
figure('Position', [100, 100, 900, 1800]);
tiledlayout(6, 3, 'TileSpacing', 'compact', 'Padding', 'tight');

font_size = 15;
line_width = 1.2;

for robot = 1:6
    gt = percorsi(time, 1:3, robot);
    gt(:, 3) = unwrap(gt(:, 3));
    tResetVect = [1, tResets{robot}, nPassi];

    for idx = 1:3
        nexttile
    
        plot(time, gt(:, idx), '--', 'Color', '#006400', 'Linewidth', line_width, 'DisplayName', 'Ground truth')    % traiettoria
        hold on
    
        if idx ~= 3
            posLoc = [ekfs(robot).xHatSLAM(1:2, time); ones(1, length(time))];
    
            if ~isempty(tResets{robot})
                posGlob = zeros(3, nPassi);
    
                for t_idx = 1:length(tResetVect)-1
                    t0 = tResetVect(t_idx);
                    t1 = tResetVect(t_idx+1);
        
                    T = TsGL{robot}(:, :, t_idx);
                    posGlob(:, t0:t1) = T*posLoc(:, t0:t1);
                end        
            else
                posGlob = TsGL{robot}(:, :, 1)*posLoc;
            end
        
            plot(time, posGlob(idx, :), 'b', 'LineWidth', line_width, 'DisplayName', 'Estimate')
        else
            thetaLoc = ekfs(robot).xHatSLAM(3, time);
            if ~isempty(tResets{robot})
                posGlob = zeros(3, nPassi);
                
                for t_idx = 1:length(tResetVect)-1
                    t0 = tResetVect(t_idx);
                    t1 = tResetVect(t_idx+1);
        
                    T = TsGL{robot}(:, :, t_idx);
                    thetaGlob(t0:t1) = thetaLoc(:, t0:t1) + atan2(T(2, 1), T(1, 1));
                end        
            else
                T = TsGL{robot}(:, :, 1);
                thetaGlob = thetaLoc + atan2(T(2, 1), T(1, 1));
            end
            plot(time, unwrap(thetaGlob), 'b', 'LineWidth', line_width, 'DisplayName', 'Estimate')
        end
    
        % if pruning
        %     for i = 1:length(stepStartPruning{robot})
        %         if stepStartPruning{robot}(i) > t_max
        %             continue
        %         end
        %         if i == 1
        %             xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
        %         else
        %             xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'HandleVisibility', 'off');
        %         end
        %     end
        % end
        if sharing
            first = 1;
            for t = startSharing
                if t ~= -1
                    if first
                        first = 0;
                        xline(t, '-.k', 'LineWidth', 1, 'DisplayName', 'Sharing');
                    else
                        xline(t, '-.k', 'LineWidth', 1, 'HandleVisibility', 'off');
                    end
                end
            end
        end
        if ~isempty(tResets{robot})
            for i = 1:length(tResets{robot})
                if i == 1
                    xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'DisplayName', 'Reset');
                else
                    xline(tResets{robot}(i), '--r', 'LineWidth', 1.5, 'HandleVisibility', 'off');
                end
            end
        end
        
        grid on
        if idx == 1
            ylabel('x [m]');
        elseif idx == 2
            ylabel('y [m]');
        else
            ylabel('\theta [rad]');
        end
        xticks(t_min-1:500:t_max);
    
        ax = gca;
        ax.FontSize = font_size;
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');

        if robot ~= 6
            set(gca, 'XTickLabel', []);
        else
            xlabel('simulation step');
        end
    end
    
    % sgtitle(sprintf('Absolute pose robot %d', robot), 'FontSize', font_size+4)
    
    set(gcf, 'Renderer', 'Painters');
    pause(0.2)
    set(gcf, 'Renderer', 'OpenGL');
end

h1 = plot(NaN, NaN, 'g--', 'LineWidth', line_width);
h2 = plot(NaN, NaN, 'b', 'LineWidth', line_width);
h3 = plot(NaN, NaN, 'k-.', 'LineWidth', line_width);
h4 = plot(NaN, NaN, 'r--', 'LineWidth', line_width);

lgd = legend([h1, h2, h3, h4], {'Ground truth', 'Estimate', 'Start sharing', 'Reset'}, ...
       'Orientation', 'horizontal');
lgd.Layout.Tile = 'south';