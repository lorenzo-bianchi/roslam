if length(dbstack) == 1
    robot = 3;
end

figure

font_size = 15;
line_width = 1.2;

gt = percorsi(time, 1:3, robot);
tResetVect = [1, tResets{robot}, nPassi];

positions = [0.05, 0.53];
tiledlayout(1, 3, 'TileSpacing', 'compact')
for idx = 1:3
    nexttile

    plot(time, gt(:, idx), 'r--', 'Linewidth', line_width, 'DisplayName', 'Ground truth')    % traiettoria
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
        plot(time, thetaGlob, 'b', 'LineWidth', line_width, 'DisplayName', 'Estimate')
    end

    if pruning
        for i = 1:length(stepStartPruning{robot})
            if stepStartPruning{robot}(i) > t_max
                continue
            end
            if i == 1
                xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'DisplayName', 'Pruning');
            else
                xline(stepStartPruning{robot}(i), '--k', 'LineWidth', 1, 'HandleVisibility', 'off');
            end
        end
    end
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
    xlabel('simulation step');
    if idx == 1
        ylabel('x [m]');
    elseif idx == 2
        ylabel('y [m]');
    else
        ylabel('\theta [rad]');
        legend('location', 'eastoutside');
    end
    xticks(t_min-1:250:t_max);

    ax = gca;
    ax.FontSize = font_size;
end

sgtitle(sprintf('Absolute pose robot %d', robot), 'FontSize', font_size+4)

set(gcf, 'Renderer', 'Painters');
pause(0.2)
set(gcf, 'Renderer', 'OpenGL');

% if length(dbstack) == 1
%     saveas(gcf, sprintf('./Article/robot%d.eps', robot), 'epsc');
% end
