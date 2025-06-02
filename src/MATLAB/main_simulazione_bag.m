rng(seed);
n_robot = roslam_data.num_robots;
dati_fusi_cell = cell(n_robot, 1);
max_tempi = -1;
for robot = 1:roslam_data.num_robots
    x0 = roslam_data.ground_truth(robot).robots_poses_world_history(1, 1, 1);
    y0 = roslam_data.ground_truth(robot).robots_poses_world_history(1, 1, 2);
    theta0 = roslam_data.ground_truth(robot).robots_poses_world_history(1, 1, 3);
    TsGL{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
    TsLG{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]]^-1;

    a = roslam_data.uwb_anchors_distances{robot}(1:2, 2:end);
    if ~isnan(a(1,1))
        misureRange = [a(1,1:4), a(2,5:8)];
    else
        misureRange = [a(1,5:8), a(2,1:4)];
    end

    stato0 = [0, 0, 0];
    ekfs(robot) = FedEkfBag(data, robot, stato0, misureRange);

    misure_uwb = roslam_data.uwb_anchors_distances{robot};
    misure_odom = roslam_data.wheels_odometry{robot};

    tempi_odom = misure_odom(:,1);
    tempi_uwb  = misure_uwb(:,1);
    
    % Unisci e ordina i tempi (con duplicati)
    tempi_unificati = sort([tempi_odom; tempi_uwb]);
    
    % Prealloca la matrice finale
    dati_fusi = -100 * ones(length(tempi_unificati), 9);  % padding iniziale con -100
    dati_fusi(:,1) = tempi_unificati;                    % prima colonna = tempi
    
    if length(tempi_unificati) > max_tempi
        max_tempi = length(tempi_unificati);
    end
    for i = 1:length(tempi_unificati)
        t = tempi_unificati(i);
    
        idx_odom = find(tempi_odom == t, 1);
        idx_uwb  = find(tempi_uwb  == t, 1);
    
        if ~isempty(idx_uwb)
            % Inserisci dati UWB nelle colonne 2-9
            dati_fusi(i, 2:end) = misure_uwb(idx_uwb, 2:end);
        elseif ~isempty(idx_odom)
            % Inserisci dati odom in colonne 2-3, padding -100 nelle restanti
            dati_fusi(i, 2:3) = misure_odom(idx_odom, 2:end);
            % Le colonne 4-9 restano a -100
        end
    end
    dati_fusi_cell{robot} = dati_fusi;
end

startSharing = -1 * ones(1, nRobot);
tResets = cell(1, nRobot);
for robot = 1:nRobot
    tResets{robot} = [];
end

tic;
for k = 2:max_tempi
    for robot = 1:nRobot
        if k > size(dati_fusi_cell{robot}, 1)
            continue
        end

        riga = dati_fusi_cell{robot}(k, :);
        if abs(riga(end) + 100) < 0.001
            % Predizione
            uRe = riga(2);
            uLe = riga(3);

            ekfs(robot).prediction(uRe, uLe);
        else
            % Correzione
            misureRange = riga(2:end);
            ekfs(robot).correction(misureRange);

            ekfs(robot).save_history();

            if pruning && k >= stepStartPruning{robot}(end)
                ekfs(robot).pruning();
            end
        end

        % Salva le coordinate cartesiane dei tag
        ekfs(robot).save_history();
    end

    % if mod(k, Nstep) == 0
    %     % CORREZIONE con altre misure
    %     if sharing && k > stepStartSharing
    %         sharedInfoArray = struct('id', {}, 'tags', {}, 'vars', {});
    %         for robot = 1:nRobot
    %             if sum(ekfs(robot).nPhiVett) == nTag
    %                 if startSharing(robot) == -1
    %                     startSharing(robot) = k;
    %                 end
    %                 sharedInfoArray(end+1) = ekfs(robot).data_to_share();
    %             end
    %         end
    % 
    %         if ~isempty(sharedInfoArray)
    %             for robot = 1:nRobot
    %                 ekfs(robot).correction_shared(sharedInfoArray);
    %                 if pruning && k >= stepStartPruning{robot}(end)
    %                     ekfs(robot).pruning();
    %                 end
    %                 if ekfs(robot).do_reset
    %                     fprintf('Robot %d resetting at t=%d\n', robot, k)
    %                     stepStartPruning{robot}(end+1) = stepStartPruning0 + k;
    %                     ekfs(robot).reset();
    %                     tResets{robot}(end+1) = k;
    % 
    %                     x0 = percorsi(k, 1, robot);
    %                     y0 = percorsi(k, 2, robot);
    %                     theta0 = percorsi(k, 3, robot);
    %                     TsGL{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]];
    %                     TsLG{robot}(:, :, end+1) = [[cos(theta0) -sin(theta0) x0]; [sin(theta0) cos(theta0) y0]; [0 0 1]]^-1;
    %                 end
    %             end
    %         end
    %     end
    % end
end
fprintf("Tempo impiegato: %f s:\n", toc);

%%
global_poses = struct('ids', {}, 'use_avg', {}, 'pose', {}, 'tags', {});
if sharing && ~isempty(sharedInfoArray)
    for robot = 1:nRobot
        global_poses(end+1) = ekfs(robot).globalize(sharedInfoArray);
    end
end