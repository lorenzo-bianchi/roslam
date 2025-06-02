load roslam_data3.mat
load gt_test7.mat

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]*100;
nTag = 4;

misureRobotLandmark = [];
misureAtteseRobotLandmarkGT = [];

for robot = 3:3

    if robot == 1
        tempiGT = robot1(:,1) + 11.1; % per allinearli con tempiMisure
        xGT = 100*robot1(:,2);
        yGT = 100*robot1(:,3);
    end
    if robot == 2
        tempiGT = robot2(:,1) + 11; % per allinearli con tempiMisure
        xGT = 100*robot2(:,2);
        yGT = 100*robot2(:,3);
    end
    if robot == 3
        tempiGT = robot3(:,1) + 11; % per allinearli con tempiMisure
        xGT = 100*robot3(:,2);
        yGT = 100*robot3(:,3);
    end

    matrice = roslam_data.uwb_anchors_distances{robot,1};
    tempiMisure = matrice(:,1);

    for landmark = 4:4

        misureVere = 100*matrice(:,landmark+1)+10;
        nMisure = numel(tempiMisure);
        misureAttese = zeros(nMisure,1);
        for indMisura = 1:nMisure
            [mm,indMin] = min(abs(tempiMisure(indMisura)-tempiGT));
            misureAttese(indMisura) = sqrt((xGT(indMin)-cTag(landmark,1))^2+(yGT(indMin)-cTag(landmark,2))^2);
            if isnan(misureVere(indMisura))
                misureVere(indMisura) = misureAttese(indMisura);
            end
            % if misureVere(indMisura) < 180
            %     misureVere(indMisura) = misureVere(indMisura) + 5;
            % else
            %     if misureVere(indMisura) > 200
            %         misureVere(indMisura) = misureVere(indMisura) - 5;
            %     end
            % end
        end
        misureRobotLandmark = [misureRobotLandmark; misureVere];        
        misureAtteseRobotLandmarkGT = [misureAtteseRobotLandmarkGT; misureAttese];
    end

end


figure
plot(misureRobotLandmark)
hold on
plot(misureAtteseRobotLandmarkGT,'k')
legend('vere','attese GT')
xlabel('indice misura')
ylabel('cm')

% Calcolo errore misura
erroriMisura = misureRobotLandmark - misureAtteseRobotLandmarkGT;

% Le misureAtteseRobotLandmarkGT sono di fatto la distanza effettiva
% robot-landmark.

[ordinati indiciOrdinati] = sort(misureAtteseRobotLandmarkGT);

erroriMisuraOrdinati = erroriMisura(indiciOrdinati);

% filtro errori misura
nTot = numel(erroriMisura);
w = 20; %2w e' la finestra in cui si fa la media
erroriFiltratiMisuraOrdinati = zeros(nTot,1);
for indErrore = 1:nTot
    indMin = max(1,indErrore-w);
    indMax = min(nTot,indErrore+w);    
    erroriFiltratiMisuraOrdinati(indErrore) = mean(erroriMisuraOrdinati(indMin:indMax));
    % pause
end

figure
histogram(erroriMisura)
xlabel('valore errore')
ylabel('occorrenza')

mediaErrori = mean(erroriMisura)
stdErrori = std(erroriMisura)

figure
% plot(ordinati,erroreMisura(indiciOrdinati))
plot(misureAtteseRobotLandmarkGT(indiciOrdinati),erroriMisuraOrdinati)
hold on
plot(misureAtteseRobotLandmarkGT(indiciOrdinati),erroriFiltratiMisuraOrdinati,'r')
grid on
xlabel('distanza [cm]')
ylabel('errore misura [cm]')
legend('errore','errore mediato')