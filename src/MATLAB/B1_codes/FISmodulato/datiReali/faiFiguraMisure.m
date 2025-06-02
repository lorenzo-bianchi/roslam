% Da eseguire dopo confronta3casi.m

misureRobotLandmark = [];
misureAtteseRobotLandmarkGT = [];

robot = robotScelto;

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

if robot == 1
    tempiMisure = matrice(IE/2+10:end,1);
else
    tempiMisure = matrice(IE/2+1:end,1);
end

if robot == 1
    landmark = 2;
else
    landmark = 4;
end

if robot == 1
    misureVere = 100*matrice(IE/2+10:end,landmark+1)+7;
else
    misureVere = 100*matrice(IE/2+1:end,landmark+1)+7;
end
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

tempiMisure = tempiMisure - tempiMisure(1);

[mmm indFinale] = min(abs(tempiMisure-tempiTot(intervalloPlot(end))));

figure
plot(tempiMisure(1:indFinale),misureRobotLandmark(1:indFinale),'r','LineWidth',1)
hold on
plot(tempiMisure(1:indFinale),misureAtteseRobotLandmarkGT(1:indFinale),'k--','LineWidth',1)
legend('true','expected')
grid on
xlabel('Time [s]')
ylabel('cm')
if robot == 1
    axis([0 75 50 350])
end
if robot == 2
    axis([0 79 120 310])
end
