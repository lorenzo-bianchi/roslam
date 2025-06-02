nSim = 100;
nTag = 4;
nPassi = 500;

erroreAssolutoRobotVett_1 = zeros(nSim,1);
rmseRobotVett_1 = zeros(nSim,1);
erroriAssolutiTagMat_1 = zeros(nSim,nTag);

erroreAssolutoRobotVett_2 = zeros(nSim,1);
rmseRobotVett_2 = zeros(nSim,1);
erroriAssolutiTagMat_2 = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    % FedEKFconPruningConKmistoSenzaDoppioniSwitch
    % FedEKFconPruningKmistoOneShot
    FedEKFconPruning
    erroreAssolutoRobotVett_1(indSim) = erroreAssolutoRobot;
    rmseRobotVett_1(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat_1(indSim,:) = erroriAssolutiTag;

    FedEKFconPruningConKmistoSenzaDoppioni
    % CampanaroReInit2azzerando
    % FedEKFconPruning
    erroreAssolutoRobotVett_2(indSim) = erroreAssolutoRobot;
    rmseRobotVett_2(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat_2(indSim,:) = erroriAssolutiTag;
end

figure
% figN = gcf;
% figN.Position = [100 450 800 350];
plot(sort(erroreAssolutoRobotVett_1),'.-')
hold on
plot(sort(erroreAssolutoRobotVett_2),'r.-')
xlabel('Simulazione')
ylabel('Errore assoluto robot [cm]')
grid on
legend('1','2')

figure
% figN = gcf;
% figN.Position = [100 450 800 350];
plot(sort(rmseRobotVett_1),'.-')
hold on
plot(sort(rmseRobotVett_2),'r.-')
xlabel('Simulazione')
ylabel('RMSE robot [cm]')
grid on
legend('1','2')

figure
% figN = gcf;
% figN.Position = [100 50 800 300];
plot(sort(mean(erroriAssolutiTagMat_1')),'.-')
hold on
plot(sort(mean(erroriAssolutiTagMat_2')),'r.-')
grid on
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')
legend('1','2')