nSim = 100;
nTag = 4;
nPassi = 500;

erroreAssolutoRobotVett_1 = zeros(nSim,1);
erroriAssolutiTagMat_1 = zeros(nSim,nTag);
erroriMediAssolutiRobotStoria_1 = zeros(nSim,nPassi);
erroriMediAssolutiTagStoria_1 = zeros(nSim,nPassi);

erroreAssolutoRobotVett_2 = zeros(nSim,1);
erroriAssolutiTagMat_2 = zeros(nSim,nTag);
erroriMediAssolutiRobotStoria_2 = zeros(nSim,nPassi);
erroriMediAssolutiTagStoria_2 = zeros(nSim,nPassi);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    FedEKFconPruningConKmistoSenzaDoppioni
    erroreAssolutoRobotVett_1(indSim) = erroreAssolutoRobot;
    erroriAssolutiTagMat_1(indSim,:) = erroriAssolutiTag;
    erroriMediAssolutiRobotStoria_1(indSim,:) = erroreTraiettoria;
    erroriMediAssolutiTagStoria_1(indSim,:) = mean(erroreTag);

    CampanaroReInit2
    % CampanaroMaster
    % FedEKFconPruning
    erroreAssolutoRobotVett_2(indSim) = erroreAssolutoRobot;
    erroriAssolutiTagMat_2(indSim,:) = erroriAssolutiTag;
    erroriMediAssolutiRobotStoria_2(indSim,:) = erroreTraiettoria;
    erroriMediAssolutiTagStoria_2(indSim,:) = mean(erroreTag);
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
% figN.Position = [100 50 800 300];
plot(sort(mean(erroriAssolutiTagMat_1')),'.-')
hold on
plot(sort(mean(erroriAssolutiTagMat_2')),'r.-')
grid on
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')
legend('1','2')

figure
plot(mean(erroriMediAssolutiRobotStoria_1),'.-')
hold on
plot(mean(erroriMediAssolutiRobotStoria_2),'r.-')
xlabel('Step k')
ylabel('Errori mediati assoluti Robot [cm]')
grid on
legend('1','2')

figure
plot(mean(erroriMediAssolutiTagStoria_1),'.-')
hold on
plot(mean(erroriMediAssolutiTagStoria_2),'r.-')
xlabel('Step k')
ylabel('Errori mediati assoluti Landmark [cm]')
grid on
legend('1','2')
