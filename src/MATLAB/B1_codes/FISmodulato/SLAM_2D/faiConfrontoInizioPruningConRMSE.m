nSim = 1000;
nTag = 4;


passoInizioPruning = 0;

erroreAssolutoRobotVett = zeros(nSim,1);
rmseRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    FedEKFconPruning%ConKmisto
    erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
    rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;

end

save dati0

figure
% plot(sort(erroreAssolutoRobotVett),'LineWidth',1,'bx-','MarkerSize',5)
plot(sort(erroreAssolutoRobotVett),'.-')
hold on
xlabel('Simulazione')
ylabel('Errore assoluto robot [cm]')
grid on

figure
plot(sort(rmseRobotVett),'.-')
hold on
xlabel('Simulazione')
ylabel('RMSE robot [cm]')
grid on

figure
plot(sort(mean(erroriAssolutiTagMat')),'.-')
hold on
grid on
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nSim = 1000;
% nTag = 4;
% 
% 
% passoInizioPruning = 50;
% 
% erroreAssolutoRobotVett = zeros(nSim,1);
% rmseRobotVett = zeros(nSim,1);
% erroriAssolutiTagMat = zeros(nSim,nTag);
% 
% for indSim = 1:nSim
% 
%     indSim
% 
%     dati
% 
%     percorsoRandom
% 
%     FedEKFconPruning%ConKmisto
%     erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot 
%     rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
%     erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;
% 
% end
% 
% % save dati50
% 
% figure(1)
% plot(sort(erroreAssolutoRobotVett),'.-')
% 
% figure(2)
% plot(sort(rmseRobotVett),'.-')
% 
% figure(3)
% plot(sort(mean(erroriAssolutiTagMat')),'.-')
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nSim = 1000;
% nTag = 4;
% 
% 
% passoInizioPruning = 100;
% 
% erroreAssolutoRobotVett = zeros(nSim,1);
% rmseRobotVett = zeros(nSim,1);
% erroriAssolutiTagMat = zeros(nSim,nTag);
% 
% for indSim = 1:nSim
% 
%     indSim
% 
%     dati
% 
%     percorsoRandom
% 
%     FedEKFconPruning%ConKmisto
%     erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
%     rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
%     erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;
% 
% end
% 
% % save dati100
% 
% figure(1)
% plot(sort(erroreAssolutoRobotVett),'.-')
% 
% figure(2)
% plot(sort(rmseRobotVett),'.-')
% 
% figure(3)
% plot(sort(mean(erroriAssolutiTagMat')),'.-')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nSim = 1000;
nTag = 4;


passoInizioPruning = 200;

erroreAssolutoRobotVett = zeros(nSim,1);
rmseRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    FedEKFconPruning%ConKmisto
    erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
    rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;

end

save dati200

figure(1)
plot(sort(erroreAssolutoRobotVett),'.-')

figure(2)
plot(sort(rmseRobotVett),'.-')

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'.-')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nSim = 1000;
% nTag = 4;
% 
% passoInizioPruning = 500;
% 
% erroreAssolutoRobotVett = zeros(nSim,1);
% rmseRobotVett = zeros(nSim,1);
% erroriAssolutiTagMat = zeros(nSim,nTag);
% 
% for indSim = 1:nSim
% 
%     indSim
% 
%     dati
% 
%     percorsoRandom
% 
%     FedEKFconPruning%ConKmisto
%     erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
%     rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
%     erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;
% 
% end
% 
% % save dati500
% 
% figure(1)
% plot(sort(erroreAssolutoRobotVett),'.-')
% 
% figure(2)
% plot(sort(rmseRobotVett),'.-')
% 
% figure(3)
% plot(sort(mean(erroriAssolutiTagMat')),'.-')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nSim = 1000;
nTag = 4;

erroreAssolutoRobotVett = zeros(nSim,1);
rmseRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    FedEKF
    erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
    rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;

end

save datiNoPruning

figure(1)
plot(sort(erroreAssolutoRobotVett),'.-')

figure(2)
plot(sort(rmseRobotVett),'.-')

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'.-')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nSim = 1000;
nTag = 4;

erroreAssolutoRobotVett = zeros(nSim,1);
rmseRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    FedEKFconPruningConKmistoSenzaDoppioni
    erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
    rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;

end

save datiKmisto

figure(1)
plot(sort(erroreAssolutoRobotVett),'.-')

figure(2)
plot(sort(rmseRobotVett),'.-')

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'.-')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nSim = 1000;
nTag = 4;

erroreAssolutoRobotVett = zeros(nSim,1);
rmseRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);

for indSim = 1:nSim

    indSim

    dati

    percorsoRandom

    CampanaroReInit2azzerando
    erroreAssolutoRobotVett(indSim) = erroreAssolutoRobot;
    rmseRobotVett(indSim) = sqrt(mean(erroreTraiettoria.^2));
    erroriAssolutiTagMat(indSim,:) = erroriAssolutiTag;

end

save CampanaroReInit2azzerato

figure(1)
plot(sort(erroreAssolutoRobotVett),'.-')
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')

figure(2)
plot(sort(rmseRobotVett),'.-')
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'.-')
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')