nSim = 100;
nTag = 4;
nPassi = 500;

erroreAssolutoRobotVett = zeros(nSim,1);
erroriAssolutiTagMat = zeros(nSim,nTag);
erroriMediAssolutiTagStoria = zeros(nSim,nPassi);
erroriMediAssolutiRobotStoria = zeros(nSim,nPassi);

for indSim = 1:nSim

    indSim

    main

    erroreAssolutoRobotVett(indSim) = robots(1).erroreAssolutoRobot;
    erroriAssolutiTagMat(indSim,:) = robots(1).erroriAssolutiTag;
    erroriMediAssolutiTagStoria(indSim,:) = mean(robots(1).erroreTag);
    erroriMediAssolutiRobotStoria(indSim,:) = robots(1).erroreTraiettoria;

end

figure
figN = gcf;
figN.Position = [100 450 800 350];
plot(sort(erroreAssolutoRobotVett),'.-')
xlabel('Simulazione')
ylabel('Errore assoluto robot [cm]')
grid on
hold on

figure
figN = gcf;
figN.Position = [100 50 800 300];
plot(sort(erroriAssolutiTagMat(:,1)),'.-')
hold on
for indTag = 2:nTag
    % figure
    plot(sort(erroriAssolutiTagMat(:,indTag)),'.-')
end
grid on
legend('Landmark 1','Landmark 2','Landmark 3','Landmark 4')
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')

figure
plot(erroriMediAssolutiTagStoria')
xlabel('Step k')
ylabel('Errori medi assoluti Landmark [cm]')
grid on

figure
plot(mean(erroriMediAssolutiTagStoria))
xlabel('Step k')
ylabel('Errori medi mediati assoluti Landmark [cm]')
grid on

figure
plot(mean(erroriMediAssolutiRobotStoria))
xlabel('Step k')
ylabel('Errori mediati assoluti Robot [cm]')
grid on