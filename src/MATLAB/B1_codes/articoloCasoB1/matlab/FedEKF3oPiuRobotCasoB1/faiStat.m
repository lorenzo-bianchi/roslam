nSim = 100;
nRobot = 5;
nTag = 4;
nPassi = 500;

erroreAssolutoRobotMat = zeros(nSim,nRobot);
erroreAssolutoTagMat = zeros(nSim,nTag);
erroriMediAssolutiAltriRobotStoria = zeros(nSim,nPassi);
erroriAssolutiRobot1Storia = zeros(nSim,nPassi);
erroriMediAssolutiTagStoria = zeros(nSim,nPassi);

nSimAllineati = 0;
numNaN = 0;
indSim = 0;
while indSim < nSim

    main

    if sum(allineatiVett) > 0 || isnan(mean(erroreAssolutoRobot))
        if sum(allineatiVett) > 0
            nSimAllineati = nSimAllineati + 1
        else
            numNaN = numNaN + 1
            % pause
        end
    else
        indSim = indSim + 1
        erroreAssolutoRobotMat(indSim,:) = erroreAssolutoRobot';
        erroreAssolutoTagMat(indSim,:) = erroreAssolutoTag';
        erroriMediAssolutiAltriRobotStoria(indSim,:) = mean(erroriAssolutiRobotStoria(2:nRobot,:));
        erroriAssolutiRobot1Storia(indSim,:) = erroriAssolutiRobotStoria(1,:);
        erroriMediAssolutiTagStoria(indSim,:) = mean(erroriAssolutiTagStoria);
        % save dati100 erroreAssolutoTagMat erroreAssolutoRobotMat erroriMediAssolutiRobotStoria erroriMediAssolutiTagStoria
    end

end


figure % Fig. 1
% figN = gcf;
% figN.Position = [100 450 800 350];
plot(sort(erroreAssolutoRobotMat(:,1)),'.-')
hold on
for indRobot = 2:nRobot
    plot(sort(erroreAssolutoRobotMat(:,indRobot)),'.-')
end
xlabel('Simulazione')
ylabel('Errore finale assoluto robot [cm]')
grid on
legend('robot 1')

figure % Fig. 2
% figN = gcf;
% figN.Position = [100 50 800 300];
plot(sort(erroreAssolutoTagMat(:,1)),'.-')
hold on
for indTag = 2:nTag
    plot(sort(erroreAssolutoTagMat(:,indTag)),'.-')
end
grid on
xlabel('Simulazione')
ylabel('Errore finale assoluto Landmark [cm]')

figure % Fig. 3
plot(sort(erroreAssolutoRobotMat(:,1)),'.-')
hold
plot(sort(mean(erroreAssolutoRobotMat(:,2:end)')),'.-')
xlabel('Simulazione')
ylabel('Errore finale medio assoluto robot [cm]')
grid on
legend('Robot 1','Altri robot')

figure % Fig. 4
plot(sort(mean(erroreAssolutoTagMat')),'.-')
xlabel('Simulazione')
ylabel('Errore finale medio assoluto Landmark [cm]')
grid on

% figure
% plot(erroriMediAssolutiRobot1Storia','k','LineWidth',2)
% hold on
% plot(erroriMediAssolutiAltriRobotStoria')
% xlabel('Step k')
% ylabel('Errori medi assoluti Robot [cm]')
% grid on

figure % Fig. 5
plot(mean(erroriAssolutiRobot1Storia))
xlabel('Step k')
ylabel('Media errori assoluti Robot 1 [cm]')
grid on

figure % Fig. 6
plot(mean(erroriMediAssolutiAltriRobotStoria))
xlabel('Step k')
ylabel('Media errori medi assoluti altri robot [cm]')
grid on

figure % Fig. 7
plot(erroriMediAssolutiTagStoria')
xlabel('Step k')
ylabel('Errori medi assoluti Landmark [cm]')
grid on

figure % Fig. 8
plot(mean(erroriMediAssolutiTagStoria))
xlabel('Step k')
ylabel('Media errori medi assoluti Landmark [cm]')
grid on