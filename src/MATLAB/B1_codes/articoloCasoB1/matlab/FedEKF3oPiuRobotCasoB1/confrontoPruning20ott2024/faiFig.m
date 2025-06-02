load confrontoPruning200unRobot
% load confrontoPruning200cinqueRobot
load confrontoPruning200cinqueRobotCon0p5
% load confrontoPruning200cinqueRobotCon0p5inizioOk
% load confrontoPruning200cinqueRobotCon0p5senzaOttimizzGrafo
% load confrontoPruning200cinqueRobotCon0p5inizCentraleSenzaOttimizzGrafo
% load confrontoPruning200treRobotCon0p5inizCentraleSenzaOttimizzGrafo

% nSim = 100;
% nRobot = 3;
nTag = 4;
[nSim,nRobot] = size(erroriAssoluti5robot);

figure % Fig. 1
% figN = gcf;
% figN.Position = [100 450 800 350];
plot(sort(erroriAssoluti5robot(:,1)),'.-')
hold on
for indRobot = 2:nRobot
    plot(sort(erroriAssoluti5robot(:,indRobot)),'.-')
end
xlabel('Simulation')
% ylabel('Errore finale assoluto cinque robot [cm]')
ylabel('Case 5 robots: final absolute robot position errors [cm]')
grid on
legend('Robot 1')

figure % Fig. 2
% figN = gcf;
% figN.Position = [100 50 800 300];
plot(sort(erroriAssolutiTag5robot(:,1)),'.-')
hold on
for indTag = 2:nTag
    plot(sort(erroriAssolutiTag5robot(:,indTag)),'.-')
end
grid on
xlabel('Simulation')
% ylabel('Errore finale assoluto Landmark (5robot) [cm]')
ylabel('Case 5 robots: final absolute landmark position errors [cm]')

figure % Fig. 3
plot(sort(erroriAssoluti5robot(:,1)),'k.-')
hold
plot(sort(mean(erroriAssoluti5robot(:,2:end)')),'b.-')
plot(sort(erroreAssolutoUnRobot),'r.-')
xlabel('Simulation')
% ylabel('Errore finale medio assoluto robot [cm]')
ylabel('Final absolute mean robot position error [cm]')
grid on
% legend('uno di 5 robot','media altri 4 robot','singolo robot')
legend('Robot 1 of 5','Mean Robots 2-5 of 5','Single robot')


figure % Fig. 4
plot(sort(mean(erroriAssolutiTag5robot')),'.-')
hold on
plot(sort(mean(erroriAssolutiTagUnRobot')),'r.-')
xlabel('Simulation')
% ylabel('Errore finale medio assoluto Landmark [cm]')
ylabel('Final absolute mean Landmark position error [cm]')
grid on
legend('5 robots','One robot')

figure % Fig. 5
plot(erroreMedioAssolutoRobot1di5storia,'k')
hold on
plot(erroreMedioAssolutoAltriRobotDi5storia,'b')
plot(erroriMediAssolutiRobotStoriaUnRobot,'r')
% plot(erroreMedioAssolutoRobot1di5storia+stdErroreMedioAssolutoRobot1di5storia,'k:')
% plot(erroreMedioAssolutoAltriRobotDi5storia+stdErroreMedioAssolutoAltriRobotDi5storia,'b:')
% plot(erroriMediAssolutiRobotStoriaUnRobot+stdErroriMediAssolutiRobotStoriaUnRobot,'r:')
xlabel('Step k')
% ylabel('Storico errori assoluti medi robot [cm]')
ylabel('Mean absolute Robot position errors [cm]')
grid on
% legend('1 di 5 robot','media altri di 5 robot','one robot')
legend('Robot 1 of 5','Mean Robots 2-5 of 5','Single robot')

figure % Fig. 6
plot(mediaErroriMediAssolutiTagStoriaCinqueRobot,'b')
hold on
plot(erroriMediAssolutiTagStoriaUnRobot,'r')
% plot(mediaErroriMediAssolutiTagStoriaCinqueRobot+stdErroriMediAssolutiTagStoriaCinqueRobot,'b:')
% plot(erroriMediAssolutiTagStoriaUnRobot+stdErroriMediAssolutiTagStoriaUnRobot,'r:')
xlabel('Step k')
% ylabel('Storico errori assoluti medi Landmark [cm]')
ylabel('Mean absolute Landmark position errors [cm]')
grid on
legend('5 robots','one robot')

% Grafici un robot da soli

% figure
% figN = gcf;
% figN.Position = [100 450 800 350];
% plot(sort(erroreAssolutoUnRobot),'.-')
% xlabel('Simulazione')
% ylabel('Errore assoluto un robot [cm]')
% grid on
% hold on
% 
% figure
% figN = gcf;
% figN.Position = [100 50 800 300];
% plot(sort(erroriAssolutiTagUnRobot(:,1)),'.-')
% hold on
% for indTag = 2:nTag
%     % figure
%     plot(sort(erroriAssolutiTagUnRobot(:,indTag)),'.-')
% end
% grid on
% legend('Landmark 1','Landmark 2','Landmark 3','Landmark 4')
% xlabel('Simulazione')
% ylabel('Errore assoluto landmark con un robot [cm]')
%
% figure
% plot(erroriMediAssolutiRobotStoriaUnRobot)
% hold on
% plot(erroriMediAssolutiRobotStoriaUnRobot+stdErroriMediAssolutiRobotStoriaUnRobot,'r')
% xlabel('Step k')
% ylabel('Errori medi mediati assoluti Landmark con un robot [cm]')
% grid on
%
% figure
% plot(erroriMediAssolutiTagStoriaUnRobot)
% hold on
% plot(erroriMediAssolutiTagStoriaUnRobot+stdErroriMediAssolutiTagStoriaUnRobot,'r')
% xlabel('Step k')
% ylabel('Errori medi mediati assoluti Landmark con un robot [cm]')
% grid on