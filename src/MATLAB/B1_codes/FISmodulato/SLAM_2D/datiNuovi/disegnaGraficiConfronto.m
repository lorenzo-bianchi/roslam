passiSel = 1:10:1000;

load dati0

figure
% plot(sort(erroreAssolutoRobotVett),'LineWidth',1,'bx-','MarkerSize',5)
plot(sort(erroreAssolutoRobotVett),'LineWidth',1)
hold on
xlabel('Simulazione')
ylabel('Errore assoluto robot [cm]')
grid on

figure
plot(sort(rmseRobotVett),'LineWidth',1)
hold on
xlabel('Simulazione')
ylabel('RMSE robot [cm]')
grid on


figure
plot(sort(mean(erroriAssolutiTagMat')),'LineWidth',1)
hold on
grid on
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')

load dati200

figure(1)
plot(sort(erroreAssolutoRobotVett),'LineWidth',1)

figure(2)
plot(sort(rmseRobotVett),'LineWidth',1)

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'LineWidth',1)

load datiNoPruning

figure(1)
plot(sort(erroreAssolutoRobotVett),'LineWidth',1)

figure(2)
plot(sort(rmseRobotVett),'LineWidth',1)

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'LineWidth',1)

load datiKmisto

figure(1)
plot(sort(erroreAssolutoRobotVett),'LineWidth',1)

figure(2)
plot(sort(rmseRobotVett),'LineWidth',1)

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'LineWidth',1)

% load CampanaroReInit2
load CampanaroReInit2azzerato

figure(1)
plot(sort(erroreAssolutoRobotVett),'LineWidth',1)
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')
close

figure(2)
plot(sort(rmseRobotVett),'LineWidth',1)
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')
set(gca, 'YScale', 'log')

figure(3)
plot(sort(mean(erroriAssolutiTagMat')),'LineWidth',1)
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')
set(gca, 'YScale', 'log')