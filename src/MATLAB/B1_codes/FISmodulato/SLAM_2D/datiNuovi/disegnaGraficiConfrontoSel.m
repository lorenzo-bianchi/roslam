passiSel = 0:50:1000;
passiSel(1) = 1;

load dati0

figure
fig1 = gcf;
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'s-','LineWidth',1,'MarkerSize',4)
hold on
xlabel('Simulazione')
ylabel('RMSE robot [cm]')
grid on

figure
fig2 = gcf;
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'s-','LineWidth',1,'MarkerSize',4)
hold on
grid on
xlabel('Simulazione')
ylabel('Errore assoluto landmark [cm]')

load dati200

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'o-','LineWidth',1,'MarkerSize',3)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'o-','LineWidth',1,'MarkerSize',3)


load datiNoPruning

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'x-','LineWidth',1)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'x-','LineWidth',1)

load datiKmisto

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'.-','LineWidth',1,'MarkerSize',10)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'.-','LineWidth',1,'MarkerSize',10)

% load CampanaroReInit2
load CampanaroReInit2azzerato

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'v-','LineWidth',1,'MarkerSize',3)
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')
set(gca, 'YScale', 'log')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
plot(passiSel,erroreSel,'v-','LineWidth',1,'MarkerSize',3)
% legend('0','50','100','200','500','FedEKF')
legend('0','200','FedEKF','Kmisto','ReInit')
set(gca, 'YScale', 'log')