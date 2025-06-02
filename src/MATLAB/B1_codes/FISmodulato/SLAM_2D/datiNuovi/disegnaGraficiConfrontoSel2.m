passiSel = 0:50:1000;
passiSel(1) = 1;
passiSel2 = 25:50:1000;
passiSel2(1) = 1;
passiSel2(end) = 1000;

load dati0

figure
fig1 = gcf;
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'bs','LineWidth',1,'MarkerSize',4)
plot(passiSel,erroreSel,'bo','LineWidth',1,'MarkerSize',5)
hold on
% plot(erroreVett,'b','LineWidth',1)
xlabel('Simulation')
ylabel('RMSE robot [cm]')
grid on

figure
fig2 = gcf;
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'s-','LineWidth',1,'MarkerSize',4)
plot(passiSel,erroreSel,'bo','LineWidth',1,'MarkerSize',5)
hold on
% plot(erroreVett,'b','LineWidth',1)
grid on
xlabel('Simulation')
ylabel('Errore assoluto landmark [cm]')

load dati200

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel2);
% plot(passiSel,erroreSel,'o-','LineWidth',1,'MarkerSize',3)
plot(passiSel2,erroreSel,'ks','LineWidth',1,'MarkerSize',6)
% plot(erroreVett,'m','LineWidth',1)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel2);
% plot(passiSel,erroreSel,'o-','LineWidth',1,'MarkerSize',3)
plot(passiSel2,erroreSel,'ks','LineWidth',1,'MarkerSize',6)
% plot(erroreVett,'m','LineWidth',1)


load datiNoPruning

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'x-','LineWidth',1)
plot(passiSel,erroreSel,'rx','LineWidth',1,'MarkerSize',7)
% plot(erroreVett,'r','LineWidth',1)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'x-','LineWidth',1)
plot(passiSel,erroreSel,'rx','LineWidth',1,'MarkerSize',7)
% plot(erroreVett,'r','LineWidth',1)

load datiKmisto

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'.-','LineWidth',1,'MarkerSize',10)
plot(passiSel,erroreSel,'k.','LineWidth',1,'MarkerSize',12)
% plot(erroreVett,'k','LineWidth',1)

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel);
% plot(passiSel,erroreSel,'.-','LineWidth',1,'MarkerSize',10)
plot(passiSel,erroreSel,'k.','LineWidth',1,'MarkerSize',12)
% plot(erroreVett,'k','LineWidth',1)

% load CampanaroReInit2
load CampanaroReInit2azzerato

figure(fig1)
erroreVett = sort(rmseRobotVett);
erroreSel = erroreVett(passiSel2);
% plot(passiSel,erroreSel,'v-','LineWidth',1,'MarkerSize',3)
plot(passiSel2,erroreSel,'rv','LineWidth',1,'MarkerSize',4)
% plot(erroreVett,'g','LineWidth',1)
legend('T_{pr}=0','T_{pr}=200','No pruning','Proposed (1)','Proposed (2)')
set(gca, 'YScale', 'log')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
erroreSel = erroreVett(passiSel2);
% plot(passiSel,erroreSel,'v-','LineWidth',1,'MarkerSize',3)
plot(passiSel2,erroreSel,'rv','LineWidth',1,'MarkerSize',4)
% plot(erroreVett,'g','LineWidth',1)
legend('T_{pr}=0','T_{pr}=200','No pruning','Proposed (1)','Proposed (2)')
set(gca, 'YScale', 'log')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Facciamo ora simboli sui grafici
load dati0

figure(fig1)
erroreVett = sort(rmseRobotVett);
plot(erroreVett,'b','LineWidth',1,'HandleVisibility','off')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
plot(erroreVett,'b','LineWidth',1,'HandleVisibility','off')

load dati200

figure(fig1)
erroreVett = sort(rmseRobotVett);
plot(erroreVett,'k','LineWidth',1,'HandleVisibility','off')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
plot(erroreVett,'k','LineWidth',1,'HandleVisibility','off')

load datiNoPruning

figure(fig1)
erroreVett = sort(rmseRobotVett);
plot(erroreVett,'r','LineWidth',1,'HandleVisibility','off')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
plot(erroreVett,'r','LineWidth',1,'HandleVisibility','off')

load datiKmisto

figure(fig1)
erroreVett = sort(rmseRobotVett);
plot(erroreVett,'k','LineWidth',1,'HandleVisibility','off')

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
plot(erroreVett,'k','LineWidth',1,'HandleVisibility','off')

% load CampanaroReInit2
load CampanaroReInit2azzerato

figure(fig1)
erroreVett = sort(rmseRobotVett);
plot(erroreVett,'r','LineWidth',1,'HandleVisibility','off')
axis([-20 1020 0 150])

figure(fig2)
erroreVett = sort(mean(erroriAssolutiTagMat'));
plot(erroreVett,'r','LineWidth',1,'HandleVisibility','off')
axis([-20 1020 0 150])