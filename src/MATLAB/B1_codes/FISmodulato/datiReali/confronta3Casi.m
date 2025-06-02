DISEGNA = 0;
robotScelto = 2;
sigmaDistanza = 10;
bias = 7;
kSwitch = 10000;
sogliaInnovazione = 0.3;%Inf;
% IE = 126;%200; 
% R1 sogliaInnovazione 0.2 o 0.3, IE80, plot fino 1450, bias 7
% R2 sogliaInnovazione 0.3 ma meglio 0.2, IE126, bias 7 
% R3 sogliaInnovazione 0.3, IE30, plot fino 1200, bias 7
if robotScelto == 1
    IE = 80;
    passiPlot = 1450;
end
if robotScelto == 2
    IE = 126;
    passiPlot = 1528;
end
if robotScelto == 3
    IE = 30;
    passiPlot = 1200;
end

figure
passoInizioPruning = 1000;
FedEKFconPruningSperimentaleDopoAvvioGT
intervalloPlot = 1:passiPlot;
intervalloPlotMarker = 1:50:passiPlot;
tempiTot = sort(repmat(tempiMisure,2,1));
tempiTot = tempiTot - tempiTot(1);
rmseRobot = sqrt(mean(erroreTraiettoria(intervalloPlot).^2))
subplot(2,1,1) % figure(100+robotScelto)
plot(tempiTot(intervalloPlot),mean(erroreTag(:,intervalloPlot)),'k','LineWidth',1)%,'HandleVisibility','off')
hold on
% plot(tempiTot(intervalloPlotMarker),mean(erroreTag(:,intervalloPlotMarker)),'kx','LineWidth',1)
xlabel('Time [s]')
ylabel('cm')
grid on
subplot(2,1,2)% figure(200+robotScelto)
plot(tempiTot(intervalloPlot),erroreTraiettoria(intervalloPlot),'k','LineWidth',1)%,'HandleVisibility','off')
hold on
% plot(tempiTot(intervalloPlotMarker),erroreTraiettoria(intervalloPlotMarker),'kx','LineWidth',1)
xlabel('Time [s]')
ylabel('cm')
grid on

insiemeSperGT
% FedEKFconPruningConKmistoSenzaDoppioniSperDopoAvvioSwitch
rmseRobot = sqrt(mean(erroreTraiettoria(intervalloPlot).^2))
subplot(2,1,1) % figure(100+robotScelto)
plot(tempiTot(intervalloPlot),mean(erroreTag(:,intervalloPlot)),'b-.','LineWidth',1)%,'HandleVisibility','off')
% plot(tempiTot(intervalloPlotMarker),mean(erroreTag(:,intervalloPlotMarker)),'b.','LineWidth',1,'MarkerSize',10)
subplot(2,1,2) % figure(200+robotScelto)
plot(tempiTot(intervalloPlot),erroreTraiettoria(intervalloPlot),'b-.','LineWidth',1)%,'HandleVisibility','off')
% plot(tempiTot(intervalloPlotMarker),erroreTraiettoria(intervalloPlotMarker),'b.','LineWidth',1,'MarkerSize',10)

CampanaroReInit2SperimentaleDopoAvvioAzzerandoGT
rmseRobot = sqrt(mean(erroreTraiettoria(intervalloPlot).^2))
subplot(2,1,1) % figure(100+robotScelto)
plot(tempiTot(intervalloPlot),mean(erroreTag(:,intervalloPlot)),'r--','LineWidth',1)%,'HandleVisibility','off')
% plot(tempiTot(intervalloPlotMarker),mean(erroreTag(:,intervalloPlotMarker)),'rv','LineWidth',1,'MarkerSize',4)
legend('T_{pr}=1000','Mod. filter','Reinit. filter')
set(gca, 'YScale', 'log')
if robotScelto == 1
    axis([0 75 8 250])
end
if robotScelto == 2
    axis([0 79 10 250])
    plot(21.4*[1 1],[10 250],'k--','HandleVisibility','off','LineWidth',1)
end
subplot(2,1,2) % figure(200+robotScelto)
plot(tempiTot(intervalloPlot),erroreTraiettoria(intervalloPlot),'r--','LineWidth',1)%,'HandleVisibility','off')
% plot(tempiTot(intervalloPlotMarker),erroreTraiettoria(intervalloPlotMarker),'rv','LineWidth',1,'MarkerSize',4)
legend('T_{pr}=1000','Mod. filter','Reinit. filter')
% set(gca, 'YScale', 'log')
if robotScelto == 1
    axis([0 75 0 30])
end
if robotScelto == 2
    axis([0 79 0 45])
    plot(21.4*[1 1],[0 45],'k--','HandleVisibility','off','LineWidth',1)
end

% title(['Robot ' num2str(robotScelto) ', sigmaD = ',num2str(sigmaDistanza)])

% if robotScelto == 2
%     plot(342*[1 1],[0 250],'k-.','LineWidth',1)
% end


