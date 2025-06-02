DISEGNA = 0;
robotScelto = 3;
sigmaDistanza = 10;
bias = 10;
kSwitch = 100;

% FedEKFconPruningSperimentaleDopoAvvio
CampanaroReInit2SperimentaleDopoAvvioAzzerando
figure
plot(mean(erroreTag),'k','LineWidth',1)
hold on

FedEKFconPruningConKmistoSenzaDoppioniSperDopoAvvioSwitch
plot(mean(erroreTag),'r--','LineWidth',1)
xlabel('Simulation step')
ylabel('cm')
grid on
% title(['Robot ' num2str(robotScelto) ', sigmaD = ',num2str(sigmaDistanza)])

if robotScelto == 2
    plot(342*[1 1],[0 250],'k-.','LineWidth',1)
end

legend('Reinit. filter','Mod. filter')
