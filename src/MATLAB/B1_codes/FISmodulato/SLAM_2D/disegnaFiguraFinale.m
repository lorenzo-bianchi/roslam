figure
subplot(2,1,1)
plot(erroreTraiettoria,'LineWidth',1)
xlabel('Passo simulazione')
ylabel('Errore assoluto robot [cm]')
grid on
axis([0 500 0 11])
subplot(2,1,2)
plot(erroreTag','LineWidth',1)
xlabel('Passo simulazione')
ylabel('Errore assoluto landmark [cm]')
grid on
axis([0 500 0 200])
% set(gca, 'YScale', 'log')