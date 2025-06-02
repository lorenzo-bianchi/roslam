nSim = 100;
% nTag = 4;

erroreAssolutoRobotVett = zeros(nSim,1);
erroriAssolutiTagVett = zeros(nSim,1);

erroriRelativiRobotVett = zeros(nSim,1);
erroriRelativiTagVett = zeros(nSim,1);

% condivisioneVett = zeros(nSim,1);

for indSim = 1:nSim

    indSim

    main
    
    % if condivisione
    %     condivisioneVett(indSim) = 1;
    % else
    %     condivisioneVett(indSim) = NaN;
    % end

    erroreAssolutoRobotVett(indSim) = robots(1).erroreAssolutoRobot;
    erroriAssolutiTagVett(indSim) = mean(robots(1).erroriAssolutiTag);

    erroriRelativiRobotVett(indSim) = mean(abs(robots(1).distanzeRobotVere-robots(1).distanzeRobotStimate));
    erroriRelativiTagVett(indSim) = mean(abs(robots(1).distanzeInterTagVere-robots(1).distanzeInterTagStimate));

end

figure
[ordinati indiciOrdinati] = sort(erroreAssolutoRobotVett);
plot(ordinati,'.-')
grid on
hold on
% plot(ordinati.*condivisioneVett(indiciOrdinati),'k*')
xlabel('simulazione')
ylabel('errore assoluto robot')

figure
[ordinati indiciOrdinati] = sort(erroriAssolutiTagVett);
plot(ordinati,'.-')
plot(erroriAssolutiTagVett(indiciOrdinati),'.-')
grid on
hold on
% plot(ordinati.*condivisioneVett(indiciOrdinati),'k*')
xlabel('simulazione')
ylabel('errore assoluto landmark')

figure
[ordinati indiciOrdinati] = sort(erroriRelativiRobotVett);
plot(ordinati,'.-')
grid on
hold on
% plot(ordinati.*condivisioneVett(indiciOrdinati),'k*')
xlabel('simulazione')
ylabel('errore relativo robot')

figure
[ordinati indiciOrdinati] = sort(erroriRelativiTagVett);
plot(ordinati,'.-')
grid on
hold on
% plot(ordinati.*condivisioneVett(indiciOrdinati),'k*')
xlabel('simulazione')
ylabel('errore relativo landmark')

