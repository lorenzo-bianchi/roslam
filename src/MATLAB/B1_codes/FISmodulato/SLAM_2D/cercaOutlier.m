% erroreAssolutoRobot = 0;
mediaInn = 0;
dati
indSim = 1;
% while erroreAssolutoRobot < 10
while abs(mediaInn) <= 10
    percorsoRandom
    CampanaroReInit2
    indSim = indSim + 1
end


figure
plot(storiaInnovazione,'.-')

disp('Errori stima posizione dei tag:')
erroriAssolutiTag
disp('Errore assoluto stima posizione robot:')
erroreAssolutoRobot

