% Coordinate di tutti i robot (compreso il robot 1)
% xRHat0
% yRHat0

% Coordinate di tutti i landmark
% xHatTag0(indTag)
% yHatTag0(indTag)

% Misure di distanza disponibili
% misuraRangeTraRobot e' nRobot x nRobot ed e' riempita solo la diag superiore
% misuraRangeTagRobot e' nRobot x nTag

% Eseguire ottimizzazione su coordinate in base alle distanze disponibili e
% riassegnare i valori nel vettore di stima iniziale

XRL0 = [xRHat0(3:nRobot); yRHat0(3:nRobot); xHatTag0; yHatTag0; alfaR1R2];
paramCostanti = [xR1; yR1; xR2; yR2];

if DISEGNA
    disegna0
    DeltaIniz = funRobTag(XRL0,paramCostanti,misuraRangeTraRobot,misuraRangeTagRobot);
    norm(DeltaIniz)
end

opts1 = optimset('Display','off');
XRL0fin = lsqnonlin(@(x) funRobTag(x,paramCostanti,misuraRangeTraRobot,misuraRangeTagRobot),XRL0,-Inf*ones(numel(XRL0),1),Inf*ones(numel(XRL0),1),opts1);
% XRL0fin = XRL0; % Scommentare questa riga e commentare la precedente se
% non si vuole fare l'ottimizzazione iniziale

nRob3 = nRobot - 2;
xR2Hat0fin = xR1 + XRL0fin(end)*(xR2-xR1);
yR2Hat0fin = yR1 + XRL0fin(end)*(yR2-yR1);
xRHat0fin = [xR1; xR2Hat0fin; XRL0fin(1:nRob3)]; 
yRHat0fin = [yR1; yR2Hat0fin; XRL0fin(nRob3+1:2*nRob3)];
xHatTag0fin = XRL0fin(2*nRob3+1:2*nRob3+nTag);
yHatTag0fin = XRL0fin(2*nRob3+nTag+1:2*nRob3+2*nTag);

if DISEGNA
    disegna0fin
    DeltaFin = funRobTag(XRL0fin,paramCostanti,misuraRangeTraRobot,misuraRangeTagRobot);
    norm(DeltaFin)
    pause
    close all
end