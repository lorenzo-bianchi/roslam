nSim = 100000;
nRobot = 3;

% Da fare con 3 robot dopo aver eseguito dati e
% percorsoRandomNrobotGuidato.

nSimAllineati = 0;

for indSim = 1:nSim

    % determinaIniziSinistra; % 0.062% di avere robot allineati (calcolato facendo 100000 simulazioni)
    determinaIniziCentrale; % 0.001% di avere robot allineati

    xCentri = posIniziali(:,1);
    yCentri = posIniziali(:,2);
    [xOrd,indOrd] = sort(xCentri);
    yOrd = yCentri(indOrd);
    deltaPhiA = atan2(yOrd(2)-yOrd(1),xOrd(2)-xOrd(1));
    deltaPhiB = atan2(yOrd(3)-yOrd(2),xOrd(3)-xOrd(2));
    if abs(deltaPhiB-deltaPhiA)<10*pi/180 
        nSimAllineati = nSimAllineati + 1;
    end

end

nSimAllineati

