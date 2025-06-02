if ((min(min(unaSolaIpo)) == 1) && (mod(k+1,Nstep) == 1 || (Nstep == 1)))

    % k
    % disp('fatta condivisione')
    % pause
    kCond(k) = 1;
    condivisione = 1;

    % Calcolo distanze medie inter-tag 
    indice = 0;
    for indTag = 1:nTag-1
        for jndTag = indTag+1:nTag
            indice = indice + 1;
            for indRobot = 1:nRobot
                DeltaXIJrobot = robots(indRobot).xHatTagStoria(indTag,k+1)-robots(indRobot).xHatTagStoria(jndTag,k+1);
                DeltaYIJrobot = robots(indRobot).yHatTagStoria(indTag,k+1)-robots(indRobot).yHatTagStoria(jndTag,k+1);
                distanzaIJvett(indRobot) = sqrt(DeltaXIJrobot^2+DeltaYIJrobot^2);                 
            end
            distanzaIJmedia(indice) = mean(distanzaIJvett);
        end
    end

    for indRobot = 1:nRobot
        
        % Per riportare la situazione prima di un passo di correzione
        robots(indRobot).xHatSLAMmeno = robots(indRobot).xHatSLAM(:,k+1);
        Pmeno = robots(indRobot).P;

        Hdist = zeros(nDistanze,3+(3+nPhi)*nTag); % Jacobiana delle misure di distanza
        innovazioneDist = zeros(nDistanze,1);

        indiceMisura = 0;
        for indTag = 1:nTag-1
            x_i = robots(indRobot).xHatSLAMmeno(4+(3+nPhi)*(indTag-1));
            y_i = robots(indRobot).xHatSLAMmeno(5+(3+nPhi)*(indTag-1));
            rho_i = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1));
            [maxx ipoMiglioreTag_i] = max(robots(indRobot).pesi(indTag,:));
            phi_i = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1)+ipoMiglioreTag_i);
            cosPhi_i = cos(phi_i);
            sinPhi_i = sin(phi_i);
            xTag_i = x_i + rho_i*cosPhi_i;
            yTag_i = y_i + rho_i*sinPhi_i;
            for jndTag = indTag+1:nTag
                x_j = robots(indRobot).xHatSLAMmeno(4+(3+nPhi)*(jndTag-1));
                y_j = robots(indRobot).xHatSLAMmeno(5+(3+nPhi)*(jndTag-1));
                rho_j = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(jndTag-1));
                [maxx ipoMiglioreTag_j] = max(robots(indRobot).pesi(jndTag,:));
                phi_j = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(jndTag-1)+ipoMiglioreTag_j);
                cosPhi_j = cos(phi_j);
                sinPhi_j = sin(phi_j);
                xTag_j = x_j + rho_j*cosPhi_j;
                yTag_j = y_j + rho_j*sinPhi_j;
                distanzaStimataIJ = sqrt((xTag_i-xTag_j)^2+(yTag_i-yTag_j)^2);
                indiceMisura = indiceMisura + 1;
                innovazioneDist(indiceMisura) = distanzaIJmedia(indiceMisura) - distanzaStimataIJ;
                Hdist(indiceMisura,3+(3+nPhi)*(indTag-1)+1) = xTag_i - xTag_j;
                Hdist(indiceMisura,3+(3+nPhi)*(indTag-1)+2) = yTag_i - yTag_j;
                Hdist(indiceMisura,3+(3+nPhi)*(indTag-1)+3) = (xTag_i - xTag_j)*cosPhi_i + (yTag_i - yTag_j)*sinPhi_i;
                Hdist(indiceMisura,6+(3+nPhi)*(indTag-1)+ipoMiglioreTag_i) = rho_i*(-(xTag_i - xTag_j)*sinPhi_i + (yTag_i - yTag_j)*cosPhi_i);
                Hdist(indiceMisura,3+(3+nPhi)*(jndTag-1)+1) = -(xTag_i - xTag_j);
                Hdist(indiceMisura,3+(3+nPhi)*(jndTag-1)+2) = -(yTag_i - yTag_j);
                Hdist(indiceMisura,3+(3+nPhi)*(jndTag-1)+3) = -((xTag_i - xTag_j)*cosPhi_j + (yTag_i - yTag_j)*sinPhi_j);
                Hdist(indiceMisura,6+(3+nPhi)*(jndTag-1)+ipoMiglioreTag_j) = -rho_j*(-(xTag_i - xTag_j)*sinPhi_j + (yTag_i - yTag_j)*cosPhi_j);
            
                Hdist(indiceMisura,:) = Hdist(indiceMisura,:)/distanzaStimataIJ;

            end
        end

        RsDist = stdMisuraMedia^2*eye(nDistanze);

        % Aggiornamento stima (stima a posteriori)
        KalmanGain = Pmeno*Hdist'*pinv(Hdist*Pmeno*Hdist'+RsDist);
        robots(indRobot).xHatSLAM(:,k+1) = robots(indRobot).xHatSLAMmeno + KalmanGain*innovazioneDist;
        robots(indRobot).P = (eye(3+(3+nPhi)*nTag) - KalmanGain*Hdist)*Pmeno;

    end
end