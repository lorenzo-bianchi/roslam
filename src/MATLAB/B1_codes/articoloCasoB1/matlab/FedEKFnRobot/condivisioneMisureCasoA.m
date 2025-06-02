if ((k>300) && (mod(k+1,Nstep) == 1 || (Nstep == 1))) % Questo k>300 andra' sostituito con una condizione sui pesi o sulla varianza della stima media

    for indRobot = 1:nRobot
        
        % Per riportare la situazione prima di un passo di correzione
        robots(indRobot).xHatSLAMmeno = robots(indRobot).xHatSLAM(:,k+1);
        Pmeno = robots(indRobot).P;

        Hx = zeros(nTag*nPhi,3+(3+nPhi)*nTag); % Jacobiana delle misure di xTag
        Hy = zeros(nTag*nPhi,3+(3+nPhi)*nTag); % Jacobiana delle misure di yTag
        innovazioneX = zeros(nTag*nPhi,1);
        innovazioneY = zeros(nTag*nPhi,1);

        for indTag = 1:nTag

            x_i = robots(indRobot).xHatSLAMmeno(4+(3+nPhi)*(indTag-1));
            y_i = robots(indRobot).xHatSLAMmeno(5+(3+nPhi)*(indTag-1));
            rho_i = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1));
            misuraX_ij = xHatTagMedia(indTag,k+1);
            misuraY_ij = yHatTagMedia(indTag,k+1);
 
            for jndPhi = 1:nPhi
                phi_ij = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                deltaMisuraX_ij = misuraX_ij - xTag_ij;
                deltaMisuraY_ij = misuraY_ij - yTag_ij;
                innovazioneX(nPhi*(indTag-1)+jndPhi) = deltaMisuraX_ij;
                innovazioneY(nPhi*(indTag-1)+jndPhi) = deltaMisuraY_ij;
%                 probMisuraX_ij(jndPhi) = exp(-deltaMisuraX_ij^2/(2*xHatTagStd(indTag,k+1)^2));
%                 probMisuraY_ij(jndPhi) = exp(-deltaMisuraY_ij^2/(2*yHatTagStd(indTag,k+1)^2));
                probMisuraX_ij(jndPhi) = exp(-deltaMisuraX_ij^2/(2*stdMisuraMedia^2));
                probMisuraY_ij(jndPhi) = exp(-deltaMisuraY_ij^2/(2*stdMisuraMedia^2));
                pesiNuovi(indTag,jndPhi) = robots(indRobot).pesi(indTag,jndPhi)*probMisuraX_ij(jndPhi)*probMisuraY_ij(jndPhi);
                Hx(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+1) = 1;
                Hy(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+2) = 1;
                Hx(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+3) = cosPhi_ij;
                Hy(nPhi*(indTag-1)+jndPhi,3+(3+nPhi)*(indTag-1)+3) = sinPhi_ij;
                Hx(nPhi*(indTag-1)+jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= -rho_i*sinPhi_ij;
                Hy(nPhi*(indTag-1)+jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= rho_i*cosPhi_ij;
            end
            lambdaX_ij = probMisuraX_ij/sum(probMisuraX_ij);
            lambdaY_ij = probMisuraY_ij/sum(probMisuraY_ij);

            % Matrice covarianza misure con formula che tiene conto
            % dell'Information Sharing
            for jndPhi = 1:nPhi
%                 RsX(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = xHatTagStd(indTag,k+1)^2/(max(.0001,lambdaX_ij(jndPhi)));
%                 RsY(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = yHatTagStd(indTag,k+1)^2/(max(.0001,lambdaY_ij(jndPhi)));
                RsX(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = stdMisuraMedia^2/(max(.0001,lambdaX_ij(jndPhi)));
                RsY(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = stdMisuraMedia^2/(max(.0001,lambdaY_ij(jndPhi)));
            end

        end

        % Aggiornamento stima (stima a posteriori)
        Htot = [Hx;Hy];
        RsTot = [RsX, zeros(nTag*nPhi); zeros(nTag*nPhi), RsY];
        innovazioneTot = [innovazioneX; innovazioneY];
        KalmanGain = Pmeno*Htot'*pinv(Htot*Pmeno*Htot'+RsTot);
        robots(indRobot).xHatSLAM(:,k+1) = robots(indRobot).xHatSLAMmeno + KalmanGain*innovazioneTot;
        robots(indRobot).P = (eye(3+(3+nPhi)*nTag) - KalmanGain*Htot)*Pmeno;

        % Aggiornamento pesi
        for indTag = 1:nTag
            robots(indRobot).pesi(indTag,:) = pesiNuovi(indTag,:)/sum(pesiNuovi(indTag,:));
        end
    end
end