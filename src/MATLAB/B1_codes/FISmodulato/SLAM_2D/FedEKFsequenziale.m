% FedEKF in cui nel passo di correzione correggo prima la posa del robot
% sulla base di tutte le ipotesi dei vari landmark (processati una alla
% volta) e poi correggo le ipotesi dei landmark, usando per� sempre un K
% calcolato con l'Information Sharing.

possibiliPhi = linspace(-pi+2*pi/nPhi,pi,nPhi);
sigmaPhi = 2*pi/(1.5*nPhi);%pi/(3*nPhi);
probMisura_ij = zeros(nPhi,1); % quello che viene chiamato elle_ij nell'articolo di ICRA 2010
% innovazione = zeros(nTag*nPhi,1);
pesi = (1/nPhi)*ones(nTag,nPhi);
pesiNuovi = zeros(nTag,nPhi);
dimStato = 3+(3+nPhi)*nTag;
xHatSLAM = zeros(dimStato,nPassi); % x = [xr,yr,thetar,{x1,y1,rho1,phi11,phi12,phi1H} x nTag]
P = zeros(dimStato,dimStato);
Probot = zeros(3,3);
Ptag = diag([0,0,sigmaDistanzaModello^2,sigmaPhi^2*ones(1,nPhi)]);

xHatTagStoria = zeros(nTag,nPassi);
yHatTagStoria = zeros(nTag,nPassi);

xHatSLAMmeno = zeros(dimStato,1);
misureRange = zeros(nTag,nPassi);
F = eye(dimStato);
W = zeros(dimStato,2);
% H = zeros(nTag*nPhi,dimStato);
% Rs = zeros(nTag*nPhi,nTag*nPhi);

% Inizializzazione
% Misure iniziali
misureRange(:,1) = sqrt((xVett(1)-cTag(:,1)).^2+(yVett(1)-cTag(:,2)).^2) + sigmaDistanza*NNrange(:,1);
% xHatSLAM viene inizializzato senza perdita di generalita' supponendo di fissare il sistema di riferimento del robot stimato 
% (che dovrebbe essere in (0,0,0) all'inizio) coincidente con quello vero
xHatSLAM(1:3,1) = [xVett(1); yVett(1); thetaVett(1)]';
xHatSLAM(4:nPhi+3:end,1) = xHatSLAM(1,1);
xHatSLAM(5:nPhi+3:end,1) = xHatSLAM(2,1);
xHatSLAM(6:nPhi+3:end,1) = misureRange(:,1);
for jndPhi = 1:nPhi
    xHatSLAM(6+jndPhi:nPhi+3:end,1) = possibiliPhi(jndPhi);
end
for indTag = 1:nTag    
    P(4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag,4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag) = Ptag;
end

T0 = clock;
for k = 1:nPassi-1
    
    % PASSO DI PREDIZIONE
    uk = ((uRe(k)+uLe(k))/2);
    omegak = ((uRe(k)-uLe(k))/d);
    cosk = cos(xHatSLAM(3,k));
    sink = sin(xHatSLAM(3,k));

    % Nella predizione gli unici elementi che cambiano sono le coordinate
    % del robot in posizione 1, 2 e 3 del vettore xHatSLAM
    xHatSLAMmeno = xHatSLAM(:,k);
    xHatSLAMmeno(1) = xHatSLAMmeno(1) + uk*cosk;
    xHatSLAMmeno(2) = xHatSLAMmeno(2) + uk*sink;
    xHatSLAMmeno(3) = xHatSLAMmeno(3) + omegak;

    % Aggiornamento degli elementi variabili della jacobiana F = df/dx
    F(1,3) = -uk*sink;
    F(2,3) = uk*cosk;
    
    % Jacobiana W = df/dw
    W(1,1) = .5*cosk; W(1,2) = .5*cosk;
    W(2,1) = .5*sink; W(2,2) = .5*sink;
    W(3,1) = 1/d; W(3,2) = -1/d;
    
    % Matrice covarianza errore odometrico
    Q = diag([KR*abs(uRe(k)); KL*abs(uLe(k))]);

    % Calcolo matrice P^-
    Pmeno = F*P*F' + W*Q*W';
    
    % PASSO DI CORREZIONE (ha effetto solo ogni Nstep passi)
    if mod(k+1,Nstep) == 1 || (Nstep == 1) 

        % Incamero la misura di range
        misureRange(:,k+1) = sqrt((xVett(k+1)-cTag(:,1)).^2+(yVett(k+1)-cTag(:,2)).^2) + sigmaDistanza*NNrange(:,k+1);

        xTemp = xHatSLAMmeno;
        Ptemp = Pmeno;
    
        % Prima correggo la posa del robot sulla base di tutte le ipotesi dei vari landmark
        for indTag = 1:nTag

            % Stima a priori posizione robot
            x_r = xTemp(1);
            y_r = xTemp(2);
            
            H = zeros(nPhi,dimStato); % Jacobiana delle misure di range dh/dx
            Rs = zeros(nPhi,nPhi);
    
            % In queste righe si calcola la stima a priori della posizione
            % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
            % da questa posizione, la sua probabilita' e la corrispondente
            % riga della jacobiana H
            x_i = xTemp(4+(3+nPhi)*(indTag-1));
            y_i = xTemp(5+(3+nPhi)*(indTag-1));
            rho_i = xTemp(6+(3+nPhi)*(indTag-1));
            for jndPhi = 1:nPhi
                phi_ij = xTemp(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                deltaMisura_ij = misureRange(indTag,k+1)-misuraRange_ij; % innovazione
                probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*sigmaDistanzaModello^2));
                pesiNuovi(indTag,jndPhi) = pesi(indTag,jndPhi)*probMisura_ij(jndPhi);
                H(jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                H(jndPhi,3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                H(jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                H(jndPhi,:) = H(jndPhi,:)/misuraRange_ij;
            end
            lambda_ij = probMisura_ij/sum(probMisura_ij);
            
            % Matrice covarianza misure con formula che tiene conto
            % dell'Information Sharing
            for jndPhi = 1:nPhi
                Rs(jndPhi,jndPhi) = sigmaDistanzaModello^2/(max(.0001,lambda_ij(jndPhi)));
            end

            % Aggiornamento stima (stima a posteriori)
            for jndPhi = 1:nPhi
                phi_ij = xTemp(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                innovazione = misureRange(indTag,k+1)-misuraRange_ij;
                indiceIpoTag = 3+(3+nPhi)*(indTag-1);
                indiciDaUsare = [1:3 indiceIpoTag+1:indiceIpoTag+3 indiceIpoTag+3+jndPhi];
                PdaUsare = Ptemp(indiciDaUsare,indiciDaUsare);
                Htemp = H(jndPhi,indiciDaUsare);
                KalmanGainIS = PdaUsare*Htemp'*pinv(Htemp*PdaUsare*Htemp'+Rs(jndPhi,jndPhi));
                KalmanGain = zeros(dimStato,1);
                KalmanGain(1:3) = KalmanGainIS(1:3);
                xTemp = xTemp + KalmanGain*innovazione;
                Ptemp = (eye(dimStato) - KalmanGain*H(jndPhi,:))*Ptemp*(eye(dimStato) - (KalmanGain*H(jndPhi,:))') + KalmanGain*Rs(jndPhi,jndPhi)*KalmanGain';    
            end
         end
          
        % Ora correggo le ipotesi dei vari landmark
        for indTag = 1:nTag

            % Stima a priori posizione robot
            x_r = xTemp(1);
            y_r = xTemp(2);
            
            H = zeros(nPhi,dimStato); % Jacobiana delle misure di range dh/dx
    
            % In queste righe si calcola la stima a priori della posizione
            % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
            % da questa posizione, la sua probabilita' e la corrispondente
            % riga della jacobiana H
            x_i = xTemp(4+(3+nPhi)*(indTag-1));
            y_i = xTemp(5+(3+nPhi)*(indTag-1));
            rho_i = xTemp(6+(3+nPhi)*(indTag-1));

            % Ricalcolo Rs con l'IS
            for jndPhi = 1:nPhi
                phi_ij = xTemp(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                deltaMisura_ij = misureRange(indTag,k+1)-misuraRange_ij; % innovazione
                probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*sigmaDistanzaModello^2));
                pesiNuovi(indTag,jndPhi) = pesi(indTag,jndPhi)*probMisura_ij(jndPhi);
                H(jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                H(jndPhi,3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                H(jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                H(jndPhi,:) = H(jndPhi,:)/misuraRange_ij;
            end
            lambda_ij = probMisura_ij/sum(probMisura_ij);
            
            % Matrice covarianza misure con formula che tiene conto
            % dell'Information Sharing
            for jndPhi = 1:nPhi
                Rs(jndPhi,jndPhi) = sigmaDistanzaModello^2/(max(.0001,lambda_ij(jndPhi)));
            end

             % Aggiornamento stima (stima a posteriori)
            for jndPhi = 1:nPhi
                phi_ij = xTemp(6+(3+nPhi)*(indTag-1)+jndPhi);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                innovazione = misureRange(indTag,k+1)-misuraRange_ij;
                indiceIpoTag = 3+(3+nPhi)*(indTag-1);
                indiciDaUsare = [1:3 indiceIpoTag+1:indiceIpoTag+3 indiceIpoTag+3+jndPhi];
                PdaUsare = Ptemp(indiciDaUsare,indiciDaUsare);
                H(jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                H(jndPhi,3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                H(jndPhi,3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                H(jndPhi,6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                H(jndPhi,:) = H(jndPhi,:)/misuraRange_ij;
                Htemp = H(jndPhi,indiciDaUsare);
                KalmanGainC = PdaUsare*Htemp'*pinv(Htemp*PdaUsare*Htemp'+Rs(jndPhi,jndPhi));
                KalmanGainC(1:3) = zeros(3,1);
                KalmanGain = zeros(dimStato,1);
                KalmanGain(indiciDaUsare) = KalmanGainC;
                xTemp = xTemp + KalmanGain*innovazione;
                Ptemp = (eye(dimStato) - KalmanGain*H(jndPhi,:))*Ptemp*(eye(dimStato) - (KalmanGain*H(jndPhi,:))') + KalmanGain*Rs(jndPhi,jndPhi)*KalmanGain';    
            end
        end

        xHatSLAM(:,k+1) = xTemp;
        P = Ptemp;

        % Aggiornamento pesi
        for indTag = 1:nTag
            pesi(indTag,:) = pesiNuovi(indTag,:)/sum(pesiNuovi(indTag,:));
        end
        
    else 
        
        % Se non ho misure confermo la stima a priori
        xHatSLAM(:,k+1) = xHatSLAMmeno;
        P = Pmeno;
        
    end

    if DISEGNA && mod(k,5) == 1
        figure(1)
        hold off
        disegnaFig
        title(num2str(k))
        plot(xVett(max(1,k-30):k+1)/100,yVett(max(1,k-30):k+1)/100,'k','LineWidth',2)
        plot(xVett(k+1)/100,yVett(k+1)/100,'ko')
        axis([0 2 -0 2])
        plot(xHatSLAM(1,k+1)/100,xHatSLAM(2,k+1)/100,'o')
        for indTag = 1:nTag
            x_i = xHatSLAM(4+(3+nPhi)*(indTag-1),k+1);
            y_i = xHatSLAM(5+(3+nPhi)*(indTag-1),k+1);
            rho_i = xHatSLAM(6+(3+nPhi)*(indTag-1),k+1);
            x_ti = 0;
            y_ti = 0;
            for jndPhi = 1:nPhi
                phi_ij = xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k+1);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                x_ti = x_ti + xTag_ij*pesi(indTag,jndPhi);
                y_ti = y_ti + yTag_ij*pesi(indTag,jndPhi);
                plot(xTag_ij/100,yTag_ij/100,'m.','MarkerSize',max(1,ceil(10*pesi(indTag,jndPhi))))
            end
            plot(x_ti/100,y_ti/100,'*') % posizione media stimata tag
        end
        axis([0 2 0 2])
%         for indTag = 1:nTag,
%             if k<nPassi,
%                 figure(2+indTag)
%                 bar(pesi(indTag,:))
%             end
%         end
        pause(.01)
    end
    
%     plot(P(7,8:16),'.-')
%     grid on
%     title(num2str(k))
%     pause

    for indTag = 1:nTag
        x_i = xHatSLAM(4+(3+nPhi)*(indTag-1),k+1);
        y_i = xHatSLAM(5+(3+nPhi)*(indTag-1),k+1);
        rho_i = xHatSLAM(6+(3+nPhi)*(indTag-1),k+1);
        x_ti = 0;
        y_ti = 0;
        for jndPhi = 1:nPhi
            phi_ij = xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k+1);
            cosPhi_ij = cos(phi_ij);
            sinPhi_ij = sin(phi_ij);
            xTag_ij = x_i + rho_i*cosPhi_ij;
            yTag_ij = y_i + rho_i*sinPhi_ij;
            x_ti = x_ti + xTag_ij*pesi(indTag,jndPhi);
            y_ti = y_ti + yTag_ij*pesi(indTag,jndPhi);
        end  
        xHatTagStoria(indTag,k+1) = x_ti;
        yHatTagStoria(indTag,k+1) = y_ti;
    end

end
DeltaTsim = etime(clock,T0);
% toc

% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
distanzeRobotVere = sqrt((xVett(end)-cTag(:,1)).^2+(yVett(end)-cTag(:,2)).^2)';
distanzeRobotStimate = zeros(1,nTag);
x_r = xHatSLAM(1,end);
y_r = xHatSLAM(2,end);
for indTag = 1:nTag
    distanzeRobotStimate(indTag) = sqrt((x_r-xHatTagStoria(indTag,end))^2+(y_r-yHatTagStoria(indTag,end))^2);
end

distanzeInterTagVere = zeros(1,nTag*(nTag-1)/2);
distanzeInterTagStimate = zeros(1,nTag*(nTag-1)/2);
indice = 0;
for indTag = 1:nTag-1
    for jndTag = indTag+1:nTag
        indice = indice + 1;
        distanzeInterTagVere(indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
        distanzeInterTagStimate(indice) = sqrt((xHatTagStoria(indTag,end)-xHatTagStoria(jndTag,end)).^2+(yHatTagStoria(indTag,end)-yHatTagStoria(jndTag,end)).^2);
    end
end

% Calcolo errori assoluti (= distanza) tra posizione vera e stimata di tag e
% robot (errore assoluto SLAM)
erroriAssolutiTag = zeros(1,nTag);
for indTag = 1:nTag
    erroriAssolutiTag(indTag) = sqrt((xHatTagStoria(indTag,end)-cTag(indTag,1))^2+(yHatTagStoria(indTag,end)-cTag(indTag,2))^2);
end
erroreAssolutoRobot = sqrt((x_r-xVett(end))^2+(y_r-yVett(end))^2);
erroreTraiettoria = zeros(nPassi,1);
erroreTag = zeros(4,nPassi);

% Calcolo errore assoluto stima posizione robot nei vari passi
for k = 1:nPassi
    erroreTraiettoria(k) = sqrt((xVett(k)-xHatSLAM(1,k))^2+(yVett(k)-xHatSLAM(2,k))^2);
    for indTag = 1:nTag
        erroreTag(indTag,k) = sqrt((cTag(indTag,1)-xHatTagStoria(indTag,k))^2+(cTag(indTag,2)-yHatTagStoria(indTag,k))^2);
    end
end

if DISEGNA

    figure
    subplot(2,1,1)
    plot(erroreTraiettoria,'.--')
    xlabel('Passo simulazione')
    ylabel('Errore assoluto robot [cm]')
    grid on
    subplot(2,1,2)
    plot(erroreTag')
    xlabel('Passo simulazione')
    ylabel('Errore assoluto landmark [cm]')
    grid on

end

if displayErrori
    disp('************************************')
    disp('ERRORI RELATIVI:')
    disp('Distanze del robot dai tag vere (sopra) e stimate (sotto)')
    [distanzeRobotVere; distanzeRobotStimate]
    disp('Distanze tra i vari tag vere (sopra) e stimate (sotto)')
    [distanzeInterTagVere; distanzeInterTagStimate]
    disp('ASSOLUTI:')
    disp('Errori stima posizione dei tag:')
    erroriAssolutiTag
    disp('Errore assoluto stima posizione robot:')
    erroreAssolutoRobot
end