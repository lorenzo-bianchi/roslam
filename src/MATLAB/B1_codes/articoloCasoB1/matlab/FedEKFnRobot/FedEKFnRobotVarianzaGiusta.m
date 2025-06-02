possibiliPhi = linspace(-pi+2*pi/nPhi,pi,nPhi);
sigmaPhi = 2*pi/(1.5*nPhi);%pi/(3*nPhi);
probMisura_ij = zeros(nPhi,1); % quello che viene chiamato elle_ij nell'articolo di ICRA 2010
innovazione = zeros(nTag*nPhi,1);
pesiNuovi = zeros(nTag,nPhi);
% robots(indRobot).xHatSLAM = zeros(3+(3+nPhi)*nTag,nPassi): [xr,yr,thetar,{x1,y1,rho1,phi11,phi12,phi1H} x nTag]
Probot = zeros(3,3);
Ptag = diag([0,0,sigmaDistanzaModello^2,sigmaPhi^2*ones(1,nPhi)]);

% robots(indRobot).xHatSLAMmeno = zeros(3+(3+nPhi)*nTag,1);
misureRange = zeros(nTag,1);
F = eye(3+(3+nPhi)*nTag);
W = zeros(3+(3+nPhi)*nTag,2);
H = zeros(nTag*nPhi,3+(3+nPhi)*nTag);
Rs = zeros(nTag*nPhi,nTag*nPhi);

% Inizializzazione
for indRobot = 1:nRobot
    % Misure iniziali
    misureRange = sqrt((robots(indRobot).xVett(1)-cTag(:,1)).^2+(robots(indRobot).yVett(1)-cTag(:,2)).^2) + sigmaDistanza*robots(indRobot).NNrange(:,1);
    % robots(indRobot).xHatSLAM inizializzato senza perdita di generalita' supponendo di fissare il sistema di riferimento del robot stimato 
    % (che dovrebbe essere in (0,0,0) all'inizio) coincidente con quello vero
    robots(indRobot).xHatSLAM(1:3,1) = [robots(indRobot).xVett(1); robots(indRobot).yVett(1); robots(indRobot).thetaVett(1)]';
    robots(indRobot).xHatSLAM(4:nPhi+3:end,1) = robots(indRobot).xHatSLAM(1,1);
    robots(indRobot).xHatSLAM(5:nPhi+3:end,1) = robots(indRobot).xHatSLAM(2,1);
    robots(indRobot).xHatSLAM(6:nPhi+3:end,1) = misureRange;
    for jndPhi = 1:nPhi
        robots(indRobot).xHatSLAM(6+jndPhi:nPhi+3:end,1) = possibiliPhi(jndPhi);
    end
    for indTag = 1:nTag    
        robots(indRobot).P(4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag,4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag) = Ptag;
    end
    if DISEGNA
        figure(indRobot)
    end
end

if DISEGNA && nRobot > 1
    disp('Sistema le figure e spingi invio')
    pause
end

T0 = clock;
for k = 1:nPassi-1
    
    for indRobot = 1:nRobot
    
        % PASSO DI PREDIZIONE
        uk = ((robots(indRobot).uRe(k)+robots(indRobot).uLe(k))/2);
        omegak = ((robots(indRobot).uRe(k)-robots(indRobot).uLe(k))/d);
        cosk = cos(robots(indRobot).xHatSLAM(3,k));
        sink = sin(robots(indRobot).xHatSLAM(3,k));

        % Nella predizione gli unici elementi che cambiano sono le coordinate
        % del robot in posizione 1, 2 e 3 del vettore robots(indRobot).xHatSLAM
        robots(indRobot).xHatSLAMmeno = robots(indRobot).xHatSLAM(:,k);
        robots(indRobot).xHatSLAMmeno(1) = robots(indRobot).xHatSLAMmeno(1) + uk*cosk;
        robots(indRobot).xHatSLAMmeno(2) = robots(indRobot).xHatSLAMmeno(2) + uk*sink;
        robots(indRobot).xHatSLAMmeno(3) = robots(indRobot).xHatSLAMmeno(3) + omegak;

        % Aggiornamento degli elementi variabili della jacobiana F = df/dx
        F(1,3) = -uk*sink;
        F(2,3) = uk*cosk;

        % Jacobiana W = df/dw
        W(1,1) = .5*cosk; W(1,2) = .5*cosk;
        W(2,1) = .5*sink; W(2,2) = .5*sink;
        W(3,1) = 1/d; W(3,2) = -1/d;

        % Matrice covarianza errore odometrico
        Q = diag([KR*abs(robots(indRobot).uRe(k)); KL*abs(robots(indRobot).uLe(k))]);

        % Calcolo matrice P^-
        Pmeno = F*robots(indRobot).P*F' + W*Q*W';

        % PASSO DI CORREZIONE (ha effetto solo ogni Nstep passi)
        if mod(k+1,Nstep) == 1 || (Nstep == 1) 

            % Incamero la misura di range
            misureRange = sqrt((robots(indRobot).xVett(k+1)-cTag(:,1)).^2+(robots(indRobot).yVett(k+1)-cTag(:,2)).^2) + sigmaDistanza*robots(indRobot).NNrange(:,k+1);

            H = zeros(nTag*nPhi,3+(3+nPhi)*nTag); % Jacobiana delle misure di range dh/dx

            % Stima a priori posizione robot
            x_r = robots(indRobot).xHatSLAMmeno(1);
            y_r = robots(indRobot).xHatSLAMmeno(2);

            for indTag = 1:nTag

                % In queste righe si calcola la stima a priori della posizione
                % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
                % da questa posizione, la sua probabilita' e la corrispondente
                % riga della jacobiana H
                x_i = robots(indRobot).xHatSLAMmeno(4+(3+nPhi)*(indTag-1));
                y_i = robots(indRobot).xHatSLAMmeno(5+(3+nPhi)*(indTag-1));
                rho_i = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1));
                for jndPhi = 1:nPhi
                    phi_ij = robots(indRobot).xHatSLAMmeno(6+(3+nPhi)*(indTag-1)+jndPhi);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                    deltaMisura_ij = misureRange(indTag)-misuraRange_ij;
                    indiceMisura = nPhi*(indTag-1)+jndPhi;
                    innovazione(indiceMisura) = deltaMisura_ij;
                    H(indiceMisura,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                    H(indiceMisura,3+(3+nPhi)*(indTag-1)+1) = xTag_ij-x_r;
                    H(indiceMisura,3+(3+nPhi)*(indTag-1)+2) = yTag_ij-y_r;
                    H(indiceMisura,3+(3+nPhi)*(indTag-1)+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                    H(indiceMisura,6+(3+nPhi)*(indTag-1)+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                    H(indiceMisura,:) = H(nPhi*(indTag-1)+jndPhi,:)/misuraRange_ij;
                    varianzaInnovazione = H(indiceMisura,:)*Pmeno*H(indiceMisura,:)' + sigmaDistanzaModello^2;
                    probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*varianzaInnovazione));
                    pesiNuovi(indTag,jndPhi) = robots(indRobot).pesi(indTag,jndPhi)*probMisura_ij(jndPhi);
                end
                lambda_ij = probMisura_ij/sum(probMisura_ij);

                % Matrice covarianza misure con formula che tiene conto
                % dell'Information Sharing
                for jndPhi = 1:nPhi
                    Rs(nPhi*(indTag-1)+jndPhi,nPhi*(indTag-1)+jndPhi) = sigmaDistanzaModello^2/(max(.0001,lambda_ij(jndPhi)));
                end

            end

            % Aggiornamento stima (stima a posteriori)
            KalmanGain = Pmeno*H'*pinv(H*Pmeno*H'+Rs);
            robots(indRobot).xHatSLAM(:,k+1) = robots(indRobot).xHatSLAMmeno + KalmanGain*innovazione;
            robots(indRobot).P = (eye(3+(3+nPhi)*nTag) - KalmanGain*H)*Pmeno;

            % Aggiornamento pesi
            for indTag = 1:nTag
                robots(indRobot).pesi(indTag,:) = pesiNuovi(indTag,:)/sum(pesiNuovi(indTag,:));
            end

        else 

            % Se non ho misure confermo la stima a priori
            robots(indRobot).xHatSLAM(:,k+1) = robots(indRobot).xHatSLAMmeno;
            robots(indRobot).P = Pmeno;

        end

        if DISEGNA && mod(k,5) == 1
            figure(indRobot)
            hold off
            disegnaFig
            title(num2str(k))
            plot(robots(indRobot).xVett(max(1,k-30):k+1)/100,robots(indRobot).yVett(max(1,k-30):k+1)/100,'k','LineWidth',2)
            plot(robots(indRobot).xVett(k+1)/100,robots(indRobot).yVett(k+1)/100,'ko')
            axis([0 2 -0 2])
            plot(robots(indRobot).xHatSLAM(1,k+1)/100,robots(indRobot).xHatSLAM(2,k+1)/100,'o')
            for indTag = 1:nTag
                x_i = robots(indRobot).xHatSLAM(4+(3+nPhi)*(indTag-1),k+1);
                y_i = robots(indRobot).xHatSLAM(5+(3+nPhi)*(indTag-1),k+1);
                rho_i = robots(indRobot).xHatSLAM(6+(3+nPhi)*(indTag-1),k+1);
                x_ti = 0;
                y_ti = 0;
                for jndPhi = 1:nPhi
                    phi_ij = robots(indRobot).xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k+1);
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    x_ti = x_ti + xTag_ij*robots(indRobot).pesi(indTag,jndPhi);
                    y_ti = y_ti + yTag_ij*robots(indRobot).pesi(indTag,jndPhi);
                    plot(xTag_ij/100,yTag_ij/100,'m.','MarkerSize',max(1,ceil(10*robots(indRobot).pesi(indTag,jndPhi))))
                end
                plot(x_ti/100,y_ti/100,'*') % posizione media stimata tag
            end
            axis([0 2 0 2])
    %         for indTag = 1:nTag,
    %             if k<nPassi,
    %                 figure(2+indTag)
    %                 bar(robots(indRobot).pesi(indTag,:))
    %             end
    %         end
%             pause(.01)
        end

        for indTag = 1:nTag
            x_i = robots(indRobot).xHatSLAM(4+(3+nPhi)*(indTag-1),k+1);
            y_i = robots(indRobot).xHatSLAM(5+(3+nPhi)*(indTag-1),k+1);
            rho_i = robots(indRobot).xHatSLAM(6+(3+nPhi)*(indTag-1),k+1);
            x_ti = 0;
            y_ti = 0;
            for jndPhi = 1:nPhi
                phi_ij = robots(indRobot).xHatSLAM(6+(3+nPhi)*(indTag-1)+jndPhi,k+1);
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                x_ti = x_ti + xTag_ij*robots(indRobot).pesi(indTag,jndPhi);
                y_ti = y_ti + yTag_ij*robots(indRobot).pesi(indTag,jndPhi);
            end  
            robots(indRobot).xHatTagStoria(indTag,k+1) = x_ti;
            robots(indRobot).yHatTagStoria(indTag,k+1) = y_ti;
        end
        
    end
    
    if DISEGNA && mod(k,5) == 1
        pause(.01)
    end
    
end
DeltaTsim = etime(clock,T0);
% toc

for indRobot = 1:nRobot

    % Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
    robots(indRobot).distanzeRobotVere = sqrt((robots(indRobot).xVett(end)-cTag(:,1)).^2+(robots(indRobot).yVett(end)-cTag(:,2)).^2)';
    robots(indRobot).distanzeRobotStimate = zeros(1,nTag);
    x_r = robots(indRobot).xHatSLAM(1,end);
    y_r = robots(indRobot).xHatSLAM(2,end);
    for indTag = 1:nTag
        robots(indRobot).distanzeRobotStimate(indTag) = sqrt((x_r-robots(indRobot).xHatTagStoria(indTag,end))^2+(y_r-robots(indRobot).yHatTagStoria(indTag,end))^2);
    end

    robots(indRobot).distanzeInterTagVere = zeros(1,nTag*(nTag-1)/2);
    robots(indRobot).distanzeInterTagStimate = zeros(1,nTag*(nTag-1)/2);
    indice = 0;
    for indTag = 1:nTag-1
        for jndTag = indTag+1:nTag
            indice = indice + 1;
            robots(indRobot).distanzeInterTagVere(indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
            robots(indRobot).distanzeInterTagStimate(indice) = sqrt((robots(indRobot).xHatTagStoria(indTag,end)-robots(indRobot).xHatTagStoria(jndTag,end)).^2+(robots(indRobot).yHatTagStoria(indTag,end)-robots(indRobot).yHatTagStoria(jndTag,end)).^2);
        end
    end

    % Calcolo errori assoluti (= distanza) tra posizione vera e stimata di tag e
    % robot (errore assoluto SLAM)
    for indTag = 1:nTag
        robots(indRobot).erroriAssolutiTag(indTag) = sqrt((robots(indRobot).xHatTagStoria(indTag,end)-cTag(indTag,1))^2+(robots(indRobot).yHatTagStoria(indTag,end)-cTag(indTag,2))^2);
    end
    robots(indRobot).erroreAssolutoRobot = sqrt((x_r-robots(indRobot).xVett(end))^2+(y_r-robots(indRobot).yVett(end))^2);
    robots(indRobot).erroreTraiettoria = zeros(nPassi,1);
    robots(indRobot).erroreTag = zeros(4,nPassi);

    % Calcolo errore assoluto stima posizione robot nei vari passi
    for k = 1:nPassi
        robots(indRobot).erroreTraiettoria(k) = sqrt((robots(indRobot).xVett(k)-robots(indRobot).xHatSLAM(1,k))^2+(robots(indRobot).yVett(k)-robots(indRobot).xHatSLAM(2,k))^2);
        for indTag = 1:nTag
            robots(indRobot).erroreTag(indTag,k) = sqrt((cTag(indTag,1)-robots(indRobot).xHatTagStoria(indTag,k))^2+(cTag(indTag,2)-robots(indRobot).yHatTagStoria(indTag,k))^2);
        end
    end

    if DISEGNA

        figure
        subplot(2,1,1)
        plot(robots(indRobot).erroreTraiettoria,'.--')
        title(['Robot ',num2str(indRobot)])
        xlabel('Passo simulazione')
        ylabel('Errore assoluto robot [cm]')
        grid on
        subplot(2,1,2)
        plot(robots(indRobot).erroreTag')
        title(['Robot ',num2str(indRobot)])
        xlabel('Passo simulazione')
        ylabel('Errore assoluto landmark [cm]')
        grid on

    end

    if displayErrori
        disp('************************************')
        disp(['ERRORI DEL ROBOT ' num2str(indRobot)])
        disp('RELATIVI:')
        disp('Distanze del robot dai tag vere (sopra) e stimate (sotto)')
        [robots(indRobot).distanzeRobotVere; robots(indRobot).distanzeRobotStimate]
        disp('Distanze tra i vari tag vere (sopra) e stimate (sotto)')
        [robots(indRobot).distanzeInterTagVere; robots(indRobot).distanzeInterTagStimate]
        disp('ASSOLUTI:')
        disp('Errori stima posizione dei tag:')
        robots(indRobot).erroriAssolutiTag
        disp('Errore assoluto stima posizione robot:')
        robots(indRobot).erroreAssolutoRobot
    end

end