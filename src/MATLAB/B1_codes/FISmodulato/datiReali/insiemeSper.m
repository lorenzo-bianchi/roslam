passoInizioPruning = 0;

load roslam_data3.mat

IE = 200; % inizio effettivo, cioe' e' il passo da cui far partire la simulazione (da scegliere pari)
% kSwitch = 500;

nPhi = 8;
nPassi = 1654-IE;
% robotScelto = 1; % e' il robot dei tre che si vuole testare
% DISEGNA = 1; % mettere 1 per vedere figure, 0 se non si vuole disegnare nulla
displayErrori = 1;

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]*100;
nTag = 4;

dimStato = 3+(3+nPhi)*nTag;
nIpoTag = nPhi*ones(nTag,1);
pesi = ones(nTag,nPhi)/nPhi;

numeroIpoTag = zeros(nTag,nPassi);

KRvera = 0.01;%0.01;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
% ERRORE NELLA LORO STIMA
KR = KRvera;%*1.01;
KL = KLvera;%*1.01;

Nstep = 2;
    
filtraMISURE = 0; % con filtraMisure conviene scegliere sigmaDistanza = 2, altrimenti 15
% sigmaDistanza = 20; % std in cm della misura di range
sigmaDistanzaModello = sigmaDistanza; % Per prevedere anche un'incertezza sulla deviazione standard dell'errore di misura
% bias = 10;

dVera = roslam_data.wheels_separation*100;
d = dVera;

odoRuote = roslam_data.wheels_odometry{robotScelto};
if robotScelto == 1
    xVett(1) =  0.0905*100;
    yVett(1) = 2.6885*100;
    thetaVett(1) = 0; % e' gia' zero ma giusto per chiarezza
    % cosi' hanno tutti lo stesso numero di elementi e i tempi molto vicini
    uRe = 100*odoRuote(IE+29:1654+28,2);
    uLe = 100*odoRuote(IE+29:1654+28,3);
end
if robotScelto == 2
    xVett(1) =  0.0605*100;
    yVett(1) = 1.6437*100;
    thetaVett(1) = -pi/2;
    uRe = 100*odoRuote(IE+21:1654+20,2);
    uLe = 100*odoRuote(IE+21:1654+20,3);
end
if robotScelto == 3
    xVett(1) =  1.7309*100;
    yVett(1) = 0.7688*100;
    thetaVett(1) = pi/2;
    uRe = 100*odoRuote(IE+21:1654+20,2);
    uLe = 100*odoRuote(IE+21:1654+20,3);
end

for k = 1:nPassi-1
    deltaRho = (uRe(k)+uLe(k))/2;
    deltaTheta = (uRe(k)-uLe(k))/dVera;
    xVett(k+1) = xVett(k) + deltaRho*cos(thetaVett(k));
    yVett(k+1) = yVett(k) + deltaRho*sin(thetaVett(k));
    thetaVett(k+1) = thetaVett(k) + deltaTheta;
end

possibiliPhi = linspace(-pi+2*pi/nPhi,pi,nPhi);
sigmaPhi = 2*pi/(1.5*nPhi);%pi/(3*nPhi);
pesiNuovi = zeros(nTag,nPhi);
xHatSLAM = zeros(dimStato,nPassi); % x = [xr,yr,thetar,{x1,y1,rho1,phi11,phi12,phi1H} x nTag]
P = zeros(dimStato,dimStato);
Probot = zeros(3,3);
Ptag = diag([0,0,sigmaDistanzaModello^2,sigmaPhi^2*ones(1,nPhi)]);

xHatTagStoria = zeros(nTag,nPassi);
yHatTagStoria = zeros(nTag,nPassi);

storiaInnovazione = zeros(nTag,nPassi);
innovazioneMinTag = zeros(4,1);

% Ogni colonna ha misure: Rscelto-Lj
misureTutte = zeros(nTag,nPassi/2);
% Ri-Lj
matrice = roslam_data.uwb_anchors_distances{robotScelto,1};
if robotScelto == 1
    for jndTag = 1:nTag
        misureTutte(jndTag,:) = 100*matrice(IE/2+10:end,jndTag+1);
    end
    % misureTutte(1:2,:) = misureTutte(1:2,:) + 5; % correzione bias
    % misureTutte(3:4,:) = misureTutte(3:4,:) + 10; % correzione bias
else
    for jndTag = 1:nTag
        misureTutte(jndTag,:) = 100*matrice(IE/2+1:end,jndTag+1);
    end
    % misureTutte(1:3,:) = misureTutte(1:3,:) + 10; % correzione bias
    % misureTutte(4,:) = misureTutte(4,:) + 15; % correzione bias     
end
misureTutte = misureTutte + bias; % correzione bias    

% Perturbazione manuale
% misureTutte(1,1:30) = misureTutte(1,1:30) + 100;

if filtraMISURE
    filtraMisureTutteUnRobot
end

% Inizializzazione
% Misure iniziali
kMisura = 1;
misureRange = misureTutte(:,kMisura);
% xHatSLAM viene inizializzato senza perdita di generalita' supponendo di fissare il sistema di riferimento del robot stimato 
% (che dovrebbe essere in (0,0,0) all'inizio) coincidente con quello vero
xHatSLAM(1:3,1) = [xVett(1); yVett(1); thetaVett(1)]';
xHatSLAM(4:nPhi+3:end,1) = xHatSLAM(1,1);
xHatSLAM(5:nPhi+3:end,1) = xHatSLAM(2,1);
xHatSLAM(6:nPhi+3:end,1) = misureRange;
for jndPhi = 1:nPhi
    xHatSLAM(6+jndPhi:nPhi+3:end,1) = possibiliPhi(jndPhi);
end
for indTag = 1:nTag    
    P(4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag,4+(3+nPhi)*(indTag-1):3+(3+nPhi)*indTag) = Ptag;
end

reinizializzaVett = zeros(nTag,nPassi); % avra' degli uni quando si reinizializza un tag
KreInit = ones(4,1);

T0 = clock;
for k = 1:nPassi-1

    % if k > 1000 && mod(k,10) == 1
    %     pause
    % end

    if mod(k+1,Nstep) == 1 || (Nstep == 1) 
        kMisura = kMisura + 1;
    end
    
    % PASSO DI PREDIZIONE
    uk = ((uRe(k)+uLe(k))/2);
    omegak = ((uRe(k)-uLe(k))/d);
    cosk = cos(xHatSLAM(3,k));
    sink = sin(xHatSLAM(3,k));

    % Nella predizione gli unici elementi che cambiano sono le coordinate
    % del robot in posizione 1, 2 e 3 del vettore xHatSLAM
    xHatSLAMmeno = xHatSLAM(1:dimStato,k);
    xHatSLAMmeno(1) = xHatSLAMmeno(1) + uk*cosk;
    xHatSLAMmeno(2) = xHatSLAMmeno(2) + uk*sink;
    xHatSLAMmeno(3) = xHatSLAMmeno(3) + omegak;

    % Aggiornamento degli elementi variabili della jacobiana F = df/dx
    F = eye(dimStato);
    F(1,3) = -uk*sink;
    F(2,3) = uk*cosk;
    
    % Jacobiana W = df/dw
    W = zeros(dimStato,2);
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
        misureRange = zeros(nTag,1);
        for indTag = 1:nTag
            % Se e' NaN usa la misura precedente
            if ~isnan(misureTutte(indTag,kMisura))
                misureRange(indTag) = misureTutte(indTag,kMisura);
            else
                misureRange(indTag) = misureTutte(indTag,kMisura-1);
            end
        end        
        
        xTemp = xHatSLAMmeno;
        Ptemp = Pmeno;
    
        % Prima correggo la posa del robot e le tre coordinate comuni (xi,yi,rhoi) dei 
        % landmark sulla base di tutte le ipotesi dei vari landmark       
        for indTag = 1:nTag

            % Stima a priori posizione robot
            x_r = xTemp(1);
            y_r = xTemp(2);

            innovazioniIpo = zeros(nIpoTag(indTag),1);
            
            H = zeros(nIpoTag(indTag),dimStato); % Jacobiana delle misure di range dh/dx
            Rs = zeros(nIpoTag(indTag),nIpoTag(indTag));     
            % In queste righe si calcola la stima a priori della posizione
            % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
            % da questa posizione, la sua probabilita' e la corrispondente
            % riga della jacobiana H
            indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1)); % in realta' e' l'indice dell'elemento precedente
            x_i = xTemp(indiceStartIndTag+1);
            y_i = xTemp(indiceStartIndTag+2);
            rho_i = xTemp(indiceStartIndTag+3);
            probMisura_ij = zeros(nIpoTag(indTag),1);
            for jndPhi = 1:nIpoTag(indTag)
                phi_ij = xTemp(indiceStartIndTag+3+jndPhi); % era 6+(3+nPhi)*(indTag-1)+jndPhi
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                deltaMisura_ij = misureRange(indTag)-misuraRange_ij; % innovazione
                probMisura_ij(jndPhi) = exp(-deltaMisura_ij^2/(2*sigmaDistanzaModello^2));
                pesiNuovi(indTag,jndPhi) = pesi(indTag,jndPhi)*probMisura_ij(jndPhi);
                H(jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                H(jndPhi,indiceStartIndTag+1) = xTag_ij-x_r;
                H(jndPhi,indiceStartIndTag+2) = yTag_ij-y_r;
                H(jndPhi,indiceStartIndTag+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                H(jndPhi,indiceStartIndTag+3+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                H(jndPhi,:) = H(jndPhi,:)/misuraRange_ij;
                innovazioniIpo(jndPhi) = deltaMisura_ij;
            end
            lambda_ij = probMisura_ij/sum(probMisura_ij);
            [minimo indMin] = min(abs(innovazioniIpo));
            innovazioneMinTag(indTag) = innovazioniIpo(indMin);

            % Matrice covarianza misure con formula che tiene conto
            % dell'Information Sharing
            for jndPhi = 1:nIpoTag(indTag)
                 Rs(jndPhi,jndPhi) = sigmaDistanzaModello^2/(max(.0001,lambda_ij(jndPhi)));
            end


            % Aggiornamento stima
            for jndPhi = 1:nIpoTag(indTag)
                phi_ij = xTemp(indiceStartIndTag+3+jndPhi); % era 6+(3+nPhi)*(indTag-1)+jndPhi
                cosPhi_ij = cos(phi_ij);
                sinPhi_ij = sin(phi_ij);
                xTag_ij = x_i + rho_i*cosPhi_ij;
                yTag_ij = y_i + rho_i*sinPhi_ij;
                misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                innovazione = misureRange(indTag)-misuraRange_ij;
                indiceIpoTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
                indiciDaUsare = [1:3 indiceIpoTag+1:indiceIpoTag+3 indiceIpoTag+3+jndPhi];
                PdaUsare = Ptemp(indiciDaUsare,indiciDaUsare);
                Htemp = H(jndPhi,indiciDaUsare);
                KalmanGainIS = PdaUsare*Htemp'*pinv(Htemp*PdaUsare*Htemp'+Rs(jndPhi,jndPhi));
                KalmanGain = zeros(dimStato,1);
                KalmanGain(1:3) = KalmanGainIS(1:3);
                KalmanGain(indiceIpoTag+1:indiceIpoTag+3) = KalmanGainIS(4:6);
                if k > KreInit(indTag) + kSwitch-1 
                    KalmanGain(indiceIpoTag+3+jndPhi) = KalmanGainIS(7);
                end
                xTemp = xTemp + KalmanGain*innovazione;
                Ptemp = (eye(dimStato) - KalmanGain*H(jndPhi,:))*Ptemp*(eye(dimStato) - (KalmanGain*H(jndPhi,:))') + KalmanGain*Rs(jndPhi,jndPhi)*KalmanGain';    
            end            

        end

        storiaInnovazione(:,k+1) = innovazioneMinTag;

        % Ora correggo le ipotesi dei vari landmark (solo all'inizio)
        for indTag = 1:nTag
    
            if k <= KreInit(indTag) + kSwitch-1

                % Stima a priori posizione robot
                x_r = xTemp(1);
                y_r = xTemp(2);
                
                H = zeros(nIpoTag(indTag),dimStato); % Jacobiana delle misure di range dh/dx
     
                % In queste righe si calcola la stima a priori della posizione
                % dell'ipotesi j-esima del landmark i-esimo, quindi la misura attesa
                % da questa posizione, la sua probabilita' e la corrispondente
                % riga della jacobiana H
                indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1)); % in realta' e' l'indice dell'elemento precedente
                x_i = xTemp(indiceStartIndTag+1);
                y_i = xTemp(indiceStartIndTag+2);
                rho_i = xTemp(indiceStartIndTag+3);
    
                % Matrice covarianza misure con formula che tiene conto
                % dell'Information Sharing
                for jndPhi = 1:nIpoTag(indTag)
                     Rs(jndPhi,jndPhi) = sigmaDistanzaModello^2;
                end
    
                % Aggiornamento stima
                for jndPhi = 1:nIpoTag(indTag)
                    phi_ij = xTemp(indiceStartIndTag+3+jndPhi); % era 6+(3+nPhi)*(indTag-1)+jndPhi
                    cosPhi_ij = cos(phi_ij);
                    sinPhi_ij = sin(phi_ij);
                    xTag_ij = x_i + rho_i*cosPhi_ij;
                    yTag_ij = y_i + rho_i*sinPhi_ij;
                    misuraRange_ij = sqrt((xTag_ij-x_r)^2+(yTag_ij-y_r)^2);
                    innovazione = misureRange(indTag)-misuraRange_ij;
                    H(jndPhi,1:2) = [x_r-xTag_ij, y_r-yTag_ij];
                    H(jndPhi,indiceStartIndTag+1) = xTag_ij-x_r;
                    H(jndPhi,indiceStartIndTag+2) = yTag_ij-y_r;
                    H(jndPhi,indiceStartIndTag+3) = (xTag_ij-x_r)*cosPhi_ij+(yTag_ij-y_r)*sinPhi_ij;
                    H(jndPhi,indiceStartIndTag+3+jndPhi)= ((x_r-xTag_ij)*sinPhi_ij+(yTag_ij-y_r)*cosPhi_ij)*rho_i;
                    H(jndPhi,:) = H(jndPhi,:)/misuraRange_ij;
                    indiceIpoTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
                    indiciDaUsare = [1:3 indiceIpoTag+1:indiceIpoTag+3 indiceIpoTag+3+jndPhi];
                    PdaUsare = Ptemp(indiciDaUsare,indiciDaUsare);
                    Htemp = H(jndPhi,indiciDaUsare);
                    KalmanGainC = PdaUsare*Htemp'*pinv(Htemp*PdaUsare*Htemp'+Rs(jndPhi,jndPhi));
                    KalmanGain = zeros(dimStato,1);
                    KalmanGain(indiceIpoTag+3+jndPhi) = KalmanGainC(end);
                    xTemp = xTemp + KalmanGain*innovazione;
                    Ptemp = (eye(dimStato) - KalmanGain*H(jndPhi,:))*Ptemp*(eye(dimStato) - (KalmanGain*H(jndPhi,:))') + KalmanGain*Rs(jndPhi,jndPhi)*KalmanGain';    
                end            
            end
        end

        % for indTag = 1:nTag
        %     indStart = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1)); % in realta' e' l'indice dell'elemento precedente
        %     Ploc = P(indStart+4:indStart+3+nIpoTag(indTag),indStart+4:indStart+3+nIpoTag(indTag));
        %     Plac = Ploc - diag(Ploc).*eye(nIpoTag(indTag));
        %     k
        %     indTag
        %     nIpoTag(indTag)
        %     sqrt(sum(sum(Plac.^2)))
        %     pause
        % end

        xHatSLAM(1:dimStato,k+1) = xTemp;
        P = Ptemp;

        % Aggiornamento pesi
        for indTag = 1:nTag
            pesi(indTag,1:nIpoTag(indTag)) = pesiNuovi(indTag,1:nIpoTag(indTag))/sum(pesiNuovi(indTag,1:nIpoTag(indTag)));
        end

        % Pruning ipotesi poco probabili
        for indTag = 1:nTag
            if (k > passoInizioPruning) && (nIpoTag(indTag) > 1)
                pesiPrec = pesi(indTag,1:nIpoTag(indTag));
                massimoPesoIndTag = max(pesiPrec);
                nIpoTagPrec = nIpoTag(indTag);
                indiceBase = 3 + 3*(indTag-1) + sum(nIpoTag(1:indTag-1));
                indiceIpotesi = 1;
                for jndPhi = 1:nIpoTagPrec
                    if (pesiPrec(jndPhi) < .0001*massimoPesoIndTag)
                        % disp('eliminata')
                        % pause
                        indiceVettore = indiceBase + 3 + indiceIpotesi;
                        xHatSLAM(indiceVettore:dimStato-1,k+1) = xHatSLAM(indiceVettore+1:dimStato,k+1);
                        P = [P(1:indiceVettore-1,1:indiceVettore-1) P(1:indiceVettore-1,indiceVettore+1:dimStato); ...
                            P(indiceVettore+1:dimStato,1:indiceVettore-1) P(indiceVettore+1:dimStato,indiceVettore+1:dimStato)];
                        pesi(indTag,indiceIpotesi:nIpoTag(indTag)-1) = pesi(indTag,indiceIpotesi+1:nIpoTag(indTag));
                        dimStato = dimStato - 1;
                        nIpoTag(indTag) = nIpoTag(indTag) - 1;
                        % if nIpoTag(indTag) == 0
                        %     disp('0')
                        %     pause
                        % end
                    else
                        indiceIpotesi = indiceIpotesi + 1;
                    end
                end
                sommaPesi = sum(pesi(indTag,1:nIpoTag(indTag)));
                pesi(indTag,1:nIpoTag(indTag)) = pesi(indTag,1:nIpoTag(indTag))/sommaPesi;
            end
        end

        % Pruning ipotesi simili
        for indTag = 1:nTag
            if (k > passoInizioPruning) && (nIpoTag(indTag) > 1)
                pesiPrec = pesi(indTag,1:nIpoTag(indTag));
                indiceBase = 3 + 3*(indTag-1) + sum(nIpoTag(1:indTag-1));
                [massimoPesoIndTag indMax] = max(pesiPrec);
                nIpoTagPrec = nIpoTag(indTag);
                phiVett = xHatSLAM(indiceBase+3+1:indiceBase+3+nIpoTagPrec,k+1);
                deltaPhi = phiVett-phiVett(indMax);
                deltaPhi = atan2(sin(deltaPhi),cos(deltaPhi));
                indiceIpotesi = 1;
                for jndPhi = 1:nIpoTagPrec
                    if abs(deltaPhi(jndPhi))<.001 && jndPhi ~= indMax
                        % disp('eliminata')
                        % pause
                        indiceVettore = indiceBase + 3 + indiceIpotesi;
                        xHatSLAM(indiceVettore:dimStato-1,k+1) = xHatSLAM(indiceVettore+1:dimStato,k+1);
                        P = [P(1:indiceVettore-1,1:indiceVettore-1) P(1:indiceVettore-1,indiceVettore+1:dimStato); ...
                            P(indiceVettore+1:dimStato,1:indiceVettore-1) P(indiceVettore+1:dimStato,indiceVettore+1:dimStato)];
                        pesi(indTag,indiceIpotesi:nIpoTag(indTag)-1) = pesi(indTag,indiceIpotesi+1:nIpoTag(indTag));
                        dimStato = dimStato - 1;
                        nIpoTag(indTag) = nIpoTag(indTag) - 1;
                        % if nIpoTag(indTag) == 0
                        %     disp('0')
                        %     pause
                        % end
                    else
                        indiceIpotesi = indiceIpotesi + 1;
                    end
                end
                sommaPesi = sum(pesi(indTag,1:nIpoTag(indTag)));
                pesi(indTag,1:nIpoTag(indTag)) = pesi(indTag,1:nIpoTag(indTag))/sommaPesi;
            end
        end


        % Reinizializzazione tag
        for indTag = 1:nTag
            passoMin = max(KreInit(indTag),k-199);
            mediaInn = mean(storiaInnovazione(indTag,passoMin:k+1));
            if abs(mediaInn) > 10 && k > KreInit(indTag)+199
                KreInit(indTag) = k;
                reinizializzaVett(indTag,k+1) = 1;
                k
                % indTag
                disp(['Reinizializzo tag ' num2str(indTag)])
                % pause
                % ReinizializzoTag
                dimStatoNuovo = dimStato + nPhi - nIpoTag(indTag);
                indiceBase = 3 + 3*(indTag-1) + sum(nIpoTag(1:indTag-1));
                indiceFinale = indiceBase+3+nIpoTag(indTag);
                % devo 'riallargare' vettore xHatSLAM, P e i pesi riempiendoli
                % con variabili di reinizializzazio di tutte le nPhi ipotesi:
                xHatSLAM(indiceBase+3+nPhi+1:dimStatoNuovo,k+1) = xHatSLAM(indiceFinale+1:dimStato,k+1);
                xHatSLAM(indiceBase+1,k+1) = xHatSLAM(1,k+1);
                xHatSLAM(indiceBase+2,k+1) = xHatSLAM(2,k+1);
                xHatSLAM(indiceBase+3,1) = misureRange(indTag);
                for jndPhi = 1:nPhi
                    xHatSLAM(indiceBase+3+jndPhi,1)= possibiliPhi(jndPhi);
                end
                PtagReInit = Ptag;
                PtagReInit(1:2,1:2) = P(1:2,1:2); 
                P = [P(1:indiceBase,1:indiceBase)              zeros(indiceBase,3+nPhi)                 P(1:indiceBase,indiceFinale+1:dimStato);
                     zeros(3+nPhi,indiceBase)                  PtagReInit                               zeros(3+nPhi,dimStato-indiceFinale);
                     P(1:indiceBase,indiceFinale+1:dimStato)'  zeros(dimStato-indiceFinale,3+nPhi)      P(indiceFinale+1:dimStato,indiceFinale+1:dimStato)];
                nIpoTag(indTag) = nPhi;
                dimStato = dimStatoNuovo;
            end
        end          

    else 

        % Se non ho misure confermo la stima a priori
        xHatSLAM(1:dimStato,k+1) = xHatSLAMmeno;
        P = Pmeno;

    end

    numeroIpoTag(:,k+1) = nIpoTag;

    if DISEGNA && mod(k,5) == 1
        figure(1)
        hold off
        disegnaFigSper
        title(num2str(k))
        plot(xVett(max(1,k-30):k+1)/100,yVett(max(1,k-30):k+1)/100,'k','LineWidth',2)
        plot(xVett(k+1)/100,yVett(k+1)/100,'ko')
        % axis([0 2 -0 2])
        plot(xHatSLAM(1,k+1)/100,xHatSLAM(2,k+1)/100,'o')
        for indTag = 1:nTag
            indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
            x_i = xHatSLAM(indiceStartIndTag+1,k+1);
            y_i = xHatSLAM(indiceStartIndTag+2,k+1);
            rho_i = xHatSLAM(indiceStartIndTag+3,k+1);
            x_ti = 0;
            y_ti = 0;
            for jndPhi = 1:nIpoTag(indTag)
                phi_ij = xHatSLAM(indiceStartIndTag+3+jndPhi,k+1);
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
        % axis([0 2 0 2])
%         for indTag = 1:nTag,
%             if k<nPassi,
%                 figure(2+indTag)
%                 bar(pesi(indTag,:))
%             end
%         end
%             pause(.01)
    end

    for indTag = 1:nTag
        indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
        x_i = xHatSLAM(indiceStartIndTag+1,k+1);
        y_i = xHatSLAM(indiceStartIndTag+2,k+1);
        rho_i = xHatSLAM(indiceStartIndTag+3,k+1);
        x_ti = 0;
        y_ti = 0;
        for jndPhi = 1:nIpoTag(indTag)
            phi_ij = xHatSLAM(indiceStartIndTag+3+jndPhi,k+1);
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

distanzeFinaliRobotLandmark = 100*[1.14, 2.27, 3.20, 2.59; 2.05, 2.01, 2.02, 2.21; 2.59, 2.59, 1.52, 1.68];
xCentri = cTag(:,1);
yCentri = cTag(:,2);
raggiCerchi = distanzeFinaliRobotLandmark(robotScelto,:)';
[xRobotFinale,yRobotFinale,dummy] = trovaPosTag(xCentri,yCentri,raggiCerchi);
erroreAssolutoFinaleRobotVero = norm([xRobotFinale,yRobotFinale]-[xHatSLAM(1,end),xHatSLAM(2,end)]);

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

    figure
    plot(mean(erroreTag),'.--')
    xlabel('Passo simulazione')
    ylabel('Errore assoluto robot [cm]')
    grid on

end

if displayErrori
    % disp('************************************')
    % disp('ERRORI RELATIVI:')
    % disp('Distanze del robot dai tag vere (sopra) e stimate (sotto)')
    % [distanzeRobotVere; distanzeRobotStimate]
    % disp('Distanze tra i vari tag vere (sopra) e stimate (sotto)')
    % [distanzeInterTagVere; distanzeInterTagStimate]
    % disp('ASSOLUTI:')
    disp('Errori stima posizione dei tag:')
    erroriAssolutiTag
    disp('Errore assoluto stima posizione robot:')
    erroreAssolutoFinaleRobotVero
end

% Figurona FINALE
if DISEGNA
    figure
    disegnaFigSper
    title(num2str(k))
    plot(xRobotFinale/100,yRobotFinale/100,'ko')
    % axis([0 2 -0 2])
    plot(xHatSLAM(1,end)/100,xHatSLAM(2,end)/100,'o')
    for indTag = 1:nTag
        indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
        x_i = xHatSLAM(indiceStartIndTag+1,end);
        y_i = xHatSLAM(indiceStartIndTag+2,end);
        rho_i = xHatSLAM(indiceStartIndTag+3,end);
        x_ti = 0;
        y_ti = 0;
        for jndPhi = 1:nIpoTag(indTag)
            phi_ij = xHatSLAM(indiceStartIndTag+3+jndPhi,end);
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
end