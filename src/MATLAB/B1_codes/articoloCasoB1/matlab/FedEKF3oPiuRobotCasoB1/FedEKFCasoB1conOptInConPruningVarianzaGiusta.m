passoInizioPruning = 200;

possibiliPhi = linspace(-pi+2*pi/nPhi,pi,nPhi);
sigmaPhi = 2*pi/(1.5*nPhi);%pi/(3*nPhi);
rapportoFedEKF = 10^5;

stimaMediaIstanze = 0; % 1 se media delle istanze del FedEKF, 0 se istanza migliore
allineatiVett = zeros(nRobot+nTag-3,1); % Verifica se nell'inizializzazione ci sono dei robot allineati

dimStatoIniziale = 3*nPhi*nRobot+2*nTag; % Ogni robot ha nPhi ipotesi, ciascuna di 3 variabili + 2 variabili per ogni tag (unica ipotesi)
% nMisure = nchoosek(nRobot,2)*nPhi^2+nRobot*nPhi*nTag; % sum_{ij,j>i} ipoRi x ipoRj + nRobot x nPhi x nTag 

% Questa struct la lascio per il caso di nR>2 robot in cui dovrei avere due
% FedEKF per le due mappe simmetriche che si vengono a creare (con un'unica
% ipotesi per la posizione dei robot e dei landmark in ciascun FedEKF)
FedEKF = struct('xHatSLAM',zeros(dimStatoIniziale,nPassi),... %'xHatSLAMmeno',zeros(dimStatoIniziale,1),...
    'P',zeros(dimStatoIniziale,dimStatoIniziale),'pesiIpoRobot',ones(nRobot,nPhi)/nPhi,...
    'numIpoRobot',nPhi*ones(nRobot,1),'dimStato',dimStatoIniziale);

nFedEKF = 2;
FedEKFs = repmat(FedEKF,1,nFedEKF);
pesiNuoviFedEKF = ones(nFedEKF,1)/nFedEKF;
pesiFedEKF = ones(nFedEKF,1)/nFedEKF;
% pesoMin = 10^(-20); % peso sotto il quale un'istanza del FedEKF viene eliminata
indBuoniFedEKF = ones(nFedEKF,1);
nFedEKFattivi = zeros(nPassi,1);
nFedEKFattivi(1) = nFedEKF;

% Stima complessiva dei robot negli nPassi
xHatRobotStoria = zeros(nRobot,nPassi);
yHatRobotStoria = zeros(nRobot,nPassi);
% Stima complessiva landmark negli nPassi
xHatTagStoria = zeros(nTag,nPassi);
yHatTagStoria = zeros(nTag,nPassi);

% pesiNuoviIpoRobot = zeros(nRobot,nPhi);

% La Ptag ha un'espressione complicata in quanto le due ipotesi per il tag
% sono ottenute mediante l'intersezione tra tre o piu' cerchi). Per semplicita' 
% si assume che le incertezze su x e y siano indipendenti, uguali tra loro 
% e paragonabili a quella sulle misure di range (anche se in realta' dipende 
% anche da come sono messi i cerchi). Si puo' mettere un fattore maggiore di uno 
% (e.g. 10) a moltiplicare Ptag per tenere conto di queste incertezze
Ptag = 1*diag(sigmaDistanzaModello^2*ones(2,1));

% F = eye(nStato);
% W = zeros(nStato,2*nRobot);
% H = zeros(nMisure,nStato);
% innovazione = zeros(nMisure,1);
% Rs = zeros(nMisure,nMisure);
Q = zeros(2*nRobot);

misuraRangeTraRobot = zeros(nRobot,nRobot);
% Misure iniziali
for indRobot = 1:nRobot-1
    for jndRobot = indRobot+1:nRobot
        misuraRangeTraRobot(indRobot,jndRobot) = sqrt((robots(indRobot).xVett(1)-robots(jndRobot).xVett(1))^2+(robots(indRobot).yVett(1)-robots(jndRobot).yVett(1))^2) + 0.5*sigmaDistanza*(robots(indRobot).NNrangeRobot(1)+robots(jndRobot).NNrangeRobot(1));
    end
end
misuraRangeTagRobot = zeros(nRobot,nTag);
for indRobot = 1:nRobot
    for indTag = 1:nTag
        misuraRangeTagRobot(indRobot,indTag) = sqrt((robots(indRobot).xVett(1)-cTag(indTag,1)).^2+(robots(indRobot).yVett(1)-cTag(indTag,2)).^2) + sigmaDistanza*robots(indRobot).NNrangeTag(indTag,1);
    end
end

% Senza perdita di generalita' si assume che il robot 1 abbia scelto il suo
% sistema di riferimento coincidente con quello globale (questo facilita i
% grafici) per cui xR1 e yR1 sono inizializzati nella loro posizione reale
% e anche xR2 e yR2 sono nella loro posizione reale scalata pero' in base
% alla misura della distanza tra R1 e R2 (questo corrisponde per esempio al
% caso in cui R1 usa un sistema di riferimento locale in cui lui all'inizio
% sta nell'origine e R2 lo mette sul suo asse x).
xRHat0 = zeros(nRobot,1);
yRHat0 = zeros(nRobot,1);
xR1 = robots(1).xVett(1);
yR1 = robots(1).yVett(1);
xRHat0(1) = xR1;
yRHat0(1) = yR1;
xR2 = robots(2).xVett(1);
yR2 = robots(2).yVett(1);
DeltaXR1R2 = xR2-xRHat0(1);
DeltaYR1R2 = yR2-yRHat0(1);
dR1R2vera = sqrt(DeltaXR1R2^2+DeltaYR1R2^2);
alfaR1R2 = misuraRangeTraRobot(1,2)/dR1R2vera;
xRHat0(2) = xRHat0(1) + (xR2-xRHat0(1))*alfaR1R2;
yRHat0(2) = yRHat0(1) + (yR2-yRHat0(1))*alfaR1R2;
[xR3Hat0a,yR3Hat0a,xR3Hat0b,yR3Hat0b] = trovaIpoTagNumerica(xRHat0(1),yRHat0(1),xRHat0(2),yRHat0(2),misuraRangeTraRobot(1,3),misuraRangeTraRobot(2,3));

for indFedEKF = 1:nFedEKF
    % xHatSLAM = [(xR1i,yR1i,thetaR1i)x8xnRobot,(xLi,yLi)xnTag]
    
    % Robot 3:
    if indFedEKF == 1
        xRHat0(3) = xR3Hat0a;
        yRHat0(3) = yR3Hat0a;
    else
        xRHat0(3) = xR3Hat0b;
        yRHat0(3) = yR3Hat0b;
    end

    % Robot successivi al 3:
    for indRobot = 4:nRobot
        xCentri = xRHat0(1:indRobot-1);
        yCentri = yRHat0(1:indRobot-1);
        raggiCerchi = misuraRangeTraRobot(1:indRobot-1,indRobot);
        [xRHat0(indRobot),yRHat0(indRobot),allineatiVett(indRobot-3)] = trovaPosTag(xCentri,yCentri,raggiCerchi);
    end

    % Inzializzazione landmark
    xHatTag0 = zeros(nTag,1);
    yHatTag0 = zeros(nTag,1);
    xCentri = xRHat0(1:nRobot);
    yCentri = yRHat0(1:nRobot);
    for indTag = 1:nTag
        % indFedEKF
        % indTag
        raggiCerchi = misuraRangeTagRobot(:,indTag);
        [xHatTag0(indTag),yHatTag0(indTag),allineatiVett(nRobot-3+indTag)] =  trovaPosTag(xCentri,yCentri,raggiCerchi);
    end

    ottimizzaGrafo

    % Robot 1:
    FedEKFs(indFedEKF).xHatSLAM(1:3:1+3*(nPhi-1),1) = xRHat0fin(1);
    FedEKFs(indFedEKF).xHatSLAM(2:3:2+3*(nPhi-1),1) = yRHat0fin(1);
    FedEKFs(indFedEKF).xHatSLAM(3:3:3+3*(nPhi-1),1) = possibiliPhi;
    PtriplettaRobot1 = zeros(3,3);
    PtriplettaRobot1(3,3) = sigmaPhi^2;
    
    % Robot 2:
    FedEKFs(indFedEKF).xHatSLAM(3*nPhi+1:3:6*nPhi-2,1) = xRHat0fin(2);
    FedEKFs(indFedEKF).xHatSLAM(3*nPhi+2:3:6*nPhi-1,1) = yRHat0fin(2);
    FedEKFs(indFedEKF).xHatSLAM(3*nPhi+3:3:6*nPhi,1) = possibiliPhi;
    PtriplettaRobot2xy = [DeltaXR1R2^2 DeltaXR1R2*DeltaYR1R2; DeltaXR1R2*DeltaYR1R2 DeltaYR1R2^2]*sigmaDistanzaModello^2/(2*dR1R2vera^2);
    PtriplettaRobot2 = [PtriplettaRobot2xy zeros(2,1); zeros(1,2) sigmaPhi^2];
    
    % Robot 3:
    FedEKFs(indFedEKF).xHatSLAM(6*nPhi+1:3:9*nPhi-2,1) = xRHat0fin(3);
    FedEKFs(indFedEKF).xHatSLAM(6*nPhi+2:3:9*nPhi-1,1) = yRHat0fin(3);
    FedEKFs(indFedEKF).xHatSLAM(6*nPhi+3:3:9*nPhi,1) = possibiliPhi;
    % Matrici covarianza dal robot 3 in poi (da aggiustare, se non altro da dividere per due)
    PtriplettaRobotxy = 0.5*sigmaDistanzaModello^2*eye(2); % Aggiunto lo 0.5 il 21ott2024 
    PtriplettaRobot = [PtriplettaRobot2xy zeros(2,1); zeros(1,2) sigmaPhi^2];

    % Robot successivi al 3:
    for indRobot = 4:nRobot
        FedEKFs(indFedEKF).xHatSLAM(3*(indRobot-1)*nPhi+1:3:3*indRobot*nPhi-2,1) = xRHat0fin(indRobot);
        FedEKFs(indFedEKF).xHatSLAM(3*(indRobot-1)*nPhi+2:3:3*indRobot*nPhi-1,1) = yRHat0fin(indRobot);
        FedEKFs(indFedEKF).xHatSLAM(3*(indRobot-1)*nPhi+3:3:3*indRobot*nPhi,1) = possibiliPhi;
    end

    % Inizializzazione matrici covarianza robot
    for indIpoRi = 1:nPhi
        FedEKFs(indFedEKF).P(1+3*(indIpoRi-1):3*indIpoRi,1+3*(indIpoRi-1):3*indIpoRi) = PtriplettaRobot1;
        FedEKFs(indFedEKF).P(3*nPhi+1+3*(indIpoRi-1):3*nPhi+3*indIpoRi,3*nPhi+1+3*(indIpoRi-1):3*nPhi+3*indIpoRi) = PtriplettaRobot2;
        for indRobot = 3:nRobot
            indici = 3*(indRobot-1)*nPhi+1+3*(indIpoRi-1):3*(indRobot-1)*nPhi+3*indIpoRi;
            FedEKFs(indFedEKF).P(indici,indici) = PtriplettaRobot;
        end
    end

    % Inzializzazione landmark
    for indTag = 1:nTag
        indici = nRobot*3*nPhi+2*indTag-1:nRobot*3*nPhi+2*indTag;
        FedEKFs(indFedEKF).xHatSLAM(indici) = [xHatTag0fin(indTag),yHatTag0fin(indTag)]';
        FedEKFs(indFedEKF).P(indici,indici) = Ptag;
    end    

end

if sum(allineatiVett) > 0
    return
end

% Qui sarebbe da aggiustare le coordinate di tutti i landmark e robot dal 2
% in poi ottimizzando il grafo. Per il robot 2 in realt il vincolo  che
% sia sulla retta che passa per lui e per il robot 1.

if DISEGNA
    for indFedEKF = 1:nFedEKF
        figure(indFedEKF)
        figN = gcf;
        % figN.Position = [950+450*8/nPhi*(1+mod(indFedEKF-1,nPhi/2)) 500-490*8/nPhi*(indFedEKF>nPhi/2) 400*8/nPhi 400*8/nPhi]; % PC uni
        figN.Position = [100+700*(indFedEKF-1) 200 600 600]; % Portatile
    end
    % disp('Sistema le figure e spingi invio')
    % pause
end

% La parte seguente serve per memorizzare le stime al primo passo della
% simulazione nei dataset storici (cioe' xHatRobotStoria, yHatRobotStoria,
% xHatTagStoria e yHatTagStoria)
k = 0;
if stimaMediaIstanze == 1
    for indFedEKF = 1:nFedEKF
        nIpoRobot = FedEKFs(indFedEKF).numIpoRobot;
        if indBuoniFedEKF(indFedEKF)
            for indRobot = 1:nRobot
                xRi = 0;
                yRi = 0;
                for indIpoRi = 1:nIpoRobot(indRobot)
                    indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
                    xRi = xRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                    yRi = yRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                end
                xHatRobotStoria(indRobot,k+1) = xHatRobotStoria(indRobot,k+1) + xRi*pesiFedEKF(indFedEKF);
                yHatRobotStoria(indRobot,k+1) = yHatRobotStoria(indRobot,k+1) + yRi*pesiFedEKF(indFedEKF);
            end

            for indTag = 1:nTag
                indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
                xTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
                yTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
                xHatTagStoria(indTag,k+1) = xHatTagStoria(indTag,k+1) + xTag*pesiFedEKF(indFedEKF);
                yHatTagStoria(indTag,k+1) = yHatTagStoria(indTag,k+1) + yTag*pesiFedEKF(indFedEKF);
            end
        end
    end
else
    if numel(indBuoniFedEKF) > 1
        [pesoMaxFedEKF,indPesoMaxFedEKF] = max(pesiFedEKF);
    else
        indPesoMaxFedEKF = find(indBuoniFedEKF);
    end
    nIpoRobot = FedEKFs(indPesoMaxFedEKF).numIpoRobot;
    for indRobot = 1:nRobot
        xRi = 0;
        yRi = 0;
        for indIpoRi = 1:nIpoRobot(indRobot)
            indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
            xRi = xRi + FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indPesoMaxFedEKF).pesiIpoRobot(indRobot,indIpoRi);
            yRi = yRi + FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indPesoMaxFedEKF).pesiIpoRobot(indRobot,indIpoRi);
        end
        xHatRobotStoria(indRobot,k+1) = xRi;
        yHatRobotStoria(indRobot,k+1) = yRi;
    end

    for indTag = 1:nTag
        indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
        xHatTagStoria(indTag,k+1) = FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
        yHatTagStoria(indTag,k+1) = FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
    end
end


T0 = clock;
for k = 1:nPassi-1

    % k
    % pause
    
    for indFedEKF = 1:nFedEKF

        nIpoRobot = FedEKFs(indFedEKF).numIpoRobot;
        nStato = FedEKFs(indFedEKF).dimStato;

        if indBuoniFedEKF(indFedEKF)
    
            % PASSO DI PREDIZIONE

            F = eye(nStato);
            W = zeros(nStato,2*nRobot);

            % Comincio con il copiare tutti gli elementi del vettore di stima
            % perche' alcune variabili (quelle dei tag) non cambiano
            xHatSLAMmeno = FedEKFs(indFedEKF).xHatSLAM(1:nStato,k);

            % Dinamica robot da 1 a nRobot
            for indRobot = 1:nRobot
                uk = ((robots(indRobot).uRe(k)+robots(indRobot).uLe(k))/2);
                omegak = ((robots(indRobot).uRe(k)-robots(indRobot).uLe(k))/d);
        
                for indIpoRi = 1:nIpoRobot(indRobot)
                    indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1); % in realt  la fine dell'ipotesi prec
                    cosk = cos(FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+3,k)); % era 3*(indRobot-1)*nPhi+3*indIpoRi
                    sink = sin(FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+3,k));
                    xHatSLAMmeno(indiceStartIpoRi+1) = xHatSLAMmeno(indiceStartIpoRi+1) + uk*cosk;
                    xHatSLAMmeno(indiceStartIpoRi+2) = xHatSLAMmeno(indiceStartIpoRi+2) + uk*sink;
                    xHatSLAMmeno(indiceStartIpoRi+3) = xHatSLAMmeno(indiceStartIpoRi+3) + omegak;
        
                    % Aggiornamento degli elementi variabili della jacobiana F = df/dx
                    % relativi al robot 2
                    F(indiceStartIpoRi+1,indiceStartIpoRi+3) = -uk*sink;
                    F(indiceStartIpoRi+2,indiceStartIpoRi+3) = uk*cosk;
        
                    % Jacobiana W = df/dw
                    W(indiceStartIpoRi+1,2*indRobot-1) = .5*cosk; W(indiceStartIpoRi+1,2*indRobot) = .5*cosk;
                    W(indiceStartIpoRi+2,2*indRobot-1) = .5*sink; W(indiceStartIpoRi+2,2*indRobot) = .5*sink;
                    W(indiceStartIpoRi+3,2*indRobot-1) = 1/d; W(indiceStartIpoRi+3,2*indRobot) = -1/d;

                end

                Q(2*indRobot-1:2*indRobot,2*indRobot-1:2*indRobot) = diag([KR*abs(robots(indRobot).uRe(k)); KL*abs(robots(indRobot).uLe(k))]);
    
            end
    
            % Calcolo matrice P^-
            Pmeno = F*FedEKFs(indFedEKF).P*F' + W*Q*W';
    
            % PASSO DI CORREZIONE (ha effetto solo ogni Nstep passi)
            if mod(k+1,Nstep) == 1 || (Nstep == 1) 
    
                % Misure di range
                for indRobot = 1:nRobot-1
                    for jndRobot = indRobot+1:nRobot
                        misuraRangeTraRobot(indRobot,jndRobot) = sqrt((robots(indRobot).xVett(k+1)-robots(jndRobot).xVett(k+1))^2+(robots(indRobot).yVett(k+1)-robots(jndRobot).yVett(k+1))^2) + 0.5*sigmaDistanza*(robots(indRobot).NNrangeRobot(jndRobot,k+1)+robots(jndRobot).NNrangeRobot(indRobot,k+1));
                    end
                end
                for indRobot = 1:nRobot
                    for indTag = 1:nTag
                        misuraRangeTagRobot(indRobot,indTag) = sqrt((robots(indRobot).xVett(k+1)-cTag(indTag,1)).^2+(robots(indRobot).yVett(k+1)-cTag(indTag,2)).^2) + sigmaDistanza*robots(indRobot).NNrangeTag(indTag,k+1);
                    end
                end
    
                indiceMisura = 0;

                nMisure = sum(nIpoRobot)*nTag; % era nchoosek(nRobot,2)*nPhi^2+nRobot*nPhi*nTag; % sum_{ij,j>i} ipoRi x ipoRj + nRobot x nPhi x nTag
                for indRobot = 1:nRobot-1
                    nMisure = nMisure + nIpoRobot(indRobot)*sum(nIpoRobot(indRobot+1:nRobot));
                end
                H = zeros(nMisure,nStato);
                innovazione = zeros(nMisure,1);
                Rs = zeros(nMisure,nMisure);

                % Misura range tra robot
                pesiNuoviIpoRobot = FedEKFs(indFedEKF).pesiIpoRobot;
                % pause
                for indRobot = 1:nRobot-1
                    for jndRobot = indRobot+1:nRobot
                        probMisura = zeros(nIpoRobot(indRobot)*nIpoRobot(jndRobot),1); % era zeros(nPhi^2,1);
                        indiceMisuraStart = indiceMisura;
                        % pause
                        for indIpoRi = 1:nIpoRobot(indRobot)
                            indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1))+3*(indIpoRi-1);
                            xRi = xHatSLAMmeno(indiceStartIpoRi+1); % era 3*(indRobot-1)*nPhi+3*(indIpoRi-1)+1
                            yRi = xHatSLAMmeno(indiceStartIpoRi+2); % era 3*(indRobot-1)*nPhi+3*(indIpoRi-1)+2
                            for indIpoRj = 1:nIpoRobot(jndRobot)
                                indiceMisura = indiceMisura + 1;
                                indiceStartIpoRj = 3*sum(nIpoRobot(1:jndRobot-1))+3*(indIpoRj-1);
                                xRj = xHatSLAMmeno(indiceStartIpoRj+1); % era 3*(jndRobot-1)*nPhi+3*(indIpoRj-1)+1
                                yRj = xHatSLAMmeno(indiceStartIpoRj+2); % era 3*(jndRobot-1)*nPhi+3*(indIpoRj-1)+2
                                rangeAttesoIpoRiIpoRj = sqrt((xRi-xRj)^2+(yRi-yRj)^2);
                                deltaMisura = misuraRangeTraRobot(indRobot,jndRobot) - rangeAttesoIpoRiIpoRj;
                                % indiceMisura = (indRobot-1)*nPhi^2 +(indIpoRi-1)*nPhi+indIpoRj;
                                innovazione(indiceMisura) = deltaMisura;
                                % indRobot 
                                % jndRobot
                                % indIpoRi
                                % indIpoRj
                                % deltaMisura
                                % pMisura = exp(-deltaMisura^2/(2*(sigmaDistanzaModello^2/2)))
                                % pause
                                vApp = [xRi-xRj yRi-yRj]/rangeAttesoIpoRiIpoRj;
                                H(indiceMisura,indiceStartIpoRi+1:indiceStartIpoRi+2) = vApp;
                                H(indiceMisura,indiceStartIpoRj+1:indiceStartIpoRj+2) = -vApp;
                                varianzaInnovazione = H(indiceMisura,:)*Pmeno*H(indiceMisura,:)' + sigmaDistanzaModello^2/2;
                                probMisura(nIpoRobot(jndRobot)*(indIpoRi-1)+indIpoRj) = exp(-deltaMisura^2/(2*varianzaInnovazione));
                            end
                        end
                        lambda_RiRj = probMisura/sum(probMisura);
                        indiceMisura = indiceMisuraStart;
                        for indIpoRi = 1:nIpoRobot(indRobot)
                            pesiNuoviIpoRobot(indRobot,indIpoRi) = pesiNuoviIpoRobot(indRobot,indIpoRi)*sum(probMisura(nIpoRobot(jndRobot)*(indIpoRi-1)+1:indIpoRi*nIpoRobot(jndRobot)));
                            for indIpoRj = 1:nIpoRobot(jndRobot)
                                indiceMisura = indiceMisura + 1;
                                % indiceMisura = (indRobot-1)*nPhi^2 +(indIpoRi-1)*nPhi+indIpoRj;
                                % NELLA FORMULA SEGUENTE FORSE la VARIANZA
                                % della MISURA ANDAVA DIVISA PER DUE come ORA!
                                Rs(indiceMisura,indiceMisura) = (sigmaDistanzaModello^2/2)/(max(.0001,lambda_RiRj((indIpoRi-1)*nIpoRobot(jndRobot)+indIpoRj)));
                                % con FISequal: Rs(indiceMisura,indiceMisura) = (sigmaDistanzaModello^2/2)*(nIpoRobot(indRobot)*nIpoRobot(jndRobot));
                            end
                        end 
                        for indIpoRj = 1:nIpoRobot(jndRobot)
                                pesiNuoviIpoRobot(jndRobot,indIpoRj) = pesiNuoviIpoRobot(jndRobot,indIpoRj)*sum(probMisura(indIpoRj:nIpoRobot(jndRobot):end)); % era sum(probMisura(indIpoRj:nPhi:end));
                        end
                    end
                end
                % pesiNuoviIpoRobot
                % pause
                 
               % Misure range Robot-landmark
               for indRobot = 1:nRobot 
                   for indTag = 1:nTag
                       indiceMisuraStart = indiceMisura;
                       % pause
                       indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
                       xTag = xHatSLAMmeno(indiceStartTagi+1); % era nRobot*3*nPhi+2*indTag-1
                       yTag = xHatSLAMmeno(indiceStartTagi+2);   % era nRobot*3*nPhi+2*indTag                   
                       probMisura = zeros(nIpoRobot(indRobot),1);
                       for indIpoRi = 1:nIpoRobot(indRobot)
                           indiceMisura = indiceMisura + 1;
                           indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1))+3*(indIpoRi-1);
                           xRi = xHatSLAMmeno(indiceStartIpoRi+1); % era (indRobot-1)*3*nPhi+3*(indIpoRi-1)+1
                           yRi = xHatSLAMmeno(indiceStartIpoRi+2);   % era (indRobot-1)*3*nPhi+3*(indIpoRi-1)+2                        
                           rangeAttesoIpoRiIndTag = sqrt((xRi-xTag)^2+(yRi-yTag)^2);
                           deltaMisura = misuraRangeTagRobot(indRobot,indTag) - rangeAttesoIpoRiIndTag;
                           % indiceMisura = nRobot*nPhi^2+2*nPhi*nTag+2*nPhi*(indTag-1)+2*(indIpoRi-1)+indIpoTag;
                           innovazione(indiceMisura) = deltaMisura;
                           vApp = [xRi-xTag yRi-yTag]/rangeAttesoIpoRiIndTag;
                           H(indiceMisura,indiceStartIpoRi+1:indiceStartIpoRi+2) = vApp;
                           H(indiceMisura,indiceStartTagi+1:indiceStartTagi+2) = -vApp;   
                           varianzaInnovazione = H(indiceMisura,:)*Pmeno*H(indiceMisura,:)' + sigmaDistanzaModello^2;
                           probMisura(indIpoRi) = exp(-deltaMisura^2/(2*varianzaInnovazione));
                        end
                        lambda_Robot_Tag = probMisura/sum(probMisura);
                        indiceMisura = indiceMisuraStart;
                        for indIpoRi = 1:nIpoRobot(indRobot)
                            indiceMisura = indiceMisura + 1;
                            pesiNuoviIpoRobot(indRobot,indIpoRi) = pesiNuoviIpoRobot(indRobot,indIpoRi)*probMisura(indIpoRi);
                            Rs(indiceMisura,indiceMisura) = sigmaDistanzaModello^2/(max(.0001,lambda_Robot_Tag(indIpoRi)));
                            % con FISequal:Rs(indiceMisura,indiceMisura) = sigmaDistanzaModello^2*nIpoRobot(indRobot);
                        end
                    end
               end
               % pesiNuoviIpoRobot
               % pause
               % indiceMisura
               % pause

                % Aggiornamento stima (stima a posteriori)
                KalmanGain = Pmeno*H'*pinv(H*Pmeno*H'+Rs);
                FedEKFs(indFedEKF).xHatSLAM(1:nStato,k+1) = xHatSLAMmeno + KalmanGain*innovazione;
                FedEKFs(indFedEKF).P = (eye(nStato) - KalmanGain*H)*Pmeno;
    
                % Aggiornamento pesi: vanno normalizzati tutti
                for indRobot = 1:nRobot
                    FedEKFs(indFedEKF).pesiIpoRobot(indRobot,1:nIpoRobot(indRobot)) = pesiNuoviIpoRobot(indRobot,1:nIpoRobot(indRobot))/sum(pesiNuoviIpoRobot(indRobot,1:nIpoRobot(indRobot)));
                end
                sommaPesiFedEKF = 0;
                for indRobot = 1:nRobot
                    sommaPesiFedEKF = sommaPesiFedEKF + sum(pesiNuoviIpoRobot(indRobot,1:nIpoRobot(indRobot)));
                end
                pesiNuoviFedEKF(indFedEKF) = pesiFedEKF(indFedEKF)*sommaPesiFedEKF;

                % Pruning
                for indRobot = 1:nRobot
                    if (k > passoInizioPruning) && (nIpoRobot(indRobot) > 1)
                        pesiPrec = FedEKFs(indFedEKF).pesiIpoRobot(indRobot,1:nIpoRobot(indRobot));
                        massimoPesoIndRobot = max(pesiPrec);
                        nIpoRobotPrec = nIpoRobot(indRobot);
                        indiceBase = 3*sum(nIpoRobot(1:indRobot-1));
                        indiceIpotesi = 1;
                        for indIpoRi = 1:nIpoRobotPrec
                            if (pesiPrec(indIpoRi) < .0001*massimoPesoIndRobot) 
                                % disp('eliminata')
                                % pause
                                indiceVettore = indiceBase + 3*(indiceIpotesi-1);
                                FedEKFs(indFedEKF).xHatSLAM(indiceVettore+1:nStato-3,k+1) = FedEKFs(indFedEKF).xHatSLAM(indiceVettore+4:nStato,k+1);
                                FedEKFs(indFedEKF).P = [FedEKFs(indFedEKF).P(1:indiceVettore,1:indiceVettore) FedEKFs(indFedEKF).P(1:indiceVettore,indiceVettore+4:nStato); ...
                                    FedEKFs(indFedEKF).P(indiceVettore+4:nStato,1:indiceVettore) FedEKFs(indFedEKF).P(indiceVettore+4:nStato,indiceVettore+4:nStato)];
                                FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indiceIpotesi:nIpoRobot(indRobot)-1) = FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indiceIpotesi+1:nIpoRobot(indRobot));
                                nStato = nStato - 3;
                                % sizeX = size(FedEKFs(indFedEKF).xHatSLAM(indiceVettore+1:nStato-3,k+1))
                                % pause
                                nIpoRobot(indRobot) = nIpoRobot(indRobot) - 1;
                                % if nIpoRobot(indRobot) == 0
                                %     disp('0')
                                %     pause
                                % end
                            else
                                indiceIpotesi = indiceIpotesi + 1;
                            end
                        end
                        
                        sommaPesi = sum(FedEKFs(indFedEKF).pesiIpoRobot(indRobot,1:nIpoRobot(indRobot)));
                        FedEKFs(indFedEKF).pesiIpoRobot(indRobot,1:nIpoRobot(indRobot)) = FedEKFs(indFedEKF).pesiIpoRobot(indRobot,1:nIpoRobot(indRobot))/sommaPesi;
                    end
                end

                FedEKFs(indFedEKF).dimStato = nStato;
                FedEKFs(indFedEKF).numIpoRobot = nIpoRobot;

            else 
    
                % Se non ho misure confermo la stima a priori
                FedEKFs(indFedEKF).xHatSLAM(1:nStato,k+1) = xHatSLAMmeno;
                FedEKFs(indFedEKF).P = Pmeno;
    
            end
    
           
            if DISEGNA && mod(k,5) == 1
                figure(indFedEKF)
                hold off
                disegnaFig
                title(num2str(k))
                for indRobot = 1:nRobot
                    plot(robots(indRobot).xVett(max(1,k-30):k+1)/100,robots(indRobot).yVett(max(1,k-30):k+1)/100,'k','LineWidth',2)
                    plot(robots(indRobot).xVett(k+1)/100,robots(indRobot).yVett(k+1)/100,'ko')
                end
                axis([0 2 -0 2])

                % Stima media posizione robot
                for indRobot = 1:nRobot
                    xRi = 0;
                    yRi = 0;
                    for indIpoRi = 1:nIpoRobot(indRobot)
                        indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
                        xRi = xRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                        yRi = yRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                    end
                    plot(xRi/100,yRi/100,'mo')
                end

                % Stima posizione tag
                for indTag = 1:nTag
                    indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
                    xTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
                    yTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
                    plot(xTag/100,yTag/100,'m*') 
                end
            end
    
        end

    end

    pesiFedEKF = pesiNuoviFedEKF/sum(pesiNuoviFedEKF);
    % Selezione ad hoc qui che sono solo 2 FedEKF
    if sum(indBuoniFedEKF) == 2 && max(pesiFedEKF) > rapportoFedEKF*min(pesiFedEKF)
        [massimo iMax] = max(pesiFedEKF);
        [minimo iMin] = min(pesiFedEKF);
        indBuoniFedEKF(iMin) = 0;
        indBuoniFedEKF(iMax) = 1;
    end
    % Prima c'era questa piu' generale
    % indBuoniFedEKF = indBuoniFedEKF>0 & pesiFedEKF>pesoMin;
    nFedEKFattivi(k+1) = sum(indBuoniFedEKF);
    % pause(.1)

    % STIMA COMPLESSIVA
    if stimaMediaIstanze == 1
        for indFedEKF = 1:nFedEKF
            nIpoRobot = FedEKFs(indFedEKF).numIpoRobot;
            if indBuoniFedEKF(indFedEKF)
                for indRobot = 1:nRobot
                    xRi = 0;
                    yRi = 0;
                    for indIpoRi = 1:nIpoRobot(indRobot)
                        indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
                        xRi = xRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                        yRi = yRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                    end
                    xHatRobotStoria(indRobot,k+1) = xHatRobotStoria(indRobot,k+1) + xRi*pesiFedEKF(indFedEKF);
                    yHatRobotStoria(indRobot,k+1) = yHatRobotStoria(indRobot,k+1) + yRi*pesiFedEKF(indFedEKF);
                end
    
                for indTag = 1:nTag
                    indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
                    xTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
                    yTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
                    xHatTagStoria(indTag,k+1) = xHatTagStoria(indTag,k+1) + xTag*pesiFedEKF(indFedEKF);
                    yHatTagStoria(indTag,k+1) = yHatTagStoria(indTag,k+1) + yTag*pesiFedEKF(indFedEKF);
                end
            end
        end
    else
        if numel(indBuoniFedEKF) > 1
            [pesoMaxFedEKF,indPesoMaxFedEKF] = max(pesiFedEKF);
        else
            indPesoMaxFedEKF = find(indBuoniFedEKF);
        end
        nIpoRobot = FedEKFs(indPesoMaxFedEKF).numIpoRobot;
        for indRobot = 1:nRobot
            xRi = 0;
            yRi = 0;
            for indIpoRi = 1:nIpoRobot(indRobot)
                indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
                xRi = xRi + FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indPesoMaxFedEKF).pesiIpoRobot(indRobot,indIpoRi);
                yRi = yRi + FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indPesoMaxFedEKF).pesiIpoRobot(indRobot,indIpoRi);
            end
            xHatRobotStoria(indRobot,k+1) = xRi;
            yHatRobotStoria(indRobot,k+1) = yRi;
        end

        for indTag = 1:nTag
            indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
            xHatTagStoria(indTag,k+1) = FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
            yHatTagStoria(indTag,k+1) = FedEKFs(indPesoMaxFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
        end
    end

    if DISEGNA && mod(k,5) == 1
        pause(.01)
    end

    % k
    % pause
    
end
DeltaTsim = etime(clock,T0)
% toc

% CALCOLO ERRORI ASSOLUTI e RELATIVI
xRobotEnd = zeros(nRobot,1);
yRobotEnd = zeros(nRobot,1);
xHatRobotEnd = zeros(nRobot,1);
yHatRobotEnd = zeros(nRobot,1);

for indRobot = 1:nRobot

    % Posizione finale vera dei vari robot
    xRobotEnd(indRobot) = robots(indRobot).xVett(end);
    yRobotEnd(indRobot) = robots(indRobot).yVett(end);

    % Posizione finale stimata dei vari robot
    xHatRobotEnd(indRobot) = xHatRobotStoria(indRobot,end);
    yHatRobotEnd(indRobot) = yHatRobotStoria(indRobot,end);

end

cHatTagEnd = [xHatTagStoria(:,end) yHatTagStoria(:,end)];



% Calcolo distanze vere e stimate tra i landmark e i robot (errori SLAM relativi)
nDistanzeRobot = nchoosek(nRobot,2);
distanzeFinaliRobotVere = zeros(nDistanzeRobot,1);
distanzeFinaliRobotStimate = zeros(nDistanzeRobot,1);
indiceDistanza = 0;
for indRobot = 1:nRobot-1
    for jndRobot = indRobot+1:nRobot
        indiceDistanza = indiceDistanza + 1;
        distanzeFinaliRobotVere(indiceDistanza) = sqrt((xRobotEnd(indRobot)-xRobotEnd(jndRobot))^2+(yRobotEnd(indRobot)-yRobotEnd(jndRobot))^2);
        distanzeFinaliRobotStimate(indiceDistanza) = sqrt((xHatRobotEnd(indRobot)-xHatRobotEnd(jndRobot))^2+(yHatRobotEnd(indRobot)-yHatRobotEnd(jndRobot))^2);
    end
end

distanzeFinaliTagVere = zeros(nTag*nRobot,1);
distanzeFinaliTagStimate = zeros(nTag*nRobot,1);
for indTag = 1:nTag
    for indRobot = 1:nRobot
        distanzeFinaliTagVere(nRobot*(indTag-1)+indRobot) = sqrt((xRobotEnd(indRobot)-cTag(indTag,1))^2+(yRobotEnd(indRobot)-cTag(indTag,2))^2);
        distanzeFinaliTagStimate(nRobot*(indTag-1)+indRobot) = sqrt((xHatRobotEnd(indRobot)-cHatTagEnd(indTag,1))^2+(yHatRobotEnd(indRobot)-cHatTagEnd(indTag,2))^2);
    end
end

distanzeInterTagVere = zeros(nTag*(nTag-1)/2,1);
distanzeInterTagStimate = zeros(nTag*(nTag-1)/2,1);
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
erroriAssolutiRobotStoria = zeros(nRobot,nPassi);
for indRobot = 1:nRobot
    erroriAssolutiRobotStoria(indRobot,:) = sqrt((robots(indRobot).xVett-xHatRobotStoria(indRobot,:)').^2+(robots(indRobot).yVett-yHatRobotStoria(indRobot,:)').^2);
end
erroreAssolutoRobot = erroriAssolutiRobotStoria(:,end);
erroriAssolutiTagStoria = zeros(nTag,nPassi);
for indTag = 1:nTag
    erroriAssolutiTagStoria(indTag,:) = sqrt((xHatTagStoria(indTag,:)-cTag(indTag,1)).^2+(yHatTagStoria(indTag,:)-cTag(indTag,2)).^2);
end
erroreAssolutoTag = erroriAssolutiTagStoria(:,end);

if DISEGNA

    figure
    for indRobot = 1:nRobot
        subplot(nRobot,1,indRobot)
        plot(erroriAssolutiRobotStoria(indRobot,2:end),'.--')
        title(['Robot ' num2str(indRobot)])
        xlabel('Passo simulazione')
        ylabel(['Errore assoluto robot ' num2str(indRobot) ' [cm]'])
        grid on
    end
    figure
    plot(erroriAssolutiTagStoria(:,2:end)','.--')
    title(['Landmark'])
    xlabel('Passo simulazione')
    ylabel('Errori assoluti landmark [cm]')
    grid on

end

if displayErrori
    disp('************************************')
    disp('ERRORI RELATIVI (finali):')
    disp('Distanze tra i robot vere (sopra) e stimate (sotto)')
    [distanzeFinaliRobotVere';distanzeFinaliRobotStimate']
    disp('Distanze tra i robot e i landmark vere (sopra) e stimate (sotto)')
    [distanzeFinaliTagVere'; distanzeFinaliTagStimate']
    disp('Distanze tra i Landmark vere (sopra) e stimate (sotto)')
    [distanzeInterTagVere'; distanzeInterTagStimate']
    disp('ERRORI ASSOLUTI (finali):')
    for indRobot = 1:nRobot
        disp(['Errore assoluto stima posizione robot' num2str(indRobot) ':'])
        erroreAssolutoRobot(indRobot)
    end
    disp('Errori assoluti landmark:')
    erroreAssolutoTag
end