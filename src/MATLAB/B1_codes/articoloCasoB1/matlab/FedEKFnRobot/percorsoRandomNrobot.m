nPassi = 500;
dVera = 26; % distanza presunta tra le due ruote
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
% ERRORE SISTEMATICO
d = dVera;%*1.001; % distanza vera tra le due ruote
deltaR = deltaRvera;%*1.001; % errore sistematico sulla ruota destra
deltaL = deltaLvera;%*1.001; % errore sistematico sulla ruota sinistra
% COSTANTI ERRORE NON SISTEMATICO
KRvera = 0.01;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KLvera = KRvera;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
% ERRORE NELLA LORO STIMA
KR = KRvera;%*1.01;
KL = KLvera;%*1.01;

passoOdometrico = 1;
clearance = 75;%.3*L;
cinque = 100;%150; % angolo max di curva totale
uno = 5; % angolo max di curva in uno step

% PARTE MULTIPLA
% Definisco la struttura robot
robot = struct('xVett',zeros(nPassi,1),'yVett',zeros(nPassi,1),'thetaVett',zeros(nPassi,1),...
'uRe',zeros(nPassi,1),'uLe',zeros(nPassi,1),...
'NNrange',randn(nTag,nPassi),'pesi',(1/nPhi)*ones(nTag,nPhi),...
'xHatSLAM',zeros(3+(3+nPhi)*nTag,nPassi),'xHatSLAMmeno',zeros(3+(3+nPhi)*nTag,1),'P',zeros(3+(3+nPhi)*nTag,3+(3+nPhi)*nTag),...
'xHatTagStoria',zeros(nTag,nPassi),'yHatTagStoria',zeros(nTag,nPassi),...
'distanzeRobotVere',zeros(1,nTag),'distanzeRobotStimate',zeros(1,nTag),...
'distanzeInterTagVere',zeros(1,nTag*(nTag-1)/2),'distanzeInterTagStimate',zeros(1,nTag*(nTag-1)/2),...
'distanzeInterTagStimateStoria',zeros(nTag*(nTag-1)/2,nPassi),...
'erroriAssolutiTag',zeros(1,nTag),'erroreAssolutoRobot',0,...
'erroreTraiettoria',zeros(nPassi,1),'erroreTag',zeros(4,nPassi));

% Definisco un insieme di robot
robots = repmat(robot,1,nRobot);

for indRobot = 1:nRobot

    deltaTheta = zeros(nPassi,1);
    
    % angoloRand = 2*pi*rand;
    robots(indRobot).xVett(1) = (L-2*clearance)*rand + clearance; %100+clearance*cos(angoloRand);%clearance;% + rand*(L-2*clearance);
    robots(indRobot).yVett(1) = (L-2*clearance)*rand + clearance; %100+clearance*sin(angoloRand);%70;% clearance + rand*(L-2*clearance);
    robots(indRobot).thetaVett(1) = pi; %0;%360*rand*gradi; % angolo in radianti: gradi e' il fattore di conversione
    deltaRho = passoOdometrico*ones(nPassi,1);
    
    uR = zeros(nPassi,1);
    uL = zeros(nPassi,1);
    
    
    k = 1;
    while k < nPassi
    
        [lato, distanza] = distanzaBordo(robots(indRobot).xVett(k),robots(indRobot).yVett(k),robots(indRobot).thetaVett(k));
      
        if distanza < clearance % || mod(k,30) == 1
            curvaDaFare = max(uno,rand*cinque);
            passiCurvaDaFare = round(curvaDaFare/uno);
            gradiPerPasso = curvaDaFare/passiCurvaDaFare;
            kIn = min(k,nPassi);
            kFin = min(k+passiCurvaDaFare,nPassi-1);
            indiceK = [kIn:kFin];
            direzione = mod(round(robots(indRobot).thetaVett(k)/gradi),360); % direzione in gradi tra 1 e 360        
            deltaRho(indiceK)=0; % mi fermo
            if lato == 1 % mi sto dirigendo contro il lato sotto
                if direzione>270 % sto puntando verso destra
                    deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
                else
                    deltaTheta(indiceK) = -gradiPerPasso*gradi;
                end
            elseif lato == 2 % mi sto dirigendo contro il lato destro
                if direzione>180 % sto puntando verso il basso
                    deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso il basso
                else
                    deltaTheta(indiceK) = gradiPerPasso*gradi;
                end
            elseif lato == 3 % mi sto dirigendo contro il lato sopra
                if direzione<90 % sto puntando verso destra
                    deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso destra
                else
                    deltaTheta(indiceK) = gradiPerPasso*gradi;
                end            
            elseif lato == 4 % mi sto dirigendo contro il lato sinistro
                if direzione>180 % sto puntando verso il basso
                    deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso il basso
                else
                    deltaTheta(indiceK) = -gradiPerPasso*gradi;
                end
            end 
            for k = kIn:kFin
                uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
                uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;
    
                robots(indRobot).xVett(k+1)= robots(indRobot).xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(robots(indRobot).thetaVett(k));
                robots(indRobot).yVett(k+1)= robots(indRobot).yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(robots(indRobot).thetaVett(k));
                robots(indRobot).thetaVett(k+1)= robots(indRobot).thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
    %                 k
            end   
        else
            uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
            uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;
    
            robots(indRobot).xVett(k+1)= robots(indRobot).xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(robots(indRobot).thetaVett(k));
            robots(indRobot).yVett(k+1)= robots(indRobot).yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(robots(indRobot).thetaVett(k));
            robots(indRobot).thetaVett(k+1)= robots(indRobot).thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
    %             k
        end
        
        k = k + 1;        
    %         pause
    end
    
        % Errori odometrici
    NiR = randn(nPassi,1)*sqrt(KRvera); 
    NiL = randn(nPassi,1)*sqrt(KLvera); 
    

    for k = 1:nPassi
        robots(indRobot).uRe(k) = uR(k) + sqrt(abs(uR(k)))*NiR(k); %encoders reading - letture odometriche ruota destra
        robots(indRobot).uLe(k) = uL(k) + sqrt(abs(uL(k)))*NiL(k); %encoders reading - letture odometriche ruota sinistra
    end
 
end