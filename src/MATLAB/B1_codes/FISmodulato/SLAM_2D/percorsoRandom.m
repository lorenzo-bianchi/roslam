nPassi = 500;
dVera = 26; % distanza presunta tra le due ruote
deltaRvera = 1; % fattore collegato con diametro ruota destra
deltaLvera = 1; % fattore collegato con diametro ruota sinistra
% ERRORE SISTEMATICO
d = dVera;%*1.001; % distanza vera tra le due ruote, se d = dVera non c'e' errore sistematico
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

% Definizione dei vettori x,y e theta delle coordinate del robot durante la simulazione:
xVett = zeros(nPassi,1);
yVett = zeros(nPassi,1);
thetaVett = zeros(nPassi,1);

deltaTheta = zeros(nPassi,1);

% angoloRand = 2*pi*rand;
xVett(1) = (L-2*clearance)*rand + clearance; %100+clearance*cos(angoloRand);%clearance;% + rand*(L-2*clearance);
yVett(1) = (L-2*clearance)*rand + clearance; %100+clearance*sin(angoloRand);%70;% clearance + rand*(L-2*clearance);
thetaVett(1) = pi;%360*rand*gradi; % angolo in radianti: gradi e' il fattore di conversione
deltaRho = passoOdometrico*ones(nPassi,1);

uR = zeros(nPassi,1);
uL = zeros(nPassi,1);

cinque = 100;%150; % angolo max di curva totale
uno = 5; % angolo max di curva in uno step

k = 1;
while k < nPassi

    [lato, distanza] = distanzaBordo(xVett(k),yVett(k),thetaVett(k));
    distanzaTag = Inf; %sqrt((xVett(k)-cTag(1,1))^2+(yVett(k)-cTag(1,2))^2);
    if distanzaTag < 20
        if abs(atan2(cTag(1,2)-yVett(k),cTag(1,1)-xVett(k))-thetaVett(k)) > 20*pi/180
            distanzaTag = Inf;
        end
    end

   
  
    if distanza < clearance || distanzaTag < 20 % ho da gira'!
        curvaDaFare = max(uno,rand*cinque); % la curva in gradi che deve essere effettuata
        passiCurvaDaFare = round(curvaDaFare/uno); % il numero di passi che il robot impieghera' per fare la curva
        gradiPerPasso = curvaDaFare/passiCurvaDaFare;
        kIn = min(k,nPassi);
        kFin = min(k+passiCurvaDaFare,nPassi-1);
        indiceK = [kIn:kFin];
        direzione = mod(round(thetaVett(k)/gradi),360); % direzione in gradi tra 1 e 360        
        deltaRho(indiceK)=0; % mi fermo
        if distanzaTag < 20 % mi sto dirigendo sotto il tag
            deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
        end
        if lato == 1 % mi sto dirigendo contro il lato sotto
            if direzione>270 % sto puntando verso destra
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
            else deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso sinistra perche' sto puntando verso sinistra
            end
        elseif lato == 2 % mi sto dirigendo contro il lato destro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso il basso
            else deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso l'alto perche' sto puntando verso l'alto
            end
        elseif lato == 3 % mi sto dirigendo contro il lato sopra
            if direzione<90 % sto puntando verso destra
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso destra
            else deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso sinistra perche' sto puntando verso sinistra
            end            
        elseif lato == 4 % mi sto dirigendo contro il lato sinistro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso il basso
            else deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso l'alto perche' sto puntando verso l'alto
            end
        end 
        for k = kIn:kFin
            uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
            uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

            xVett(k+1)= xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
            yVett(k+1)= yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
            thetaVett(k+1)= thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
%                 k
        end   
    else
        uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
        uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

        xVett(k+1)= xVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
        yVett(k+1)= yVett(k)+ ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
        thetaVett(k+1)= thetaVett(k)+ ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
%             k
    end
    
    k = k + 1;        
%         pause
end


% Errori odometrici
NiR = randn(nPassi,1)*sqrt(KRvera); 
NiL = randn(nPassi,1)*sqrt(KLvera); 

% Letture odometriche corrotte
uRe = zeros(nPassi,1); % vettore letture odometriche ruota destra
uLe = zeros(nPassi,1); % vettore letture odometriche ruota sinistra
for k = 1:nPassi
    uRe(k) = uR(k) + sqrt(abs(uR(k)))*NiR(k); %encoders reading - letture odometriche ruota destra
    uLe(k) = uL(k) + sqrt(abs(uL(k)))*NiL(k); %encoders reading - letture odometriche ruota sinistra
end

% Disturbo letture fase
NNrange = randn(nTag,nPassi); 