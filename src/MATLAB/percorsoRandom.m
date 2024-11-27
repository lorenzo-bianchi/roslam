function percorso = percorsoRandom(data, passi_traj, seed)
    rng(seed);
    
    passoOdometrico = 0.01;
    nPassi = passi_traj;
    dVera = data.dVera;
    deltaRvera = data.deltaRvera;
    deltaLvera = data.deltaLvera;
    KRvera = data.KRvera;
    KLvera = data.KLvera;
    L = data.L;

    % Definizione dei vettori x, y e theta delle coordinate del robot durante la simulazione:
    xVett = zeros(nPassi,1);
    yVett = zeros(nPassi,1);
    thetaVett = zeros(nPassi,1);
    
    clearance = 0.75; %.3*L;

    deltaTheta = zeros(nPassi,1);

    xVett(1) = (L-2*clearance)*rand + clearance; % 1 + clearance*cos(2*pi*rand()); %clearance;% + rand*(L-2*clearance);
    yVett(1) = (L-2*clearance)*rand + clearance; % 1 + clearance*sin(2*pi*rand()); %70;% clearance + rand*(L-2*clearance);
    thetaVett(1) = 2*pi*rand - pi; % angolo in radianti
    deltaRho = passoOdometrico*ones(nPassi,1);

    uR = zeros(nPassi,1);
    uL = zeros(nPassi,1);

    cinque = 150*pi/180; % angolo max di curva totale
    uno = 5*pi/180; % angolo max di curva in uno step

    k = 1;
    while k < nPassi
        [lato, distanza] = distanzaBordo(xVett(k), yVett(k), thetaVett(k), data.L);
        distanzaTag = Inf; %sqrt((xVett(k)-cTag(1,1))^2+(yVett(k)-cTag(1,2))^2);
        % if distanzaTag < 0.2
        %     if abs(atan2(cTag(1,2)-yVett(k), cTag(1,1)-xVett(k))-thetaVett(k)) > 20*pi/180
        %         distanzaTag = 1e9;
        %     end
        % end
      
        if distanza < clearance || distanzaTag < 0.2 || rand > 0.995 % ho da girà!
            curvaDaFare = max(uno, rand*cinque); % è la curva in radianti che deve essere effettuata
            passiCurvaDaFare = round(curvaDaFare/uno); % è il numero di passi che il robot impiegherà per fare la curva
            radPerPasso = curvaDaFare/passiCurvaDaFare;
            kIn = min(k, nPassi);
            kFin = min(k+passiCurvaDaFare, nPassi-1);
            indiceK = kIn:kFin;
            direzione = mod(round(thetaVett(k)), 360); % direzione in radianti tra 1 e 360        
            deltaRho(indiceK) = 0; % mi fermo
            if distanzaTag < 0.2 % mi sto dirigendo sotto il tag
                deltaTheta(indiceK) = radPerPasso; % giro verso destra
            end
            if lato == 1 % mi sto dirigendo contro il lato sotto
                if direzione>270 % sto puntando verso destra
                    deltaTheta(indiceK) = radPerPasso; % giro verso destra
                else 
                    deltaTheta(indiceK) = -radPerPasso; % giro verso sinistra perché sto puntando verso sinistra
                end
            elseif lato == 2 % mi sto dirigendo contro il lato destro
                if direzione>180 % sto puntando verso il basso
                    deltaTheta(indiceK) = -radPerPasso; % giro verso il basso
                else 
                    deltaTheta(indiceK) = radPerPasso; % giro verso l'alto perché sto puntando verso l'alto
                end
            elseif lato == 3 % mi sto dirigendo contro il lato sopra
                if direzione<90 % sto puntando verso destra
                    deltaTheta(indiceK) = -radPerPasso; % giro verso destra
                else 
                    deltaTheta(indiceK) = radPerPasso; % giro verso sinistra perché sto puntando verso sinistra
                end            
            elseif lato == 4 % mi sto dirigendo contro il lato sinistro
                if direzione>180 % sto puntando verso il basso
                    deltaTheta(indiceK) = radPerPasso; % giro verso il basso
                else 
                    deltaTheta(indiceK) = -radPerPasso; % giro verso l'alto perché sto puntando verso l'alto
                end
            end 
            for k = kIn:kFin
                uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
                uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

                xVett(k+1) = xVett(k) + ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
                yVett(k+1) = yVett(k) + ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
                thetaVett(k+1) = thetaVett(k) + ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
            end
        else
            uR(k) = (deltaRho(k) + (dVera/2)*deltaTheta(k))/deltaRvera;
            uL(k) = (deltaRho(k) - (dVera/2)*deltaTheta(k))/deltaLvera;

            xVett(k+1) = xVett(k) + ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*cos(thetaVett(k));
            yVett(k+1) = yVett(k) + ((deltaRvera*uR(k)+deltaLvera*uL(k))/2)*sin(thetaVett(k));
            thetaVett(k+1) = thetaVett(k) + ((deltaRvera*uR(k)-deltaLvera*uL(k))/dVera);
        end
        
        k = k + 1;        
        % pause
    end

    % Passaggio da deltaRho e deltaTheta agli spostamenti delle due ruote
    uR = (deltaRho + (dVera/2)*deltaTheta)/deltaRvera;
    uL = (deltaRho - (dVera/2)*deltaTheta)/deltaLvera;

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
    
    percorso = [xVett, yVett, thetaVett, uRe, uLe];
end