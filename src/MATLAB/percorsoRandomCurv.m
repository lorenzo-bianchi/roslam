function percorso = percorsoRandomCurv(data, passi_traj, seed)
    rng(seed);

    % Parametri
    passoOdometrico = 0.01;
    nPassi = passi_traj;
    dVera = data.dVera;
    deltaRvera = data.deltaRvera;
    deltaLvera = data.deltaLvera;
    KRvera = data.KRvera;
    KLvera = data.KLvera;
    L = data.L;

    % Coordinate iniziali del robot
    xVett = zeros(nPassi, 1);
    yVett = zeros(nPassi, 1);
    thetaVett = zeros(nPassi, 1);

    clearance = 0.1 * L; % Distanza minima dalle pareti
    xVett(1) = (L - 2 * clearance) * rand + clearance; 
    yVett(1) = (L - 2 * clearance) * rand + clearance; 
    thetaVett(1) = atan2(L/2-yVett(1), L/2-xVett(1)) + (rand-0.5)*5*pi/180;

    % Variabili per la traiettoria
    deltaRho = passoOdometrico * ones(nPassi, 1); % Movimento lineare iniziale
    deltaTheta = zeros(nPassi, 1); % Rotazione iniziale

    k = 1;
    while k < nPassi
        % Estrazione casuale di un'azione
        azione = randi([1, 5]); % Estrai un intero tra 1 e 5
        passi = 0;
        if azione == 1
            % Vai dritto per 10 istanti
            passi = min(randi([10, 20]), nPassi - k);
            deltaRho(k:k+passi-1) = passoOdometrico;
            deltaTheta(k:k+passi-1) = 0;
        elseif azione == 2
            % Gira a destra per 10 istanti
            passi = min(randi([10, 40]), nPassi - k);
            deltaRho(k:k+passi-1) = passoOdometrico;
            deltaTheta(k:k+passi-1) = -2*pi/180; % Curvatura verso destra
        elseif azione == 3
            % Gira a sinistra per 10 istanti
            passi = min(randi([10, 40]), nPassi - k);
            deltaRho(k:k+passi-1) = passoOdometrico;
            deltaTheta(k:k+passi-1) = 2*pi/180; % Curvatura verso sinistra
        elseif azione == 5
            % Sta fermo per 20 istanti
            passi = min(randi([10, 50]), nPassi - k);
            deltaRho(k:k+passi-1) = 0;
            deltaTheta(k:k+passi-1) = 0;
        end

        % Aggiorna la traiettoria
        for j = k:min(k+passi-1, nPassi-1)
            uR = (deltaRho(j) + (dVera/2) * deltaTheta(j)) / deltaRvera;
            uL = (deltaRho(j) - (dVera/2) * deltaTheta(j)) / deltaLvera;

            xVett(j+1) = xVett(j) + ((deltaRvera*uR + deltaLvera*uL) / 2) * cos(thetaVett(j));
            yVett(j+1) = yVett(j) + ((deltaRvera*uR + deltaLvera*uL) / 2) * sin(thetaVett(j));
            thetaVett(j+1) = thetaVett(j) + ((deltaRvera*uR - deltaLvera*uL) / dVera);
        end

        % Avanza nel tempo
        k = k + passi;
    end

    % Aggiunta di errori odometrici
    NiR = randn(nPassi, 1) * sqrt(KRvera); 
    NiL = randn(nPassi, 1) * sqrt(KLvera); 
    
    % Letture odometriche corrotte
    uRe = zeros(nPassi, 1); 
    uLe = zeros(nPassi, 1); 
    for k = 1:nPassi
        uRe(k) = (deltaRho(k) + (dVera/2) * deltaTheta(k)) / deltaRvera + sqrt(abs(deltaRho(k))) * NiR(k);
        uLe(k) = (deltaRho(k) - (dVera/2) * deltaTheta(k)) / deltaLvera + sqrt(abs(deltaRho(k))) * NiL(k);
    end

    % Output finale
    percorso = [xVett, yVett, thetaVett, uRe, uLe];
end
