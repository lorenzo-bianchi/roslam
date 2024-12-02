dati;
seed = 3;

rng(seed);

% Parametri
passoOdometrico = 0.01;
passi_traj = 10000;
dVera = data.dVera;
deltaRvera = data.deltaRvera;
deltaLvera = data.deltaLvera;
KRvera = data.KRvera;
KLvera = data.KLvera;
L = data.L;

passi_traj = 10000;
percorsi = zeros(passi_traj, 5, nRobot);

for robot = 1:nRobot
    percorsi(:, :, robot) = percorsoRandomCurv(data, passi_traj, seed + 10*robot);     % xVett, yVett, thetaVett, uRe, uLe
    
    figure;
    axis([0 L 0 L]); % Definisce i limiti dell'area di disegno
    grid on;
    title('Disegna una curva con il mouse');
    hold on;
    
    h = drawfreehand('LineWidth', 2, 'Color', 'b'); % Disegna una curva libera
    curvePoints = h.Position; % Ottieni i punti della curva
    disp('Punti della curva:');
    disp(curvePoints);
    
    % Opzionale: Traccia i punti catturati
    plot(curvePoints(:,1), curvePoints(:,2), 'r.--');
    
    
    
    
    
    
    nPointsOriginal = size(curvePoints, 1);
    
    tOriginal = linspace(0, 1, nPointsOriginal);
    tInterpolated = linspace(0, 1, passi_traj);
    
    interpolatedX = interp1(tOriginal, curvePoints(:, 1), tInterpolated, 'linear');
    interpolatedY = interp1(tOriginal, curvePoints(:, 2), tInterpolated, 'linear');
    
    interpolatedData = [interpolatedX', interpolatedY'];

    xVett = interpolatedX;
    yVett = interpolatedY;
    dx = diff(xVett); % Differenze tra i punti di x
    dy = diff(yVett); % Differenze tra i punti di y
    theta_local = atan2(dy, dx); % Angoli tra i punti consecutivi
    delta_theta = diff([theta_local(1); theta_local]); % Include il primo angolo
    delta_theta = mod(delta_theta + pi, 2*pi) - pi; % Normalizza variazioni a [-pi, pi]
    theta = cumsum(delta_theta); % Angolo continuo

    thetaVett = [thetaVett, thetaVett(end)]; % Ripeti l'ultimo angolo per l'ultimo punto
    dtheta = [0, diff(thetaVett)];
    
    deltaRho = sqrt(dx.^2 + dy.^2); % Spostamento lineare
    deltaRho = [deltaRho, deltaRho(end)];

    uR = (deltaRho + (dVera/2)*dtheta)/deltaRvera;
    uL = (deltaRho - (dVera/2)*dtheta)/deltaLvera;

    % Errori odometrici
    NiR = randn(passi_traj,1)*sqrt(KRvera); 
    NiL = randn(passi_traj,1)*sqrt(KLvera); 
    
    % Letture odometriche corrotte
    uRe = zeros(passi_traj,1); % vettore letture odometriche ruota destra
    uLe = zeros(passi_traj,1); % vettore letture odometriche ruota sinistra
    for k = 1:passi_traj
        uRe(k) = uR(k) + sqrt(abs(uR(k)))*NiR(k); %encoders reading - letture odometriche ruota destra
        uLe(k) = uL(k) + sqrt(abs(uL(k)))*NiL(k); %encoders reading - letture odometriche ruota sinistra
    end

    percorsi(:, :, robot) = [xVett', yVett', thetaVett', uRe, uLe];

    % Visualizzazione
    figure;
    plot(curvePoints(:, 1), curvePoints(:, 2), 'bo-', 'LineWidth', 1.5, 'MarkerSize', 6); % Dati originali
    hold on;
    plot(interpolatedData(:, 1), interpolatedData(:, 2), 'r-', 'LineWidth', 1); % Dati interpolati
    legend('Original Data', 'Interpolated Data');
    grid on;
    
    set(gcf, 'Renderer', 'Painters');
    pause(0.2)
    set(gcf, 'Renderer', 'OpenGL');

    pause()
end
save('percorsi.mat', 'percorsi');
