% Definizione dei quadrilateri (ogni riga è un vertice [x, y])
quad1 = anchors_poses_world; % Quadrato
quad2 = pos_anchors; % Rettangolo ruotato e scalato

% ---- Normalizzazione e centratura ----
% Baricentro
centroid1 = mean(quad1, 1);
centroid2 = mean(quad2, 1);

% Centrare i quadrilateri
Q1 = quad1 - centroid1;
Q2 = quad2 - centroid2;

% Normalizzare la scala (usiamo la diagonale più lunga come riferimento)
scale1 = max(vecnorm(Q1, 2, 2));
scale2 = max(vecnorm(Q2, 2, 2));

Q1 = Q1 / scale1;
Q2 = Q2 / scale2;

% ---- Trova la rotazione ottimale ----
H = Q1' * Q2;
[U, ~, V] = svd(H);
R = V * U';

% ---- Ruota il primo quadrilatero ----
Q1_rotated = (R * Q1')';

% ---- Calcola l'errore medio ----
similarity_score = mean(vecnorm(Q1_rotated - Q2, 2, 2));

% ---- Visualizza i risultati ----
fprintf('Errore di similarità: %.4f\n', similarity_score);

figure; hold on; axis equal;
scatter(Q1(:,1), Q1(:,2), 'ro', 'filled'); % Primo quadrilatero (normalizzato)
scatter(Q2(:,1), Q2(:,2), 'bo', 'filled'); % Secondo quadrilatero (normalizzato)
scatter(Q1_rotated(:,1), Q1_rotated(:,2), 'go'); % Primo quadrilatero ruotato
legend('Quad1 Normalizzato', 'Quad2 Normalizzato', 'Quad1 Ruotato');
title('Confronto tra quadrilateri');
grid on;