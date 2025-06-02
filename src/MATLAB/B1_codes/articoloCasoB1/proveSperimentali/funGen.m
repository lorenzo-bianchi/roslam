function resto = funGen(x,pos,rhoVett)

% Sia nC il numero dei cerchi considerati.
% Allora, pos e' una matrice 2xnC con le coordinate dei centri degli nC cerchi:
% le x sulla prima riga e le y sulla seconda.
% rhoVett e' un vettore riga 1xnC con i raggi degli nC cerchi. 
% x sono le coordinate scritte come vettore colonna 2x1 del punto
% candidato ad essere l'intersezione degli nC cerchi.

deltaPos = pos-x; % crea una matrice 2xnCerchi
distanzeAttuali = vecnorm(deltaPos); % vettore riga 1xnC con le distanze dei vari centri dei cerchi da x
resto = distanzeAttuali - rhoVett; % se x e' l'intersezione giusta, questo resto dovrebbe essere un vettore di zeri