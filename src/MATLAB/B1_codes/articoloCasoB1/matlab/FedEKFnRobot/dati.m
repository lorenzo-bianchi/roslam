gradi = pi/180;

DISEGNA = 0; % mettere 1 per vedere figure, 0 se non si vuole disegnare nulla
displayErrori = 0;

L = 200; % lunghezza lato ambiente (quadrato)

sigmaDistanza = 5; % std in cm della misura di range
sigmaDistanzaModello = sigmaDistanza; % Per prevedere anche un'incertezza sulla deviazione standard dell'errore di misura

% Possibili configurazioni dei landmark (o ancore o tag): cTag e' una matrice dove 
% la riga i-esima contiene le coordinate (x,y) del landmark i-esimo

% cTag = [50 + 100*rand(nTagVett(indProva),1) 50 + 100*rand(nTagVett(indProva),1)];
% cTag = [50 + 100*rand(10,1) 50 + 100*rand(10,1)];
% cTag = [60 60;
%         140 60;
%         100 140];
cTag = [60 60;
        60 150;
        150 60;
        150 150];
% cTag = [40 40;
%         160 40;
%         100 100;
%         40 160;
%         160 160];    
% cTag = [133.3333  133.3333
%   133.3333  266.6667
%   266.6667  133.3333
%   266.6667  266.6667
%   200 200];
[nTag,pippo] = size(cTag);

nRobot = 1; % numero di robot
Nstep = 1; % numero di passi tra una misura e la successiva
nPhi = 8; % numero ipotesi angolo (si puo' poi variare in funzione della distanza misurata)