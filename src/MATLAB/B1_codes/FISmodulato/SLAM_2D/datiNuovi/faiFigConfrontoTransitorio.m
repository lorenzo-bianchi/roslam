load esempioSigma10Confronto.mat
cd ..

DISEGNA = 0;

% Metodo con reinizializzazione
% CampanaroReInit2: % 3.9628, 3.8331, 5.7637, 3.9277, 4.1864
CampanaroReInit2azzerando
disegnaFiguraFinale
hold on
plot(125*[1 1],[0 200],'k:','LineWidth',1)
plot(169*[1 1],[0 200],'k:','LineWidth',1)
legend('1','2','3','4')
erroreTag(:,end)
    % 4.9131
    % 5.3552
    % 7.2852
    % 5.1629
rmseRobot = sqrt(mean(erroreTraiettoria.^2))
%  5.1784
% Per vedere quando ci sono state le reinizializzazioni
for indTag = 1:nTag
    find(reinizializzaVett(indTag,:))
end
% Disegna la media delle innovazioni, scegliendo il tag
indTag = 4; % 2 e 4 sono i tag reinizializzati nell'esempio
cd datiNuovi
disegnaStoriaInnovazioneAzzerata
cd ..

% Metodo con K misto
FedEKFconPruningConKmistoSenzaDoppioni
disegnaFiguraFinale
legend('1','2','3','4')
erroreTag(:,end)
    % 1.3918
    % 3.5613
    % 2.4814
    % 2.9266
rmseRobot = sqrt(mean(erroreTraiettoria.^2))
    % 2.1757

% Metodo base con pruning a 200 (con pruning a zero diverge):
FedEKFconPruning
disegnaFiguraFinale
legend('1','2','3','4')
erroreTag(:,end)
%     1.7605
%     0.3718
%     3.1423
%     2.9273
rmseRobot = sqrt(mean(erroreTraiettoria.^2))
%     2.6952

% Senza pruning:
% erroreTag(:,end)
%     3.2709
%     2.2900
%     1.8420
%     1.8946
% rmseRobot = sqrt(mean(erroreTraiettoria.^2))
%     1.9923

% Per trovare altro bell'esempio:
% 1. Eseguire cercaOutlier.m scommentando pero' in CampanaroReInit2.m le
% righe 189-190.
% 2. Fare plot(storiaInnovazione(tagReinizializzato,:),'.-'), per vedere se
% viene una bella innovazione scollimata
% 3. Fare DISEGNA = 1, ricommentare le righe 189-190 ed eseguire le righe 
% sopra, per vedere se vengono belle figure e begli errori.