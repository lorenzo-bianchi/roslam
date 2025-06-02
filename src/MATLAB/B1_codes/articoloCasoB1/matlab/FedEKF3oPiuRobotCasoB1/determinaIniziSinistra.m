% Sinistra nel senso che il terzo robot viene messo in alto a sinistra
% (mentre i primi due sono a sinistra e a destra in basso). Questo in
% realta' e' quello che non volendo facevo in determinaIniziPrec, in cui poi
% c'era anche qualche altra cosa da aggiustare se i robot non erano 5...

distMin = 20;

if nRobot > 4
    coordX = linspace(0,L,ceil(nRobot/2)+2);
    coordX = coordX(2:end-1);
    coordY = linspace(0,L,nRobot-ceil(nRobot/2)+2);
    coordY = coordY(2:end-1);
else
    coordX = [0.33*L 0.5*L 0.66*L];
    coordY = [0.33*L 0.66*L];
end
deltaX = coordX(2) - coordX(1);
nX = numel(coordX);
deltaY = coordY(2) - coordY(1);
nY = numel(coordY);

posIniziali = zeros(nRobot,2);
usatiInizi = zeros(nX,nY);

% figure
% disegnaFig

sigmaX = deltaX/6;
sigmaY = deltaY/6;

posIniziali(1,1) = max(0,min(L,coordX(1) + sigmaX*randn));
posIniziali(1,2) = max(0,min(L,coordY(1) + sigmaY*randn));
while min(sqrt(diag((posIniziali(1,:)-cTag)*(posIniziali(1,:)-cTag)'))) < distMin
    posIniziali(1,1) = max(0,min(L,coordX(1) + sigmaX*randn));
    posIniziali(1,2) = max(0,min(L,coordY(1) + sigmaY*randn));
end
usatiInizi(1,1) = 1;
% plot(posIniziali(1,1)/100,posIniziali(1,2)/100,'ro')

posIniziali(2,1) = max(0,min(L,coordX(end) + sigmaX*randn));
posIniziali(2,2) = max(0,min(L,coordY(1) + sigmaY*randn));
while min(sqrt(diag((posIniziali(2,:)-cTag)*(posIniziali(2,:)-cTag)'))) < distMin
    posIniziali(2,1) = max(0,min(L,coordX(end) + sigmaX*randn));
    posIniziali(2,2) = max(0,min(L,coordY(1) + sigmaY*randn));
end
usatiInizi(end,1) = 1;
% plot(posIniziali(2,1)/100,posIniziali(2,2)/100,'ko')

coordMedia = coordX(1);
posIniziali(3,1) = max(0,min(L,coordMedia + sigmaX*randn));
posIniziali(3,2) = max(0,min(L,coordY(end) + sigmaY*randn));
while min(sqrt(diag((posIniziali(3,:)-cTag)*(posIniziali(3,:)-cTag)'))) < distMin
    posIniziali(3,1) = max(0,min(L,coordMedia + sigmaX*randn));
    posIniziali(3,2) = max(0,min(L,coordY(end) + sigmaY*randn));
end
usatiInizi(coordMedia,end) = 1;
% plot(posIniziali(3,1)/100,posIniziali(3,2)/100,'bo')

indX = 1;
indY = 1;
indRobot = 4;

while indRobot <= nRobot

    if usatiInizi(indX,indY) == 0
        posIniziali(indRobot,1) = max(0,min(L,coordX(indX) + sigmaX*randn));
        posIniziali(indRobot,2) = max(0,min(L,coordY(indY) + sigmaY*randn));
        while min(sqrt(diag((posIniziali(indRobot,:)-cTag)*(posIniziali(indRobot,:)-cTag)'))) < distMin
            posIniziali(indRobot,1) = max(0,min(L,coordX(indX) + sigmaX*randn));
            posIniziali(indRobot,2) = max(0,min(L,coordY(indY) + sigmaY*randn));
        end
        % plot(posIniziali(indRobot,1)/100,posIniziali(indRobot,2)/100,'co')
        indRobot = indRobot + 1;
    end

    indX = indX + 1;
    if indX > nX
        indX = 1;
        indY = indY + 1;
    end

end

% figure
% disegnaFig
% for indRobot = 1:3
%     plot(posIniziali(indRobot,1)/100,posIniziali(indRobot,2)/100,'ko')
% end
% for indRobot = 4:nRobot
%     plot(posIniziali(indRobot,1)/100,posIniziali(indRobot,2)/100,'bo')
% end