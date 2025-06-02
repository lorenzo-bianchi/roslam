distMin = 20;

coordX = linspace(0,L,ceil(nRobot/2)+2);
coordX = coordX(2:end-1);
deltaX = coordX(2) -coordX(1);
nX = numel(coordX);
coordY = linspace(0,L,nRobot-ceil(nRobot/2)+2);
coordY = coordY(2:end-1);
deltaY = coordY(2) -coordY(1);
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

posIniziali(3,1) = max(0,min(L,coordX(floor(nX/2)) + sigmaX*randn));
posIniziali(3,2) = max(0,min(L,coordY(end) + sigmaY*randn));
while min(sqrt(diag((posIniziali(3,:)-cTag)*(posIniziali(3,:)-cTag)'))) < distMin
    posIniziali(3,1) = max(0,min(L,coordX(floor(nX/2)) + sigmaX*randn));
    posIniziali(3,2) = max(0,min(L,coordY(end) + sigmaY*randn));
end
usatiInizi(floor(nX/2),end) = 1;
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