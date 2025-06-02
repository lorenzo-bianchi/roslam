% Da eseguire dopo FedEKFCasoB1conOptInConPruning.m

distanzeFinaliRobotLandmark = 100*[1.14, 2.27, 3.20, 2.59; 2.05, 2.01, 2.02, 2.21; 2.59, 2.59, 1.52, 1.68];
xCentri = cTag(:,1);
yCentri = cTag(:,2);

figure
hold on
for indTag = 1:nTag
    plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rh','Linewidth',2,'MarkerSize',12);
    if indTag > 2
        text(cTag(indTag,1)/100-.1,cTag(indTag,2)/100-.3,['L' num2str(indTag)],'FontSize',12,'FontWeight','Bold')
    else
        text(cTag(indTag,1)/100-.1,cTag(indTag,2)/100+.3,['L' num2str(indTag)],'FontSize',12,'FontWeight','Bold')
    end
end

title('All robots')

% GT robots
for indRobot = 1:nRobot
    raggiCerchi = distanzeFinaliRobotLandmark(indRobot,:)';
    [xRobotFinale,yRobotFinale,dummy] = trovaPosTag(xCentri,yCentri,raggiCerchi);
    plot(xRobotFinale/100,yRobotFinale/100,'ro','LineWidth',3,'MarkerSize',10)
    text(xRobotFinale/100+.1,yRobotFinale/100-.1,['R' num2str(indRobot)],'FontSize',12,'FontWeight','Bold')
end

% Stima robots
for indRobot = 1:nRobot
    xRi = 0;
    yRi = 0;
    for indIpoRi = 1:nIpoRobot(indRobot)
        indiceStartIpoRi = 3*sum(nIpoRobot(1:indRobot-1)) + 3*(indIpoRi-1);
        xRi = xRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+1,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
        yRi = yRi + FedEKFs(indFedEKF).xHatSLAM(indiceStartIpoRi+2,k+1)*FedEKFs(indFedEKF).pesiIpoRobot(indRobot,indIpoRi);
    end
    plot(xRi/100,yRi/100,'kx','LineWidth',3,'MarkerSize',10)
end

% Stima landmarks
for indTag = 1:nTag
    indiceStartTagi = 3*sum(nIpoRobot(1:nRobot))+2*(indTag-1);
    xTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+1,k+1);
    yTag = FedEKFs(indFedEKF).xHatSLAM(indiceStartTagi+2,k+1);
    plot(xTag/100,yTag/100,'kp','LineWidth',3) 
end
grid on
axis([-.5 2.2 -.3 4.1])
xlabel('x[m]')
ylabel('y[m]')