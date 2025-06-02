% Da eseguire dopo FedEKFnRobotConPruning.m

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

title(['Robot ' num2str(robotScelto)])

% GT robots
raggiCerchi = distanzeFinaliRobotLandmark(robotScelto,:)';
[xRobotFinale,yRobotFinale,dummy] = trovaPosTag(xCentri,yCentri,raggiCerchi);
plot(xRobotFinale/100,yRobotFinale/100,'ro','LineWidth',3,'MarkerSize',10)
text(xRobotFinale/100+.1,yRobotFinale/100-.1,['R' num2str(robotScelto)],'FontSize',12,'FontWeight','Bold')

% Stima robot
plot(robots(indRobot).xHatSLAM(1,end)/100,robots(indRobot).xHatSLAM(2,end)/100,'kx','LineWidth',3,'MarkerSize',10)
text(robots(indRobot).xHatSLAM(1,end)/100+.1,robots(indRobot).xHatSLAM(2,end)/100-.1,['R' num2str(robotScelto) 'hat'],'FontSize',12,'FontWeight','Bold')

% Stima landmark
for indTag = 1:nTag
    indiceStartIndTag = 3+3*(indTag-1)+sum(nIpoTag(1:indTag-1));
    x_i = robots(indRobot).xHatSLAM(indiceStartIndTag+1,end);
    y_i = robots(indRobot).xHatSLAM(indiceStartIndTag+2,end);
    rho_i = robots(indRobot).xHatSLAM(indiceStartIndTag+3,end);
    x_ti = 0;
    y_ti = 0;
    for jndPhi = 1:nIpoTag(indTag)
        phi_ij = robots(indRobot).xHatSLAM(indiceStartIndTag+3+jndPhi,end);
        cosPhi_ij = cos(phi_ij);
        sinPhi_ij = sin(phi_ij);
        xTag_ij = x_i + rho_i*cosPhi_ij;
        yTag_ij = y_i + rho_i*sinPhi_ij;
        x_ti = x_ti + xTag_ij*robots(indRobot).pesi(indTag,jndPhi);
        y_ti = y_ti + yTag_ij*robots(indRobot).pesi(indTag,jndPhi);
        plot(xTag_ij/100,yTag_ij/100,'m.','MarkerSize',max(1,ceil(10*robots(indRobot).pesi(indTag,jndPhi))))
    end
    plot(x_ti/100,y_ti/100,'kp','LineWidth',3) 
    text(x_ti/100-.1,y_ti/100-.3,['L' num2str(indTag) 'hat'],'FontSize',12,'FontWeight','Bold')
end

grid on
% axis([-.5 2.2 -.3 4.1]) % R1
% axis([-2.2 2.4 -.3 4.1]) % R2
axis([-.5 3 -.5 4.5]) % R3
xlabel('x[m]')
ylabel('y[m]')