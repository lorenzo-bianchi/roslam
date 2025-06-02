load roslam_data3.mat

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]*100;
nTag = 4;
      
robot = 3;
%rRuote = roslam_data.wheel_radius*100;
dVera = roslam_data.wheels_separation*100;
odoRuote = roslam_data.wheels_odometry{robot};
tempi = odoRuote(:,1);
tempiOdo = roslam_data.robot_odometry(robot).times;
uRe = 100*odoRuote(:,2);
uLe = 100*odoRuote(:,3);
nPassi = numel(tempi);
xVett = zeros(nPassi+1,1);
yVett = zeros(nPassi+1,1);
thetaVett = zeros(nPassi+1,1);
if robot == 1
    xVett(1) =  0.0905*100;
    yVett(1) = 2.6885*100;
    % thetaVett(1) = 0.000249377917498350;
end
if robot == 2
    xVett(1) =  0.0605*100;
    yVett(1) = 1.6437*100;
    thetaVett(1) = -pi/2;
end
if robot == 3
    xVett(1) =  1.7309*100;
    yVett(1) = 0.7688*100;
    thetaVett(1) = pi/2;
end

for k = 1:nPassi

    deltaRho = (uRe(k)+uLe(k))/2;
    deltaTheta = (uRe(k)-uLe(k))/dVera;
    xVett(k+1) = xVett(k) + deltaRho*cos(thetaVett(k));
    yVett(k+1) = yVett(k) + deltaRho*sin(thetaVett(k));
    thetaVett(k+1) = thetaVett(k) + deltaTheta;

end

xOdo = roslam_data.robot_odometry(robot).x;
yOdo = roslam_data.robot_odometry(robot).y;
thetaRot = thetaVett(1);
cTheta = cos(thetaRot);
sTheta = sin(thetaRot);
xOdoRot = cTheta*xOdo-sTheta*yOdo;
yOdoRot = sTheta*xOdo+cTheta*yOdo;
xOdo = 100*xOdoRot + xVett(1);
yOdo = 100*yOdoRot + yVett(1);

figure
plot(xVett,yVett)
hold on
plot(xOdo,yOdo,'g')
for indTag = 1:nTag
    plot(cTag(indTag,1),cTag(indTag,2),'rh')
end
legend('odo ruote','odo robot','','','','')
title(['Robot' num2str(robot)])

%% Misure UWB robot-landmark

matrice = roslam_data.uwb_anchors_distances{robot,1};
tempiMisure = matrice(:,1);
landmark = 4;
misureRobotLandmark = 100*matrice(:,landmark+1)+10;
nMisure = numel(tempiMisure);
misureAtteseRobotLandmarkVett = zeros(nMisure,1);
misureAtteseRobotLandmarkOdo = zeros(nMisure,1);
for indMisura = 1:nMisure
    [mm,indMin] = min(abs(tempiMisure(indMisura)-tempi));
    misureAtteseRobotLandmarkVett(indMisura) = sqrt((xVett(indMin)-cTag(landmark,1))^2+(yVett(indMin)-cTag(landmark,2))^2);
end
for indMisura = 1:nMisure
    [mm,indMin] = min(abs(tempiMisure(indMisura)-tempiOdo));
    misureAtteseRobotLandmarkOdo(indMisura) = sqrt((xOdo(indMin)-cTag(landmark,1))^2+(yOdo(indMin)-cTag(landmark,2))^2);
end

distanzeFinaliRobotLandmark = 100*[1.14, 2.27, 3.20, 2.59; 2.05, 2.01, 2.02, 2.21; 2.59, 2.59, 1.52, 1.68];
xCentri = cTag(:,1);
yCentri = cTag(:,2);
raggiCerchi = distanzeFinaliRobotLandmark(robot,:)';
[xRobotFinale,yRobotFinale,dummy] = trovaPosTag(xCentri,yCentri,raggiCerchi);
misuraAttesaFinaleVera = sqrt((xRobotFinale-cTag(landmark,1))^2+(yRobotFinale-cTag(landmark,2))^2);

figure
plot(misureRobotLandmark)
hold on
plot(misureAtteseRobotLandmarkVett,'r')
plot(misureAtteseRobotLandmarkOdo,'g')
plot(misuraAttesaFinaleVera*ones(numel(misureRobotLandmark),1),'k:')
legend('vere','attese odo ruote')%,'attese odo robot')
title(['R' num2str(robot) '-L' num2str(landmark)])