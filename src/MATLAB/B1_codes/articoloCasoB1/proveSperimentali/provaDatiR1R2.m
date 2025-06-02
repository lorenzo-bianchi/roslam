% load roslam_data3.mat

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]*100;
nTag = 4;
dVera = roslam_data.wheels_separation*100;
      
robot1 = 1;
robot2 = 3; % Scegliere robot2 > robot1

odoRuote1 = roslam_data.wheels_odometry{robot1};
tempi1 = odoRuote1(:,1);
tempiOdo1 = roslam_data.robot_odometry(robot1).times;
uRe1 = 100*odoRuote1(:,2);
uLe1 = 100*odoRuote1(:,3);
nPassi1 = numel(tempi1);
xVett1 = zeros(nPassi1+1,1);
yVett1 = zeros(nPassi1+1,1);
thetaVett1 = zeros(nPassi1+1,1);
if robot1 == 1
    xVett1(1) =  0.0905*100;
    yVett1(1) = 2.6885*100;
end
if robot1 == 2
    xVett1(1) =  0.0605*100;
    yVett1(1) = 1.6437*100;
    thetaVett1(1) = -pi/2;
end
if robot1 == 3
    xVett1(1) =  1.7309*100;
    yVett1(1) = 0.7688*100;
    thetaVett1(1) = pi/2;
end

odoRuote2 = roslam_data.wheels_odometry{robot2};
tempi2 = odoRuote2(:,1);
tempiOdo2 = roslam_data.robot_odometry(robot2).times;
uRe2 = 100*odoRuote2(:,2);
uLe2 = 100*odoRuote2(:,3);
nPassi2 = numel(tempi2);
xVett2 = zeros(nPassi2+1,1);
yVett2 = zeros(nPassi2+1,1);
thetaVett2 = zeros(nPassi2+1,1);
if robot2 == 1
    xVett2(1) =  0.0905*100;
    yVett2(1) = 2.6885*100;
end
if robot2 == 2
    xVett2(1) =  0.0605*100;
    yVett2(1) = 1.6437*100;
    thetaVett2(1) = -pi/2;
end
if robot2 == 3
    xVett2(1) =  1.7309*100;
    yVett2(1) = 0.7688*100;
    thetaVett2(1) = pi/2;
end

for k = 1:nPassi1
    deltaRho = (uRe1(k)+uLe1(k))/2;
    deltaTheta = (uRe1(k)-uLe1(k))/dVera;
    xVett1(k+1) = xVett1(k) + deltaRho*cos(thetaVett1(k));
    yVett1(k+1) = yVett1(k) + deltaRho*sin(thetaVett1(k));
    thetaVett1(k+1) = thetaVett1(k) + deltaTheta;
end

xOdo = roslam_data.robot_odometry(robot1).x;
yOdo = roslam_data.robot_odometry(robot1).y;
thetaRot = thetaVett1(1);
cTheta = cos(thetaRot);
sTheta = sin(thetaRot);
xOdoRot = cTheta*xOdo-sTheta*yOdo;
yOdoRot = sTheta*xOdo+cTheta*yOdo;
xOdo1 = 100*xOdoRot + xVett1(1);
yOdo1 = 100*yOdoRot + yVett1(1);

for k = 1:nPassi2
    deltaRho = (uRe2(k)+uLe2(k))/2;
    deltaTheta = (uRe2(k)-uLe2(k))/dVera;
    xVett2(k+1) = xVett2(k) + deltaRho*cos(thetaVett2(k));
    yVett2(k+1) = yVett2(k) + deltaRho*sin(thetaVett2(k));
    thetaVett2(k+1) = thetaVett2(k) + deltaTheta;
end

xOdo = roslam_data.robot_odometry(robot2).x;
yOdo = roslam_data.robot_odometry(robot2).y;
thetaRot = thetaVett2(1);
cTheta = cos(thetaRot);
sTheta = sin(thetaRot);
xOdoRot = cTheta*xOdo-sTheta*yOdo;
yOdoRot = sTheta*xOdo+cTheta*yOdo;
xOdo2 = 100*xOdoRot + xVett2(1);
yOdo2 = 100*yOdoRot + yVett2(1);

figure
plot(xVett1,yVett1)
hold on
plot(xOdo1,yOdo1,'g')
for indTag = 1:nTag
    plot(cTag(indTag,1),cTag(indTag,2),'rh')
end
plot(xVett2,yVett2,':')
plot(xOdo2,yOdo2,'g:')

%% Misure UWB robot-robot

% matrice = roslam_data.uwb_inter_robots_distances{1,1};  
% tempiMisureR1R2 = matrice(:,1);
% tempiMisureR1R3 = matrice(:,1);
% misureR1R2 = 100*matrice(:,3);
% misureR1R3 = 100*matrice(:,4);
% 
% matrice = roslam_data.uwb_inter_robots_distances{2,1};  
% tempiMisureR2R3 = matrice(:,1);
% misureR2R3 = 100*matrice(:,4);

if robot1 == 1 && robot2 == 2
    matrice = roslam_data.uwb_inter_robots_distances{1,1};  
    tempiMisure = matrice(:,1);
    nMisure = numel(tempiMisure);
    misureInterRobot = 100*matrice(:,3);
end
if robot1 == 1 && robot2 == 3
    matrice = roslam_data.uwb_inter_robots_distances{1,1};  
    tempiMisure = matrice(:,1);
    nMisure = numel(tempiMisure);
    misureInterRobot = 100*matrice(:,4);
end
if robot1 == 2 && robot2 == 3
    matrice = roslam_data.uwb_inter_robots_distances{2,1};  
    tempiMisure = matrice(:,1);
    nMisure = numel(tempiMisure);
    misureInterRobot = 100*matrice(:,4);
end

% misureInterRobot = misureInterRobot - 15;

misureAtteseInterRobotVett = zeros(nMisure,1);
misureAtteseInterRobotOdo = zeros(nMisure,1);
for indMisura = 1:nMisure
    [mm,indMin1] = min(abs(tempiMisure(indMisura)-tempi1));
    [mm,indMin2] = min(abs(tempiMisure(indMisura)-tempi2));
    misureAtteseInterRobotVett(indMisura) = sqrt((xVett1(indMin1)-xVett2(indMin2))^2+(yVett1(indMin1)-yVett2(indMin2))^2);
end
for indMisura = 1:nMisure
    [mm,indMin1] = min(abs(tempiMisure(indMisura)-tempiOdo1));
    [mm,indMin2] = min(abs(tempiMisure(indMisura)-tempiOdo2));
    misureAtteseInterRobotOdo(indMisura) = sqrt((xOdo1(indMin1)-xOdo2(indMin2))^2+(yOdo1(indMin1)-yOdo2(indMin2))^2);
end

figure
plot(misureInterRobot)
hold on
plot(misureAtteseInterRobotVett,'r')
plot(misureAtteseInterRobotOdo,'g')
legend('vere','attese odo ruote','attese odo robot')
title(['R' num2str(robot1) '-R' num2str(robot2)])