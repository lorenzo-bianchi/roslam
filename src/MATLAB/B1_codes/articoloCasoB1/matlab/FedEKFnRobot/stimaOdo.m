indRobot = 1;

xOdo = zeros(nPassi,1);
yOdo = zeros(nPassi,1);
thetaOdo = zeros(nPassi,1);

xOdo(1) = robots(indRobot).xVett(1);
yOdo(1) = robots(indRobot).yVett(1);
thetaOdo(1) = robots(indRobot).thetaVett(1);

for k = 1:nPassi-1
    
    uk = ((robots(indRobot).uRe(k)+robots(indRobot).uLe(k))/2);
    omegak = ((robots(indRobot).uRe(k)-robots(indRobot).uLe(k))/d);
    
    xOdo(k+1) = xOdo(k) + uk*cos(thetaOdo(k));
    yOdo(k+1) = yOdo(k) + uk*sin(thetaOdo(k));
    thetaOdo(k+1) = thetaOdo(k) + omegak;
    
end

erroreOdo = zeros(nPassi,1);
for k = 1:nPassi
    erroreOdo(k) = sqrt((robots(indRobot).xVett(k)-xOdo(k))^2+(robots(indRobot).yVett(k)-yOdo(k))^2);
end
figure
plot(erroreOdo)
figure
plot(xOdo/100,yOdo/100,'m:')
hold on
plot(robots(indRobot).xVett/100,robots(indRobot).yVett/100,'k')
