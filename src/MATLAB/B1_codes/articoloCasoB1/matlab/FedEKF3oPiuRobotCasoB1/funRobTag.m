function Delta = funRobTag(x,paramCostanti,misuraRangeTraRobot,misuraRangeTagRobot)

% XRL0 = [xRHat0(3:nRobot); yRHat0(3:nRobot); xHatTag0; yHatTag0; alfaR1R2];
% paramCostanti = [xRHat0(1); yRHat0(1); xR2; yR2];

[nRobot,nTag] = size(misuraRangeTagRobot);

Delta = zeros(nRobot*(nRobot-1)/2+nRobot*nTag,1);

xR1 = paramCostanti(1);
yR1 = paramCostanti(2);
xR2 = paramCostanti(3);
yR2 = paramCostanti(4);
nRob3 = nRobot - 2;

xR2Hat0 = xR1 + x(end)*(xR2-xR1);
yR2Hat0 = yR1 + x(end)*(yR2-yR1);
xRHat0 = [xR1; xR2Hat0; x(1:nRob3)]; 
yRHat0 = [yR1; yR2Hat0; x(nRob3+1:2*nRob3)];
xHatTag0 = x(2*nRob3+1:2*nRob3+nTag);
yHatTag0 = x(2*nRob3+nTag+1:2*nRob3+2*nTag);

indiceDelta = 1;

for indRobot = 1:nRobot-1
    for jndRobot = indRobot+1:nRobot
        dRiRj = sqrt((xRHat0(indRobot)-xRHat0(jndRobot))^2+(yRHat0(indRobot)-yRHat0(jndRobot))^2);
        Delta(indiceDelta) = (dRiRj-misuraRangeTraRobot(indRobot,jndRobot));
        indiceDelta = indiceDelta + 1;
    end
end

for indRobot = 1:nRobot
    for jndTag = 1:nTag
        dRiTj = sqrt((xRHat0(indRobot)-xHatTag0(jndTag))^2+(yRHat0(indRobot)-yHatTag0(jndTag))^2);
        Delta(indiceDelta) = (dRiTj-misuraRangeTagRobot(indRobot,jndTag));
        indiceDelta = indiceDelta + 1;
    end
end