figure
disegnaFig
for indRobot = 1:nRobot
    plot(robots(indRobot).xVett(1)/100,robots(indRobot).yVett(1)/100,'ko')
end
axis([0 2 -0 2])

% Posizione 0 robot
for indRobot = 1:nRobot
    plot(xRHat0fin(indRobot)/100,yRHat0fin(indRobot)/100,'mo')
end

% Stima posizione tag
for indTag = 1:nTag
    plot(xHatTag0fin/100,yHatTag0fin/100,'m*') 
end