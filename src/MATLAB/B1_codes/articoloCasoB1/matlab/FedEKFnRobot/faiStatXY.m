nSim = 100;
nTag = 4;
nPassi = 500;

erroreAssolutoRobotXVett = zeros(nSim,1);
erroreAssolutoRobotYVett = zeros(nSim,1);

for indSim = 1:nSim

    indSim

    main

    indRobot = 1;
    erroreAssolutoRobotXVett(indSim) = robots(indRobot).xHatSLAM(1,k)-robots(indRobot).xVett(k);
    erroreAssolutoRobotYVett(indSim) = robots(indRobot).xHatSLAM(2,k)-robots(indRobot).yVett(k);

end

figure
plot(erroreAssolutoRobotXVett,erroreAssolutoRobotYVett,'.')
grid on
