load roslam_data3.mat

cTag = [0, 0;
    1.7903, 0;
    1.7241, 3.6934;
    -0.1471, 3.7211]*100;
nTag = 4;
dVera = roslam_data.wheels_separation*100;

figure

for robot = 1:3

    odoRuote = roslam_data.wheels_odometry{robot};
    tempi = odoRuote(:,1);
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

    subplot(1,3,robot)
    disegnaFig
    plot(xVett/100,yVett/100,'k')
    plot(xVett(1)/100,yVett(1)/100,'sk')
    plot(xVett(end)/100,yVett(end)/100,'ok')
    % Posizione finale vera
    if robot == 1
        plot(-18.2/100,112.7/100,'pb')
    end
    if robot == 2
        plot(95.8/100,181.9/100,'pb')
    end
    if robot == 3
        plot(90.4/100,242.3/100,'pb')
    end
    title(['Robot' num2str(robot)])

end

% Odometry reconstruction of three robot trajectories from their true
% measured initial position (square marker): the final reconstructed 
% position (blue star marker) does not coincide with the true measured
% position (circle marker) of the robots.