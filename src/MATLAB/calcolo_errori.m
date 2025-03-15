%% Calcolo distanze vere e stimate tag-tag e tag-robot (errori SLAM relativi)
erroreAssolutoRobot = zeros(1, nRobot);
distanzeRobotVere = zeros(nRobot, nTag);
distanzeRobotStimate = zeros(nRobot, nTag);
distanzeInterTagVere = zeros(nRobot, nTag*(nTag-1)/2);
distanzeInterTagStimate = zeros(nRobot, nTag*(nTag-1)/2);
erroriAssolutiTag = zeros(nRobot, nTag);
erroriMediPostICP = zeros(1, nRobot);
erroriMediTagGlobali = zeros(1, nRobot);
erroriPosGlobale = zeros(1, nRobot);

for robot = 1:nRobot
    xVett = percorsi(:, 1, robot);
    yVett = percorsi(:, 2, robot);

    distanzeRobotVere(robot, :) = sqrt((xVett(k)-cTag(:,1)).^2+(yVett(k)-cTag(:,2)).^2)';

    xRobotEnd = ekfs(robot).xHatSLAM(1, end);
    yRobotEnd = ekfs(robot).xHatSLAM(2, end);
    xHatTagEnd = ekfs(robot).xHatTagStoria(:, end);
    yHatTagEnd = ekfs(robot).yHatTagStoria(:, end);
    distanzeRobotStimate(robot, :) = sqrt((xRobotEnd-xHatTagEnd).^2+(yRobotEnd-yHatTagEnd).^2);
    
    indice = 0;
    for indTag = 1:nTag-1
        for jndTag = indTag+1:nTag
            indice = indice + 1;
            distanzeInterTagVere(robot, indice) = sqrt((cTag(indTag,1)-cTag(jndTag,1)).^2+(cTag(indTag,2)-cTag(jndTag,2)).^2);
            distanzeInterTagStimate(robot, indice) = sqrt((xHatTagEnd(indTag)-xHatTagEnd(jndTag)).^2+(yHatTagEnd(indTag)-yHatTagEnd(jndTag)).^2);
        end
    end

    posHatTagLoc = [xHatTagEnd'; yHatTagEnd'; ones(1, nTag)];
    posHatTagGlob = (TsGL{robot}(:, :, end)*posHatTagLoc)';
    
    erroriAssolutiTag(robot, :) = sqrt((posHatTagGlob(:, 1)-cTag(:, 1)).^2+(posHatTagGlob(:, 2)-cTag(:, 2)).^2);

    posRobLoc = [xRobotEnd; yRobotEnd; 1];
    posRobGlob = TsGL{robot}(:, :, end)*posRobLoc;
    erroreAssolutoRobot(robot) = sqrt((posRobGlob(1)-xVett(k))^2+(posRobGlob(2)-yVett(k))^2);

    robot_tags = [ekfs(robot).xHatTagStoria(:, end)'; ekfs(robot).yHatTagStoria(:, end)']';
    [RR, tt, inl] = ransacRototranslation(robot_tags, cTag, data.numIterationsA, 0.25, 5);
    if isempty(inl)
        erroriMediPostICP(robot) = nan;
    else
        TT = [RR, tt; zeros(1, 2), 1];
        temp = TT*[robot_tags'; ones(1, nTag)];
        temp = temp(1:2, :);
        erroriMediPostICP(robot) = mean(vecnorm(cTag' - temp));
    end

    if pruning && sharing
        ids = global_poses(robot).ids;
        point1 = cTag(ids(1), :);
        point2 = cTag(ids(2), :);

        t = point1;
        s = (point2(2)-point1(2)) / norm(point2-point1);
        c = (point2(1)-point1(1)) / norm(point2-point1);
        T = [c -s t(1); s c t(2); 0 0 1]^-1;

        transformed_tags = T * [cTag'; ones(1, nTag)];

        delta = transformed_tags(1:2, :) - global_poses(robot).tags;
        global_errors_tags = sqrt(delta(1, :).^2 + delta(2, :).^2);

        mask = true(size(global_errors_tags));
        mask(ids) = false;
        erroriMediTagGlobali(robot) = mean(global_errors_tags(mask));

        transformed_pos = T * [percorsi(k, 1:2, robot)'; 1];
        erroriPosGlobale(robot) = norm(transformed_pos(1:2) - global_poses(robot).pose(1:2, :));
    end
    
    if displayErrori
        fprintf("Robot %d:\n", robot);
        % fprintf("\tDistanze robot-tag vere: ")
        % fprintf("%.3f ", distanzeRobotVere(robot, :));
        % fprintf("\n");
        % fprintf("\tDistanze robot-tag stimate: ")
        % fprintf("%.3f ", distanzeRobotStimate(robot, :));
        % fprintf("\n");
        fprintf("\tDifferenza distanze robot-tag: ")
        fprintf("%.3f ", abs(distanzeRobotVere(robot, :) - distanzeRobotStimate(robot, :)));
        fprintf("\n");
        fprintf("\tMedia differenza distanze robot-tag: ")
        fprintf("%.3f ", mean(abs(distanzeRobotVere(robot, :) - distanzeRobotStimate(robot, :))));
        fprintf("\n");
        % fprintf("\tDistanze tag-tag vere: ")
        % fprintf("%.3f ", distanzeInterTagVere(robot, :));
        % fprintf("\n");
        % fprintf("\tDistanze tag-tag stimate: ")
        % fprintf("%.3f ", distanzeInterTagStimate(robot, :));
        % fprintf("\n");
        % fprintf("\tDifferenza distanze tag-tag: ")
        % fprintf("%.3f ", abs(distanzeInterTagVere(robot, :) - distanzeInterTagStimate(robot, :)));
        % fprintf("\n");
        fprintf("\tMedia differenza distanze tag-tag: ")
        fprintf("%.3f ", mean(abs(distanzeInterTagVere(robot, :) - distanzeInterTagStimate(robot, :))));
        fprintf("\n");
        % fprintf("\tErrori assoluti tag: ")
        % fprintf("%.3f ", erroriAssolutiTag(robot, :));
        % fprintf("\n");
        fprintf("\tMedia errori assoluti tag: ")
        fprintf("%.3f ", mean(erroriAssolutiTag(robot, :)));
        fprintf("\n");
        fprintf("\tErrore assoluto robot: ")
        fprintf("%.3f ", erroreAssolutoRobot(robot));
        fprintf("\n\n");
        fprintf("\tMedia distanze tag post ICP: ")
        fprintf("%.3f ", erroriMediPostICP);
        fprintf("\n\n");
        if pruning && sharing
            fprintf("\tErrori tag post globalize: ")
            fprintf("%.3f ", global_errors_tags);
            fprintf("\n");
            fprintf("\tErrore robot post globalize: ")
            fprintf("%.3f ", erroriPosGlobale(robot));
            fprintf("\n");
        end
    end
end