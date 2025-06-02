for robot = 1:nRobot
    figure
    % for indTag = 1:nTag
    %     text(cTag(indTag,1) + 0.2, cTag(indTag,2), num2str(indTag), 'FontSize', 12, 'Color', 'red');
    %     plot(cTag(indTag,1), cTag(indTag,2), 'rh', 'Linewidth', 2, 'MarkerSize', 12);
    % end

    end_test = size(roslam_data.wheels_odometry{robot}, 1);

    xVett = roslam_data.ground_truth(robot).robots_poses_world_history(:, 1, 1);
    yVett = roslam_data.ground_truth(robot).robots_poses_world_history(:, 1, 2);

    plot(xVett, yVett, ':', 'Linewidth', 1)    % traiettoria
    hold on
    plot(xVett(1), yVett(1), 'k^', 'Linewidth', 2)  % pos iniziale
    plot(xVett(end), yVett(end), 'ks', 'Linewidth', 2) % pos finale

    title(sprintf('Robot %d, step %d', robot, num2str(k)));
    % plot(xVett(max(1,k-30):k), yVett(max(1,k-30):k), 'k', 'LineWidth', 2)   % traccia
    % plot(xVett(k), yVett(k), 'ko')  % pos attuale vera

    posLoc = [ekfs(robot).xHatSLAM(1:2, end_test); 1];
    posGlob = TsGL{robot}(:, :, end)*posLoc;
    plot(posGlob(1), posGlob(2), 'o') % pos attuale stimata

    for indTag = 1:nTag
        ind0 = ekfs(robot).xHatCumIndices(indTag+1);

        x_i   = ekfs(robot).xHatSLAM(0+ind0, end_test);
        y_i   = ekfs(robot).xHatSLAM(1+ind0, end_test);
        rho_i = ekfs(robot).xHatSLAM(2+ind0, end_test); % invariante
        x_ti = 0;
        y_ti = 0;

        nPhi = ekfs(robot).nPhiVett(indTag);
        for indPhi = 1:nPhi
            phi_ij = ekfs(robot).xHatSLAM(2+ind0+indPhi, end_test);
            cosPhi_ij = cos(phi_ij);
            sinPhi_ij = sin(phi_ij);
            xTag_ij = x_i + rho_i*cosPhi_ij;
            yTag_ij = y_i + rho_i*sinPhi_ij;
            x_ti = x_ti + xTag_ij*ekfs(robot).pesi(indTag, indPhi);
            y_ti = y_ti + yTag_ij*ekfs(robot).pesi(indTag, indPhi);
            posLoc = [xTag_ij, yTag_ij, 1]';
            posGlob = TsGL{robot}(:, :, end)*posLoc;
            plot(posGlob(1), posGlob(2), 'b.', 'MarkerSize', max(7, ceil(20*ekfs(robot).pesi(indTag,indPhi))))
            text(posGlob(1) - 0.2, posGlob(2), num2str(indTag), 'FontSize', 12);
        end
        posLoc = [x_ti, y_ti, 1]';
        posGlob = TsGL{robot}(:, :, end)*posLoc;
        plot(posGlob(1), posGlob(2), '*') % posizione media stimata tag
    end

    axis equal
    grid on
end