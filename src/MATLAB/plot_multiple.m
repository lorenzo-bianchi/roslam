% 'erroreAssolutoRobotTot',
% 'erroriAssolutiTagTot',
% 'distanzeRobotVereTot', 'distanzeRobotStimateTot',
% 'distanzeInterTagVereTot', 'distanzeInterTagStimateTot',
% 'erroriMediPostICPTot'

clc; clear; close all

font_size = 16;
line_width = 1;

%%
folder_path = "data/";
filenames_tot = { ... {"sims_100_pruning_100_no_reset.mat", ...
                   ... "sims_100_pruning_100_reset.mat"}, ...
                   ... ...
                      {"sims_100_8_hyp_no_reset.mat", ...
                       "sims_100_8_hyp_reset.mat"}, ...
                      ...
                      {"sims_100_8_hyp_pruning_100_no_reset.mat", ...
                       "sims_100_8_hyp_pruning_100_reset.mat"}, ...
              ...
                      {"sims_100_8_hyp_sigma_up_pruning_100_no_reset.mat", ...
                       "sims_100_8_hyp_sigma_up_pruning_100_reset.mat"}, ...
              ...
                      {"sims_100_8_hyp_sigma_up_no_reset.mat", ...
                       "sims_100_8_hyp_sigma_up_reset.mat"}, ...
              ...
                      {"sims_100_8_hyp_sigma2_up_no_reset.mat", ... 
                       "sims_100_8_hyp_sigma2_up_reset.mat"}, ...
              ...
                      {"sims_100_no_reset.mat", ...
                       "sims_100_reset.mat"}, ...
                    % {"sims_100_8_hyp_sigma2_upup_no_reset.mat"}
                };

for n = 1:3
    if n == 1
        filenames = filenames_tot(1:3);
        pdf_name = './figures/comparison_100_sims_first.pdf';
    elseif n == 2
        filenames = filenames_tot(4:6);
        pdf_name = './figures/comparison_100_sims_second.pdf';
    elseif n == 3
        filenames = filenames_tot;
        pdf_name = './figures/comparison_100_sims_total.pdf';
    end

    %%
    n_cols = length(filenames);
    
    if n == 3
        figure('Position', [100, 100, 1800, 1500]);
    else
        figure('Position', [100, 100, 880, 1500]);
    end
    tiledlayout(5, n_cols , 'TileSpacing', 'compact', 'Padding', 'tight');
    
    for k = 1:n_cols
        filename_no_reset = filenames{k}{1};
        filename_reset = filenames{k}{2};
        fprintf("%s, %s\n", filename_no_reset, filename_reset)
        
        no_reset = load(folder_path + filename_no_reset);
        reset = load(folder_path + filename_reset);
    
        %% Errori assoluti robot
        single_abs_robot = no_reset.erroreAssolutoRobotTot(1:2:end);
        multi_abs_robot_no_reset = no_reset.erroreAssolutoRobotTot(2:2:end);
        multi_abs_robot_reset = reset.erroreAssolutoRobotTot(2:2:end);
        
        avg_single_abs_robot = zeros(1, 100);
        avg_multi_abs_robot_no_reset = zeros(1, 100);
        avg_multi_abs_robot_reset = zeros(1, 100);
        
        for i = 1:100
            avg_single_abs_robot(i) = mean(single_abs_robot{i});
            avg_multi_abs_robot_no_reset(i) = mean(multi_abs_robot_no_reset{i});
            avg_multi_abs_robot_reset(i) = mean(multi_abs_robot_reset{i});
        end
        
        sorted_avg_single_abs_robot = sort(avg_single_abs_robot);
        sorted_avg_multi_abs_robot_no_reset = sort(avg_multi_abs_robot_no_reset);
        sorted_avg_multi_abs_robot_reset = sort(avg_multi_abs_robot_reset);
        
        nexttile(k)
        plot(1:100, sorted_avg_single_abs_robot, 'LineWidth', line_width)
        grid on
        hold on
        plot(1:100, sorted_avg_multi_abs_robot_no_reset, 'LineWidth', line_width)
        plot(1:100, sorted_avg_multi_abs_robot_reset, 'LineWidth', line_width)
        xlim([-5 105])
        ylim([0 7])
        if (n == 3 && k == 1) || (n == 1 && k == 1)
            col_title = "$\mathbf{n_\phi=8, SP=10, \sigma_S=0.2, \sigma_R=0.2}$";
        elseif (n == 3 && k == 2) || (n == 1 && k == 2)
            col_title = "$\mathbf{n_\phi=8, SP=100, \sigma_S=0.2, \sigma_R=0.2}$";
        elseif (n == 3 && k == 3) || (n == 1 && k == 3)
            col_title = "$\mathbf{n_\phi=8, SP=100, \sigma_S=0.4, \sigma_R=0.2}$";
        elseif (n == 3 && k == 4) || (n == 2 && k == 1)
           col_title = "$\mathbf{n_\phi=8, SP=10, \sigma_S=0.4, \sigma_R=0.2}$";
        elseif (n == 3 && k == 5) || (n == 2 && k == 2)
            col_title = "$\mathbf{n_\phi=8, SP=10, \sigma_S=0.2, \sigma_R=0.4}$";
        elseif (n == 3 && k == 6) || (n == 2 && k == 3)
            col_title = "$\mathbf{n_\phi=16, SP=10, \sigma_S=0.2, \sigma_R=0.2}$";
        end
        title(col_title, 'Interpreter', 'latex')
    
        if k == 1
            legend('single robot', 'multi robot (no reset)', 'multi robot (reset)', 'Location', 'northwest')
            ylabel('robot abs. error [m]');
        end
        
        if k == 1
            set(gca, 'YTickLabel', compose('%.1f', yticks));
        end
        if k ~= 1
            set(gca, 'YTickLabel', []);
        end
        set(gca, 'XTickLabel', []);
        
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
        
        %% Errori assoluti robot post ICP
        single_icp_robot = no_reset.erroriMediPostICPTot(1:2:end);
        multi_icp_robot_no_reset = no_reset.erroriMediPostICPTot(2:2:end);
        multi_icp_robot_reset = reset.erroriMediPostICPTot(2:2:end);
        
        avg_single_icp_robot = zeros(1, 100);
        avg_multi_icp_robot_no_reset = zeros(1, 100);
        avg_multi_icp_robot_reset = zeros(1, 100);
        
        for i = 1:100
            avg_single_icp_robot(i) = mean(single_icp_robot{i});
            avg_multi_icp_robot_no_reset(i) = mean(multi_icp_robot_no_reset{i});
            avg_multi_icp_robot_reset(i) = mean(multi_icp_robot_reset{i});
        end
        
        sorted_avg_single_icp_robot = sort(avg_single_icp_robot);
        sorted_avg_multi_icp_robot_no_reset = sort(avg_multi_icp_robot_no_reset);
        sorted_avg_multi_icp_robot_reset = sort(avg_multi_icp_robot_reset);
        
        nexttile(k+n_cols)
        plot(1:100, sorted_avg_single_icp_robot, 'LineWidth', line_width)
        grid on
        hold on
        plot(1:100, sorted_avg_multi_icp_robot_no_reset, 'LineWidth', line_width)
        plot(1:100, sorted_avg_multi_icp_robot_reset, 'LineWidth', line_width)
        xlim([-5 105])
        ylim([0 0.45])
        % legend('single robot', 'multi robot (no reset)', 'multi robot (reset)', 'Location', 'northwest')
        if k == 1
            ylabel('robot abs. err. post AERIS [m]');
            % set(gca, 'YTickLabel', compose('%.1f', yticks));
        end
        if k ~= 1
            set(gca, 'YTickLabel', []);
        end
        set(gca, 'XTickLabel', []);
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
        
        %% Errori assoluti tag
        single_abs_tag = no_reset.erroriAssolutiTagTot(1:2:end);
        multi_abs_tag_no_reset = no_reset.erroriAssolutiTagTot(2:2:end);
        multi_abs_tag_reset = reset.erroriAssolutiTagTot(2:2:end);
        
        rmse_single_abs_tag = zeros(1, 100);
        rmse_multi_abs_tag_no_reset = zeros(1, 100);
        rmse_multi_abs_tag_reset = zeros(1, 100);
        
        for i = 1:100
            temp = single_abs_tag{i};
            rmse_single_abs_tag(i) = sqrt(mean(temp(:).^2));
        
            temp = multi_abs_tag_no_reset{i};
            rmse_multi_abs_tag_no_reset(i) = sqrt(mean(temp(:).^2));
    
            temp = multi_abs_tag_reset{i};
            rmse_multi_abs_tag_reset(i) = sqrt(mean(temp(:).^2));
        end
        
        sorted_rmse_single_abs_tag = sort(rmse_single_abs_tag);
        sorted_rmse_multi_abs_tag_no_reset = sort(rmse_multi_abs_tag_no_reset);
        sorted_rmse_multi_abs_tag_reset = sort(rmse_multi_abs_tag_reset);
        
        nexttile(k+n_cols*2)
        plot(1:100, sorted_rmse_single_abs_tag, 'LineWidth', line_width)
        grid on
        hold on
        plot(1:100, sorted_rmse_multi_abs_tag_no_reset, 'LineWidth', line_width)
        plot(1:100, sorted_rmse_multi_abs_tag_reset, 'LineWidth', line_width)
        xlim([-5 105])
        ylim([0 8])
        % legend('single robot', 'multi robot (no reset)', 'multi robot (reset)', 'Location', 'northwest')
        if k == 1
            ylabel('lndmk abs. errors [m]');
            set(gca, 'YTickLabel', compose('%.1f', yticks));
        end
        if k ~= 1
            set(gca, 'YTickLabel', []);
        end
        set(gca, 'XTickLabel', []);
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
        
        %% Distanze tag-tag{1}
        single_est_tt = no_reset.distanzeInterTagStimateTot(1:2:end);
        multi_est_tt_no_reset = no_reset.distanzeInterTagStimateTot(2:2:end);
        multi_est_tt_reset = reset.distanzeInterTagStimateTot(2:2:end);
        
        true_tt = reset.distanzeInterTagVereTot(1:2:end);
        
        rmse_single_tt = zeros(1, 100);
        rmse_multi_tt_no_reset = zeros(1, 100);
        rmse_multi_tt_reset = zeros(1, 100);
        
        for i = 1:100
            temp = single_est_tt{i} - true_tt{i};
            rmse_single_tt(i) = sqrt(mean(temp(:).^2));
        
            temp = multi_est_tt_no_reset{i} - true_tt{i};
            rmse_multi_tt_no_reset(i) = sqrt(mean(temp(:).^2));
    
            temp = multi_est_tt_reset{i} - true_tt{i};
            rmse_multi_tt_reset(i) = sqrt(mean(temp(:).^2));
        end
        
        sorted_rmse_single_tt = sort(rmse_single_tt);
        sorted_rmse_multi_tt_no_reset = sort(rmse_multi_tt_no_reset);
        sorted_rmse_multi_tt_reset = sort(rmse_multi_tt_reset);
        
        nexttile(k+n_cols*3)
        plot(1:100, sorted_rmse_single_tt, 'LineWidth', line_width)
        grid on
        hold on
        plot(1:100, sorted_rmse_multi_tt_no_reset, 'LineWidth', line_width)
        plot(1:100, sorted_rmse_multi_tt_reset, 'LineWidth', line_width)
        xlim([-5 105])
        ylim([0 2.5])
        % legend('single robot', 'multi robot (no reset)', 'multi robot (reset)', 'Location', 'northwest')
        if k == 1
            ylabel('RMSE lndmk-lndmk [m]');
            set(gca, 'YTickLabel', compose('%.1f', yticks));
        end
        if k ~= 1
            set(gca, 'YTickLabel', []);
        end
        set(gca, 'XTickLabel', []);
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
        
        %% Distanze robot-tag
        single_est_rt = no_reset.distanzeRobotStimateTot(1:2:end);
        multi_est_rt_no_reset = no_reset.distanzeRobotStimateTot(2:2:end);
        multi_est_rt_reset = reset.distanzeRobotStimateTot(2:2:end);
        
        true_rt = reset.distanzeRobotVereTot(1:2:end);
        
        rmse_single_rt = zeros(1, 100);
        rmse_multi_rt_no_reset = zeros(1, 100);
        rmse_multi_rt_reset = zeros(1, 100);
        
        for i = 1:100
            temp = single_est_rt{i} - true_rt{i};
            rmse_single_rt(i) = sqrt(mean(temp(:).^2));
        
            temp = multi_est_rt_no_reset{i} - true_rt{i};
            rmse_multi_rt_no_reset(i) = sqrt(mean(temp(:).^2));
    
            temp = multi_est_rt_reset{i} - true_rt{i};
            rmse_multi_rt_reset(i) = sqrt(mean(temp(:).^2));
        end
        
        sorted_rmse_single_rt = sort(rmse_single_rt);
        sorted_rmse_multi_rt_no_reset = sort(rmse_multi_rt_no_reset);
        sorted_rmse_multi_rt_reset = sort(rmse_multi_rt_reset);
    
        nexttile(k+n_cols*4)
        plot(1:100, sorted_rmse_single_rt, 'LineWidth', line_width)
        grid on
        hold on
        plot(1:100, sorted_rmse_multi_rt_no_reset, 'LineWidth', line_width)
        plot(1:100, sorted_rmse_multi_rt_reset, 'LineWidth', line_width)
        xlim([-5 105])
        ylim([0 1.5])
        % legend('single robot', 'multi robot (no reset)', 'multi robot (reset)', 'Location', 'northwest')
        xlabel('simulation');
        if k == 1
            ylabel('RMSE robot-lndmk [m]');
            set(gca, 'YTickLabel', compose('%.1f', yticks));
        end
        if k ~= 1
            set(gca, 'YTickLabel', []);
        end
        set(gca, 'FontSize', font_size, 'FontWeight', 'bold');
    end

    exportgraphics(gcf, pdf_name, 'ContentType', 'vector');
end