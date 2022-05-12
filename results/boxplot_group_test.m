% This script is used to create 2x3 plot, each plot is for each
% trasnformation. In each plot divided into number of noise, and each noise
% divided into number of algorithm.

clear; close all;
addpath(genpath('..\functions\display'));

% specify source
sourcepath = 'backup\amode_new\tibia\trials3';
resultpath = 'pictures\tibia\algorithm_comparison';

% specify the filenames and the name of the algorithm
% filenames  = {'ukfnormal_15_trials2', 'ukfnormal_20_trials2', 'ukfnormal_25_trials2', 'ukfnormal_30_trials2'};
% alg_names  = {'15', '20', '25', '30'};
% filenames  = {'icp_15_trials2', 'cpd_15_trials2', 'ukf_15_trials2', 'goicp_15_trials2_b', 'icpnormal_15_trials2', 'ukfnormal_15_trials2'};
% alg_names  = {'ICP', 'CPD', 'UKF', 'GOICP', 'ICP+norm', 'UKF+norm'};
filenames  = {'icp_15_trials3', 'cpd_15_trials0', 'ukf_15_trials3', 'goicp_15_trials3', 'ukfnormal_15_trials3'};
alg_names  = {'ICP', 'CPD', 'UKF', 'GOICP', 'UKF+norm'};

% for visualization purpose
colorpalette = {'#57606f', '#5352ed', '#70a1ff', '#2ed573', '#ffa502', '#ff4757'};

% storing some variable
total_algorithms = length(filenames);
total_dof        = 6;

%% Preparing Data

% rearrange data, the requirement for boxplotGroup, please refer
% https://www.mathworks.com/matlabcentral/answers/331381-how-do-you-create-a-grouped-boxplot-with-categorical-variables-on-the-x-axis#answer_418952
data = {};
for filename_idx=1:total_algorithms
    load(strcat(sourcepath, filesep, filenames{filename_idx},'.mat'));
    
    if(filename_idx==2)
        
    % renaming variables
    init_poses       = description.init_poses;
    total_poses      = length(init_poses);
    init_poses_sel   = 3;
    noises           = description.noises;
    total_noises     = length(noises);
    noises_sel       = [1, 2, 3];
    total_noises_sel = length(noises_sel);
    
    for dof_idx=1:total_dof
        data{filename_idx, dof_idx} = reshape( absolute_errors(:, dof_idx, noises_sel, init_poses_sel ), [], total_noises_sel);
    end
    
    else
    
    % renaming variables
    init_poses       = description.init_poses;
    total_poses      = length(init_poses);
    init_poses_sel   = 4;
    noises           = description.noises;
    total_noises     = length(noises);
    noises_sel       = [1, 3, 5];
    total_noises_sel = length(noises_sel);
    
    for dof_idx=1:total_dof
        data{filename_idx, dof_idx} = reshape( abs( errors(:, dof_idx, noises_sel, init_poses_sel )), [], total_noises_sel);
    end
        
    end
end
data = data';

%% Visualization

% set this to true if you want to see all of the transformation, set false
% if you want to see only the tz and Rz
all_transformation = true;
save_picture       = false;
% limit error to visualized
ymax = 10;
yticks = (1:1:ymax);

if (all_transformation)

    % we use subaxis function to control more for the spacing for the subplot
    % https://www.mathworks.com/matlabcentral/fileexchange/3696-subaxis-subplot
    fig1   = figure('Name', 'Error distribution', 'Position', [0 0 1200 600]);
    titles = {'Error distribution t_x (mm)', 'Error distribution t_y (mm)', 'Error distribution t_z (mm)', ...
              'Error distribution R_x (deg)', 'Error distribution R_y (deg)', 'Error distribution R_z (deg)'};
    for dof_idx=1:total_dof

        % prepare the subaxis
        subaxis( 2,3, dof_idx, ...
                 'SpacingVertical',0.13, 'SpacingHorizontal', 0.05, ...
                 'MarginLeft', 0, 'MarginRight', 0, 'MarginTop', 0.05);
        hold on;

        % get the dof
        if(dof_idx <=3)
            data_temp = data(dof_idx, :);
        else
            data_temp = data(abs(dof_idx-(total_dof+1))+3, :);
        end

        % draw the box plot
        h = boxplotGroup( data_temp, ...
                          'PrimaryLabels', alg_names, ...
                          'SecondaryLabels', strcat('Noise', {' '}, arrayfun(@num2str, noises(noises_sel), 'UniformOutput', 0)), ...
                          'GroupLabelType', 'Vertical', ...
                          'interGroupSpace', 1, ...
                          'BoxStyle', 'filled', ...
                          'Symbol', '.', ...
                          'MedianStyle', 'target');
        xtickangle(60);

        % coloring the box plot
        start_boxelement = (length(h.axis.Children) - total_algorithms)+1;
        end_boxelement   = length(h.axis.Children);
        colorpallete_idx = length(colorpalette);
        for element = end_boxelement:-1:start_boxelement
            set(h.axis.Children(element).Children,'Color', colorpalette{colorpallete_idx});
            colorpallete_idx = colorpallete_idx-1;
        end

        % coloring the dots (median and outliers)
        median_obj = findobj(gcf, 'Tag', 'MedianOuter');
        set(median_obj, 'MarkerSize', 6);
        median_obj = findobj(gcf, 'Tag', 'MedianInner');
        set(median_obj, 'MarkerEdgeColor', 'r');
        outlier_obj = findobj(gcf, 'Tag', 'Outliers');
        set(outlier_obj, 'MarkerEdgeColor', [0.3412, 0.3961, 0.4549]); 
        grid on;

        % limit the y_axis
        ymax = 15;
        ylim([0, ymax]);
        set(gca,'YTick',(1:2:ymax));

        % set the title
        title(titles{dof_idx});
    end
    
    if (save_picture)
        % save the picture
        % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
        set(fig1,'Units','Inches');
        pos = get(fig1,'Position');
        set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
        print(fig1, strcat(resultpath, filesep, sprintf('allalg_allT_initpose%d_abserror', init_poses(init_poses_sel))), '-dpdf','-r0');
        saveas(fig1, strcat(resultpath, filesep, sprintf('allalg_allT_initpose%d_abserror', init_poses(init_poses_sel))), 'png');
    end

else
    
    titles     = {'t_z absolute error (mm)', 'R_z absolute error  (deg)'};
    outputname = { sprintf('allukf_tz_initpose%d_abserror', init_poses(init_poses_sel)) , ...
                   sprintf('allukf_rz_initpose%d_abserror', init_poses(init_poses_sel)) };
        
    start_dofidx = 3;
    end_dofidx   = 4;
    for dof_idx=start_dofidx:end_dofidx
    
        % we use subaxis function to control more for the spacing for the subplot
        % https://www.mathworks.com/matlabcentral/fileexchange/3696-subaxis-subplot
        fig1 = figure('Name', 'Error distribution', 'Position', [0 0 500 300]);
        subaxis( 1,1, 1, ...
                 'SpacingVertical',0.15, 'SpacingHorizontal', 0.05, ...
                 'MarginLeft', 0.05, 'MarginRight', 0.01, 'MarginTop', 0.025);
        % get the data
        data_temp = data(dof_idx, :);
        % draw the box plot
        h = boxplotGroup( data_temp, ...
                          'PrimaryLabels', alg_names, ...
                          'SecondaryLabels', strcat('Noise', {' '}, arrayfun(@num2str, noises(noises_sel), 'UniformOutput', 0)), ...
                          'GroupLabelType', 'Vertical', ...
                          'interGroupSpace', 1, ...
                          'BoxStyle', 'filled', ...
                          'Symbol', '.', ...
                          'MedianStyle', 'target');
        xtickangle(60);

        % coloring the box plot
        start_boxelement = (length(h.axis.Children) - total_algorithms)+1;
        end_boxelement   = length(h.axis.Children);
        colorpallete_idx = length(colorpalette);
        % for element = start_boxelement : end_boxelement
        for element = end_boxelement:-1:start_boxelement
            % color_pallete_idx = (element - start_boxelement) + 1;
            set(h.axis.Children(element).Children,'Color', colorpalette{colorpallete_idx});
            colorpallete_idx = colorpallete_idx-1;
        end

        % coloring the dots (median and outliers)
        median_obj = findobj(gcf, 'Tag', 'MedianOuter');
        set(median_obj, 'MarkerSize', 6);
        median_obj = findobj(gcf, 'Tag', 'MedianInner');
        set(median_obj, 'MarkerEdgeColor', 'r');
        outlier_obj = findobj(gcf, 'Tag', 'Outliers');
        set(outlier_obj, 'MarkerEdgeColor', [0.3412, 0.3961, 0.4549]); 
        grid on;

        % limit the y_axis
        ylim([0, ymax]);
        set(gca,'YTick',yticks);

        % set the title
        % prepare the subaxis
        title_idx = (dof_idx-start_dofidx)+1;
        ylabel(titles{title_idx});
        
        if (save_picture)
            % save the picture
            % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
            set(fig1,'Units','Inches');
            pos = get(fig1,'Position');
            set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
            print(fig1, strcat(resultpath, filesep, outputname{title_idx}), '-dpdf','-r0');
            saveas(fig1, strcat(resultpath, filesep, outputname{title_idx}), 'png');
        end
    end
    
end
