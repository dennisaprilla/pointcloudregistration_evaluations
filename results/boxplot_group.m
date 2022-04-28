clear; close all;
addpath(genpath('..\functions\display'));

filename = 'backup\amode_normal\trials2\ukfnormal_30_trials2';
load(strcat(filename,'.mat'));

% renaming variables
init_poses   = description.init_poses;
total_poses  = length(init_poses);
noises       = description.noises;
total_noises = length(noises);

% rearrange data, the requirement for boxplotGroup, please refer
% https://www.mathworks.com/matlabcentral/answers/331381-how-do-you-create-a-grouped-boxplot-with-categorical-variables-on-the-x-axis#answer_418952
data = {};
for init_pose=1:total_poses
    for dof=1:6
        data{dof,init_pose} = reshape( abs(errors(:, dof, 1:total_noises, init_pose)), [], total_noises);
    end
end

%{
% use this block if you want to display trans and rots in the same figure
% we use subaxis function to control more for the spacing for the subplot
% https://www.mathworks.com/matlabcentral/fileexchange/3696-subaxis-subplot
figure('Name', 'Translation Error', 'Position', [0 0 1200 900])
total_poses = length(description.init_poses);
boxplot_handle = {};
for axis=1:(total_poses*2)
    
    init_pose = mod( axis-1, total_poses)+1;    
    subaxis(2,3, axis, 'SpacingVertical',0.15, 'SpacingHorizontal',0); hold on;
    
    if(axis<=3)
        data_temp = data(1:3,init_pose)';
        label = {'tx', 'ty', 'tz'};
    else
        data_temp = data(4:6,init_pose)';
        label = {'rz', 'ry', 'rx'};
    end
    boxplot_handle{axis} = boxplotGroup( data_temp, 'PrimaryLabels', label, 'SecondaryLabels', {'0', '1', '2', '3'});   
    grid on;
    
    if(init_pose==1)
        if(axis<=3)
            ylabel('Translation MAD (degree)');
        else
            ylabel('Rotation MAD (degree)');
        end
    else
        set(gca,'yticklabel', [])
    end
    ylim([0, 10]);
    title(sprintf('Initial Pose: %d', description.init_poses(init_pose)));
end
% saveas(fig1, sprintf('pictures/%s_trans', filename), 'png');
%}

%
% use this block if you want to display trans and rots in the diff figure
fig1 = figure('Name', 'Translation Error', 'Position', [0 0 1200 400]);
for axis=1:total_poses
   
    subaxis(1,total_poses, axis, 'SpacingVertical',0, 'SpacingHorizontal', 0, 'MarginLeft', 0.09, 'MarginRight', 0); hold on;
    
    init_pose = axis;
    data_temp = data(1:3,init_pose)';
    label = {'tx', 'ty', 'tz'};
    h = boxplotGroup( data_temp, 'PrimaryLabels', label, 'SecondaryLabels', {'0', '0.5', '1', '1.5', '2', '2.5'}, ...
                  'BoxStyle', 'filled', 'Symbol', '.', 'MedianStyle', 'target'); 
    set(h.axis.Children(1).Children,'Color', '#ff7675');
    set(h.axis.Children(2).Children,'Color', '#55efc4');
    set(h.axis.Children(3).Children,'Color', '#74b9ff');
    median_obj = findobj(gcf, 'Tag', 'MedianOuter');
    set(median_obj, 'MarkerSize', 5);
    median_obj = findobj(gcf, 'Tag', 'MedianInner');
    set(median_obj, 'MarkerEdgeColor', 'r');
    outlier_obj = findobj(gcf, 'Tag', 'Outliers');
    set(outlier_obj, 'MarkerEdgeColor', [0.3412, 0.3961, 0.4549]);
    
    grid on;
    
    if(init_pose==1)
        ylabel('Translation Absolute Difference (meter)');
    else
        set(gca,'yticklabel', [])
    end
    ylim([0, 10]);
    title(sprintf('Initial Pose: %d', init_poses(init_pose)));
end
% % save the picture
% % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
% set(fig1,'Units','Inches');
% pos = get(fig1,'Position');
% set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
% print(fig1, sprintf('pictures/%s_abserror_trans', filename),'-dpdf','-r0');
% saveas(fig1, sprintf('pictures/%s_abserror_trans', filename), 'png');

% window 2
fig2 = figure('Name', 'Rotation Error', 'Position', [0 0 1200 400]);
for axis=1:total_poses
   
    subaxis(1,total_poses, axis, 'SpacingVertical',0, 'SpacingHorizontal', 0, 'MarginLeft', 0.09, 'MarginRight', 0); hold on;
    
    init_pose = axis;
    data_temp = data(6:-1:4,init_pose)';
    label = {'rx', 'ty', 'tz'};
    h = boxplotGroup( data_temp, 'PrimaryLabels', label, 'SecondaryLabels', {'0', '0.5', '1', '1.5', '2', '2.5'}, ...
                  'BoxStyle', 'filled', 'Symbol', '.', 'MedianStyle', 'target'); 
    set(h.axis.Children(1).Children,'Color', '#ff7675');
    set(h.axis.Children(2).Children,'Color', '#55efc4');
    set(h.axis.Children(3).Children,'Color', '#74b9ff');
    median_obj = findobj(gcf, 'Tag', 'MedianOuter');
    set(median_obj, 'MarkerSize', 5);
    median_obj = findobj(gcf, 'Tag', 'MedianInner');
    set(median_obj, 'MarkerEdgeColor', 'r');
    outlier_obj = findobj(gcf, 'Tag', 'Outliers');
    set(outlier_obj, 'MarkerEdgeColor', [0.3412, 0.3961, 0.4549]);
    grid on;
    
    if(init_pose==1)
        ylabel('Rotation Absolute Difference (degree)');
    else
        set(gca,'yticklabel', [])
    end
    ylim([0, 10]);
    title(sprintf('Initial Pose: %d', init_poses(init_pose)));
end
% % save the picture
% set(fig2,'Units','Inches');
% pos = get(fig2,'Position');
% set(fig2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
% print(fig2, sprintf('pictures/%s_abserror_rots', filename),'-dpdf','-r0');
% saveas(fig2, sprintf('pictures/%s_abserror_rots', filename), 'png');
%