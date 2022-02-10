clear; close all;
addpath(genpath('..\functions\display'));

filename = 'cpdtrials_bone';
load(strcat(filename,'.mat'));

% rearrange data, the requirement for boxplotGroup, please refer
% https://www.mathworks.com/matlabcentral/answers/331381-how-do-you-create-a-grouped-boxplot-with-categorical-variables-on-the-x-axis#answer_418952
data = {};
for init_pose=1:length(description.init_poses)
    for dof=1:6
        data{dof,init_pose} = reshape( absolute_errors(:, dof, 1:4, init_pose), [], 4);
    end
end

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

%{
% use this block if you want to display trans and rots in the diff figure
total_poses = length(description.init_poses);
fig1 = figure('Name', 'Translation Error', 'Position', [0 0 1200 450]);
for axis=1:total_poses
   
    subaxis(1,3, axis, 'SpacingVertical',0, 'SpacingHorizontal', 0, 'MarginLeft', 0.09, 'MarginRight', 0); hold on;
    
    init_pose = axis;
    data_temp = data(1:3,init_pose)';
    label = {'tx', 'ty', 'tz'};
    boxplotGroup( data_temp, 'PrimaryLabels', label, 'SecondaryLabels', {'0', '1', '2', '3'});   
    grid on;
    
    if(init_pose==1)
        ylabel('Translation Absolute Difference (degree)');
    else
        set(gca,'yticklabel', [])
    end
    ylim([0, 10]);
    title(sprintf('Initial Pose: %d', description.init_poses(init_pose)));
end
% save the picture
% https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
set(fig1,'Units','Inches');
pos = get(fig1,'Position');
set(fig1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
print(fig1, sprintf('pictures/%s_trans', filename),'-dpdf','-r0');
fig2 = figure('Name', 'Rotation Error', 'Position', [0 0 1200 450]);

% window 2
for axis=1:total_poses
   
    subaxis(1,3, axis, 'SpacingVertical',0, 'SpacingHorizontal', 0, 'MarginLeft', 0.09, 'MarginRight', 0); hold on;
    
    init_pose = axis;
    data_temp = data(6:-1:4,init_pose)';
    label = {'rx', 'ty', 'tz'};
    boxplotGroup( data_temp, 'PrimaryLabels', label, 'SecondaryLabels', {'0', '1', '2', '3'});   
    grid on;
    
    if(init_pose==1)
        ylabel('Rotation Absolute Difference (degree)');
    else
        set(gca,'yticklabel', [])
    end
    ylim([0, 10]);
    title(sprintf('Initial Pose: %d', description.init_poses(init_pose)));
end
% save the picture
set(fig2,'Units','Inches');
pos = get(fig2,'Position');
set(fig2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
print(fig2, sprintf('pictures/%s_rots', filename),'-dpdf','-r0');
%}