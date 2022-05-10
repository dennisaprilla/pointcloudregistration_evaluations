close all; clear; clc;

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Femur_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = ptCloud.Points - ptCloud_centroid;

% display the femure bone
figure1 = figure('Name', 'Bone', 'Position', [0 -100 400 900]);
axes1 = axes('Parent', figure1);
plot3( axes1, ...
       U_breve(:,1), ...
       U_breve(:,2), ...
       U_breve(:,3), ...
       '.', 'Color', [0.7 0.7 0.7], ...
       'MarkerSize', 0.1, ...
       'Tag', 'plot_bone_full');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; hold on;

% % load('amode_measure3.mat');
% % U = [ vertcat(amode_prereg.Position); vertcat(amode_mid.Position) ];
% filename = 'amode_tibia_30';
% load(strcat(filename, '.mat'));
% U = [ vertcat(amode_prereg1.Position); ...
%       vertcat(amode_prereg2.Position); ...
%       vertcat(amode_prereg3.Position); ...
%       vertcat(amode_mid.Position) ];
% plot3( axes1, ...
%        U(:,1), ...
%        U(:,2), ...
%        U(:,3), ...
%        'or', 'MarkerFaceColor', 'r', ...
%        'Tag', 'plot_bone_full');

filename = 'amode_femur_15';
path = 'amode_accessible_sim2';
load(strcat(path, filesep, filename, '.mat'));
U = vertcat(amode_all.Position);
plot3( axes1, ...
       U(:,1), ...
       U(:,2), ...
       U(:,3), ...
       'or', 'MarkerFaceColor', 'r', ...
       'Tag', 'plot_bone_full');
%    
% % save picture to pdf
% % https://www.mathworks.com/matlabcentral/answers/12987-how-to-save-a-matlab-graphic-in-a-right-size-pdf
% set(figure1,'Units','Inches');
% pos = get(figure1,'Position');
% set(figure1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)]);
% print(figure1, sprintf('%s', 'amode_tibiawd1_30'),'-dpdf','-r0');

% filename = 'usdata_b_0b';
% path = 'D:\Documents\BELANDA\PhD Thesis\Code\MATLAB\boneUSsimple\outputs\usmeasurement_b';
% load(strcat(path, filesep, filename, '.mat'));
% Ub_pointcloud = bmode_simulation.pointcloud;
% plot3( axes1, ...
%        Ub_pointcloud(:,1), ...
%        Ub_pointcloud(:,2), ...
%        Ub_pointcloud(:,3), ...
%        'or', 'MarkerFaceColor', 'r', ...
%        'Tag', 'plot_bone_full');



















