close all; 

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Tibia_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = ptCloud.Points - ptCloud_centroid;

% display the femure bone
% figure1 = figure('Name', 'Bone', 'Position', [0 -100 400 900]);
figure1 = figure(1);
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

% load('amode_measure3.mat');
% U = [ vertcat(amode_prereg.Position); vertcat(amode_mid.Position) ];
filename = 'amodewd_tibia2_30';
load(strcat(filename, '.mat'));
U = vertcat(amode_all.Position);
plot3( axes1, ...
       U(:,1), ...
       U(:,2), ...
       U(:,3), ...
       'or', 'MarkerFaceColor', 'r', ...
       'Tag', 'plot_bone_full');