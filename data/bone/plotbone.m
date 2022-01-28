clear; close all;

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Femur_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = ptCloud.Points - ptCloud_centroid;

% display the femure bone
figure1 = figure(1);
figure1.WindowState  = 'maximized';
axes1 = axes('Parent', figure1);
plot3( axes1, ...
       U_breve(:,1), ...
       U_breve(:,2), ...
       U_breve(:,3), ...
       '.', 'Color', [0.7 0.7 0.7], ...
       'Tag', 'plot_bone_full');
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; hold on;