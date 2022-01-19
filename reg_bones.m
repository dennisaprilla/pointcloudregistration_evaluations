clc; clear all; close all;

% path_to_function = 'D:\Documents\BELANDA\PhD Thesis\Code\MATLAB\kalman_filter\kalmanfilter_registration_fixed\functions\ukf';
path_to_function = "D:\Documents\MATLAB\GoICP_V1.3";
% addpath(path_to_function);

%%

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Tibia_R.stl');
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = (ptCloud.Points - ptCloud_centroid)' * ptCloud_scale;

%%

% contruct a arbritary transformation then apply it to Ŭ in order to
% generate Y̆, the noiseless, complete, fixed dataset.
init_pose    = 5;
max_t        = init_pose;
max_theta    = init_pose;
random_trans = -max_t     + (max_t -(-max_t))         .* rand(1, 3);
random_theta = -max_theta + (max_theta -(-max_theta)) .* rand(1, 3);
random_R     = eul2rotm(deg2rad(random_theta), 'ZYX');
GT           = [random_trans, random_theta];
Y_breve      = random_R * U_breve + random_trans';

% Read the simulated a-mode measurement point cloud, which is a subset of Ŭ.
% These a-mode simulated measurement is manually selected from the bone model.
selectedpoint_str = sprintf('data/bone/amode_measure.mat');
load(selectedpoint_str);
U = [vertcat(amode_prereg.Position); vertcat(amode_mid.Position)]';
% add isotropic zero-mean gaussian noise to U, simulating noise measurement
var_yacute   = 0;
N_point      = size(U, 2);
Sigma_yacute = var_yacute * eye(3);
n_yacute     = mvnrnd( [0 0 0], Sigma_yacute, N_point)';
U = U + n_yacute;

%%

% plot Ŭ, the noiseless, complete, moving dataset
figure1 = figure(1);
figure1.WindowState  = 'maximized';
axes1 = axes('Parent', figure1);
plot3( axes1, ...
       U_breve(1,:), ...
       U_breve(2,:), ...
       U_breve(3,:), ...
       '.r', 'MarkerSize', 0.1, ...
       'Tag', 'plot_Ubreve');
xlabel('X'); ylabel('Y');
grid(axes1, 'on'); axis(axes1, 'equal'); hold(axes1, 'on');
% plot U, the noisy, incomplete, moving dataset
plot3( axes1, ...
       U(1,:), ...
       U(2,:), ...
       U(3,:), ...
       'or', ...
       'Tag', 'plot_U');
%plot Y̆, the noiseless, complete, fixed dataset.
plot3( axes1, ...
       Y_breve(1,:), ...
       Y_breve(2,:), ...
       Y_breve(3,:), ...
       '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 0.1, ...
       'Tag', 'plot_Ybreve');

%%

% CPD Registration
% change the point structure to be suit to matlab icp built in function
moving = pointCloud(U');
fixed  = pcdownsample(pointCloud(Y_breve'), 'gridAverage', 5);
% register with icp
[tform, movingReg, icp_rmse] = pcregistercpd( moving, ...
                                              fixed, ...
                                              'Transform', 'Rigid', ...
                                              'OutlierRatio', 0.15, ...
                                              'MaxIteration', 100, ...
                                              'verbose', false);

T_all   = tform.T';

%% calculate performance

t_all      = T_all(1:3, 4);
R_all      = T_all(1:3, 1:3);
eul_all    = rad2deg(rotm2eul(R_all, 'ZYX'));
Uest       = R_all * U + t_all;
Uest_breve = R_all * U_breve + t_all;

disp([GT; [t_all', eul_all] ]);

clf;
plot3(Uest(1,:), Uest(2,:), Uest(3,:), 'ob');
grid on; axis equal; hold on;
plot3(Y_breve(1,:), Y_breve(2,:), Y_breve(3,:), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 0.1);  
plot3(Uest_breve(1,:), Uest_breve(2,:), Uest_breve(3,:), '.r', 'MarkerSize', 0.1);  






                       