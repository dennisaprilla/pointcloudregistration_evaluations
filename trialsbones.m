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


%% Simulation trials

noises            = [0 1 2 3];
init_poses        = [3 5 8];
n_trials          = 100;
selectedpoint_str = sprintf('data/bone/amode_measure.mat');

description.algorithm  = 'ukf';
description.noises     = noises;
description.init_poses = init_poses;
description.trials     = n_trials;
description.dim_desc   = ["trials", "observation dimensions", "noises", "initial poses"];

absolute_errors   = zeros(n_trials, 6, length(noises), length(init_poses));
rmse_measurements = zeros(n_trials, 1, length(noises), length(init_poses));
rmse_trues        = zeros(n_trials, 1, length(noises), length(init_poses));

for init_pose=1:length(init_poses)
    
max_t     = init_poses(init_pose);
max_theta = init_poses(init_pose);

for noise=1:length(noises)
    
var_yacute = noises(noise);

for trial=1:n_trials
    fprintf('init pose: %d, noise: %d, trial: %d\n', init_pose, noise, trial);
    
    %% radom transformation, point selection, and noise
    
    % contruct a arbritary transformation then apply it to Ŭ in order to
    % generate Y̆, the noiseless, complete, fixed dataset.
    random_trans = -max_t     + (max_t -(-max_t))         .* rand(1, 3);
    random_theta = -max_theta + (max_theta -(-max_theta)) .* rand(1, 3);
    random_R     = eul2rotm(deg2rad(random_theta), 'ZYX');
    GT           = [random_trans, random_theta];
    Y_breve      = random_R * U_breve + random_trans';

    % Read the simulated a-mode measurement point cloud, which is a subset of Ŭ.
    % These a-mode simulated measurement is manually selected from the bone model.
    load(selectedpoint_str);
    U = [vertcat(amode_prereg.Position); vertcat(amode_mid.Position)]';
    % add isotropic zero-mean gaussian noise to U, simulating noise measurement
    N_point      = size(U, 2);
    Sigma_yacute = var_yacute * eye(3);
    n_yacute     = mvnrnd( [0 0 0], Sigma_yacute, N_point)';
    U = U + n_yacute;
    

    %% registration
    
    if (strcmp(description.algorithm, 'icp'))

        % ICP Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U');
        fixed  = pointCloud(Y_breve');
        % register with icp
        [tform, movingReg, icp_rmse] = pcregistericp( moving, ...
                                                      fixed, ...
                                                      'InlierRatio', 1, ...
                                                      'Verbose', false, ...
                                                      'MaxIteration', 50 );
        % change the T form
        T_all   = tform.T';
        % store the rmse
        rmse_measurement = icp_rmse;

    elseif (strcmp(description.algorithm, 'cpd'))
    
        % CPD Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U');
        fixed  = pcdownsample( pointCloud(Y_breve'), 'gridAverage', 2);
        % register with icp
        [tform, movingReg, cpd_rmse] = pcregistercpd( moving, ...
                                                      fixed, ...
                                                      'Transform', 'Rigid', ...
                                                      'OutlierRatio', 0.15, ...
                                                      'MaxIteration', 150, ...
                                                      'Tolerance', 1e-6, ...
                                                      'verbose', false);
    	% change the T form
    	T_all   = tform.T';
        % store the rmse
        rmse_measurement = cpd_rmse;

    elseif (strcmp(description.algorithm, 'ukf'))

        % UKF Registration
        [T_all, mean_dist] = ukf_isotropic_registration( U, Y_breve, U_breve, ...
                               'threshold', 0.5, ...
                               'iteration', 150, ...
                               'expectednoise', 1.25*var_yacute, ...
                               'sigmaxanneal', 0.98, ...
                               'sigmaxtrans', 1.2*max_t, ...
                               'sigmaxtheta', 1.2*max_theta, ...
                               'verbose', false, ...
                               'display', false);
        % store the rmse
        rmse_measurement = mean_dist;

    elseif (strcmp(description.algorithm, 'goicp'))
            
        % GO-ICP Registration
        % normalize everything
        temp = [U, Y_breve]';
        scale = max(max(temp));
        temp = temp ./ scale;
        data = temp(1:size(U, 2), :);
        model = temp(size(U, 2)+1:end, :);
        % store data.txt
        fileID = fopen('data\temp\data.txt','w');
        fprintf(fileID,'%d\n', size(data, 1));
        fprintf(fileID,'%f %f %f\n', data');
        fclose(fileID);
        % store model.txt
        fileID = fopen('data\temp\model.txt','w');
        fprintf(fileID,'%d\n',  size(model, 1));
        fprintf(fileID,'%f %f %f\n', model');
        fclose(fileID);    
        % run GO-ICP
        goicp_exe  = "GoICP_vc2012";
        cmd = sprintf("%s %s %s %d %s %s", ...
                      strcat(path_to_function, filesep, "bin", filesep, goicp_exe), ...
                      "data\temp\model.txt", ...
                      "data\temp\data.txt", ...
                      N_point, ...
                      strcat(path_to_function, filesep, "demo", filesep, "config.txt"), ...
                      "data\temp\output.txt");
        system(cmd);
        % open output file
        file = fopen('data\temp\output.txt', 'r');
        time = fscanf(file, '%f', 1);
        R    = fscanf(file, '%f', [3,3])';
        t    = fscanf(file, '%f', [3,1]) * scale;
        fclose(file);
        % reformat the T
        T_all = [R, t; 0 0 0 1];
        % no rmse reported, so give it NaN
        rmse_measurement = NaN;
    end
    
    
    %% calculate performance

    t_all      = T_all(1:3, 4);
    R_all      = T_all(1:3, 1:3);
    eul_all    = rad2deg(rotm2eul(R_all, 'ZYX'));
    Uest       = R_all * U + t_all;
    Uest_breve = R_all * U_breve + t_all;
    
    absolute_errors(trial, :, noise, init_pose)    = abs( diff( [GT; [t_all', eul_all] ], 1));
    rmse_measurements(trial, :, noise, init_pose)  = rmse_measurement;
    rmse_trues(trial, :, noise, init_pose)         = mean(sqrt(sum((Uest_breve - Y_breve).^2, 2)));
    
    %% visualization
    
%     clf;
%     plot3(Uest(1,:), Uest(2,:), Uest(3,:), 'ob');
%     grid on; axis equal; hold on;
%     plot3(Y_breve(1,:), Y_breve(2,:), Y_breve(3,:), '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 0.1);  
%     plot3(Uest_breve(1,:), Uest_breve(2,:), Uest_breve(3,:), '.r', 'MarkerSize', 0.1);   

% end trials
end

% end init_poses

end

save(sprintf('results/%strials_%s.mat', description.algorithm, 'bone'), 'absolute_errors', 'rmse_measurements', 'rmse_trues', 'description');

% end noise
end


