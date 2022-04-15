clc; clear; close all;

% path to data
path_bone   = 'data\bone';
path_amode  = 'data\bone\amode_normals';
path_result = 'results';
% path to project
path_icpnormal = 'functions\experimental';
path_ukf   = 'D:\Documents\BELANDA\PhD Thesis\Code\MATLAB\kalman_filter\kalmanfilter_registration_fixed\functions\ukf';
path_goicp = "D:\Documents\MATLAB\GoICP_V1.3";

addpath(path_icpnormal);
addpath(path_ukf);
addpath(path_goicp);

displaybone = false;

%% Prepare the bone point cloud

% read the point cloud (bone) from STL/PLY file
filename_bonedata = 'CT_Tibia_R';
filepath_bonedata = strcat(path_bone, filesep, filename_bonedata, '.stl');
ptCloud           = stlread(filepath_bonedata);
% scale the point cloud in in mm unit
ptCloud_scale     = 1000;
ptCloud_Npoints   = size(ptCloud.Points,1);
ptCloud_centroid  = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve           = (ptCloud.Points - ptCloud_centroid) * ptCloud_scale;
U_breve_hat       = STLVertexNormals(ptCloud.ConnectivityList, ptCloud.Points);

% show figure for sanity check
if(displaybone)
    figure1 = figure('Name', 'Registration in Measurement Coordinate System', 'Position', [0 0 350 780]);
    axes1 = axes('Parent', figure1);
    plot3( axes1, ...
           U_breve(:,1), ...
           U_breve(:,2), ...
           U_breve(:,3), ...
           '.', 'Color', [0.7 0.7 0.7], ...
           'MarkerSize', 0.1, ...
           'Tag', 'plot_Ubreve');
    grid on; axis equal; hold on;
%     quiver3(U_breve(:,1), U_breve(:,2), U_breve(:,3), ...
%             U_breve_normals(:,1), U_breve_normals(:,2), U_breve_normals(:,3));
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

%% Prepare the A-mode measurement simulation

% read the point cloud (A-mode) from the mat file
filename_amodedata = 'amode_tibia_15_withnormals_3';
filepath_amodedata = strcat(path_amode, filesep, filename_amodedata, '.mat');
load(filepath_amodedata);

% get the amode
U     = vertcat(amode_all.Position) * ptCloud_scale;
U_hat = U_breve_hat(vertcat(amode_all.DataIndex), :);

% (for debugging only) show figure for sanity check
if(displaybone)
    plot3( axes1, ...
           U(:,1), ...
           U(:,2), ...
           U(:,3), ...
           'or', 'MarkerFaceColor', 'r', ...
           'Tag', 'plot_U');
    quiver3(U(:,1),     U(:,2),     U(:,3), ...
            U_hat(:,1), U_hat(:,2), U_hat(:,3), 0.1, ...
            'Tag', 'plot_Uhat');
    title('Initial Setup');

    drawnow;
    pause(0.5);
end

%% Simulation Config

noises            = [0 1 2 3];
noisenormal_const = 3;
init_poses        = [3 5 8 10];
n_trials          = 1;

description.algorithm  = 'ukfnormal';
description.noises     = noises;
description.init_poses = init_poses;
description.trials     = n_trials;
description.dim_desc   = ["trials", "observation dimensions", "noises", "initial poses"];

filename_result   = sprintf('%s_trials%d.mat', description.algorithm, 1);

GTs               = zeros(n_trials, 6, length(noises), length(init_poses));
estimations       = zeros(n_trials, 6, length(noises), length(init_poses));
errors            = zeros(n_trials, 6, length(noises), length(init_poses));
rmse_measurements = zeros(n_trials, 1, length(noises), length(init_poses));
rmse_trues        = zeros(n_trials, 1, length(noises), length(init_poses));


%% Simulation Start

for init_pose=1:length(init_poses)
    
max_t     = init_poses(init_pose);
max_theta = init_poses(init_pose);

for noise=1:length(noises)
    
noise_point = noises(noise);
noise_normal = noise_point * noisenormal_const;

for trial=1:n_trials
    fprintf('init pose: %d, noise: %d, trial: %d\n', init_pose, noise, trial);
    
    %% apply some random noise

    % % add isotropic zero-mean gaussian noise to U, simulating noise measurement
    % % uncomment this block if you want to use standard deviation for noise
    % N_point      = size(U, 1);
    % Sigma_yacute = var_yacute * eye(3);
    % n_yacute     = mvnrnd( [0 0 0], Sigma_yacute, N_point);
    % U_noised     = U + n_yacute;
    random_point   = -noise_point + 2*noise_point * rand(size(U, 1), 3);
    U_noised       = U + random_point;
    
    % if the algorithm specified by user is using normal, we provide the
    % normal calculations
    if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
        U_hat_noised = [];
        for i=1:size(U_hat,1)
            random_normal = -noise_normal + 2*noise_normal * rand(1, 3);
            random_R      = eul2rotm(deg2rad(random_normal), 'ZYX');
            U_hat_noised  = [U_hat_noised; (random_R * U_hat(i,:)')'];
        end
    end
    
    % show figure for sanity check
    if (displaybone)
        delete(findobj('Tag', 'plot_U'));
        delete(findobj('Tag', 'plot_Uhat'));
        plot3( axes1, ...
               U_noised(:,1), ...
               U_noised(:,2), ...
               U_noised(:,3), ...
               'or', ...
               'Tag', 'plot_U_noised');    
        % if the algorithm specified by user is using normal, we provide the
        % normal calculations
        if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
            quiver3(axes1, ...
                    U_noised(:,1),     U_noised(:,2),     U_noised(:,3), ...
                    U_hat_noised(:,1), U_hat_noised(:,2), U_hat_noised(:,3), 0.1, ...
                    'Tag', 'plot_Uhat_noised');
        end
        title('Noise Measurment Added');
        
        drawnow;
        pause(0.5);       
    end
    
    
    %% radom transformation, point selection, and noise
    
    % contruct a arbritary transformation then apply it to Ŭ in order to
    % generate Y̆, the noiseless, complete, fixed dataset.
    random_trans = -max_t     + (max_t -(-max_t))         .* rand(1, 3);
    random_theta = -max_theta + (max_theta -(-max_theta)) .* rand(1, 3);
    random_R     = eul2rotm(deg2rad(random_theta), 'ZYX');
    GT           = [random_trans, random_theta];
    Y_breve      = (random_R * U_breve' + random_trans')';
    
    % if the algorithm specified by user is using normal, we provide the
    % normal calculations
    if (strcmp(description.algorithm, 'ukfnormal') || strcmp(description.algorithm, 'icpnormal'))
        Y_breve_hat  = (random_R * U_breve_hat')';
    end
    
    % show figure for sanity check
    if(displaybone)
        plot3( axes1, ...
               Y_breve(:,1), ...
               Y_breve(:,2), ...
               Y_breve(:,3), ...
               '.g', 'MarkerSize', 0.1, ...
               'Tag', 'plot_Ybreve');
        title('Random Transformation Applied');

        drawnow;
        pause(0.5);
    end
    

    %% registration
    
    if (strcmp(description.algorithm, 'icp'))

        % ICP Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U_noised);
        fixed  = pointCloud(Y_breve);
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
        
    elseif (strcmp(description.algorithm, 'icpnormal'))
        
        % ICP normal registration
        moving       = U_noised;
        movingnormal = U_hat_noised * ptCloud_scale;
        fixed        = Y_breve;
        fixednormal  = Y_breve_hat * ptCloud_scale;
        [T_all, icpnormal_rmse] = icpnormal( moving, movingnormal, ...
                                             fixed, fixednormal, ...
                                             U_breve, ...
                                             'iteration', 100, ...
                                             'threshold', 1, ...
                                             'normalratio', 0.05, ...
                                             'ransacdistance', 5, ...
                                             'verbose', true, ...
                                             'display', true);

        % store the rmse
        rmse_measurement = icpnormal_rmse;        

    elseif (strcmp(description.algorithm, 'cpd'))
    
        % CPD Registration
        % change the point structure to be suit to matlab icp built in function
        moving = pointCloud(U_noised);
        fixed  = pcdownsample( pointCloud(Y_breve), 'gridAverage', 2);
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
        
    elseif (strcmp(description.algorithm, 'cpdnormal'))
        
        % to be implemented

    elseif (strcmp(description.algorithm, 'ukf'))

        % UKF Registration
        [T_all, mean_dist] = ukf_isotropic_registration( U_noised', Y_breve', U_breve', ...
                               'threshold', 0.5, ...
                               'iteration', 150, ...
                               'expectednoise', 1.25*noise_point, ...
                               'sigmaxanneal', 0.98, ...
                               'sigmaxtrans', 1.2*max_t, ...
                               'sigmaxtheta', 1.2*max_theta, ...
                               'verbose', false, ...
                               'display', false);
        % store the rmse
        rmse_measurement = mean_dist;
        
    elseif (strcmp(description.algorithm, 'ukfnormal'))
        
        % UKF Registration with normals
        movingnormal = U_hat_noised' * ptCloud_scale;
        fixednormal = Y_breve_hat' * ptCloud_scale;
        [T_all, mean_dist, history] = ukf_isotropic_registration_ex2( U_noised', Y_breve', U_breve', ...
                                           'movingnormal', movingnormal, ...
                                           'fixednormal', fixednormal, ...
                                           'normalratio', 0.05, ...
                                           'threshold', 0.0001, ...
                                           'iteration', 100, ...
                                           'expectednoise', 1.0*noise_normal, ...
                                           'sigmaxanneal', 0.98, ...
                                           'sigmaxtrans', 1.0*max_t, ...
                                           'sigmaxtheta', 1.0*max_theta, ...
                                           'bestrmse', true, ...
                                           'verbose', false, ...
                                           'display', false);
                           
        % store the rmse
        rmse_measurement = mean_dist;

    elseif (strcmp(description.algorithm, 'goicp'))
            
        % GO-ICP Registration
        % normalize everything
        temp = [U_noised, Y_breve]';
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
    Uest       = (R_all * U' + t_all)';
    Uest_breve = (R_all * U_breve' + t_all)';
   
    if(displaybone)
        delete(findobj('Tag', 'plot_Ubreve'));
        delete(findobj('Tag', 'plot_U_noised'));
        delete(findobj('Tag', 'plot_Uhat_noised'));
        plot3( axes1, ...
               Uest(:,1), ...
               Uest(:,2), ...
               Uest(:,3), 'or', ...
               'Tag', 'plot_Uest');
        plot3( axes1, ...
               Uest_breve(:,1), ...
               Uest_breve(:,2), ...
               Uest_breve(:,3), ...
               '.r', 'MarkerSize', 0.1, ...
               'Tag', 'plot_Uest_breve');
    end
    
    % store the results
    GTs(trial, :, noise, init_pose)               = GT;
    estimations(trial, :, noise, init_pose)       = [t_all', eul_all];
    errors(trial, :, noise, init_pose)            = diff( [GT; [t_all', eul_all] ], 1, 1);
    rmse_measurements(trial, :, noise, init_pose) = rmse_measurement;
    rmse_trues(trial, :, noise, init_pose)        = mean(sqrt(sum((Uest_breve - Y_breve).^2, 2)));

    % if debug mode go out of the loop
    if( displaybone )
        disp('Estimated');
        disp([t_all', eul_all]);
        disp('GT');
        disp(GT);
        disp('Error');
        disp(errors(trial, :, noise, init_pose));
        break;
    end

% end trials
end

% save the trials if not debug mode
if (~displaybone)
    save( strcat(path_result, filesep, filename_result), ...
          'GTs', 'estimations', 'errors', 'rmse_measurements', 'rmse_trues', 'description');
% if debug mode, go out of the loop
else
    break;
end

% end init_poses
end

% if debug mode go out of the loop
if( displaybone )
    break;
end

% end noise
end


