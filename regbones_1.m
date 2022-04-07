%% Bone Registration Simulation
% This script is assumed that the coarse registration is already been done,
% so that, inital position of simulation is assumed to be, at least, under
% 8 degrees in rotation and 8 mm in translation.

% Several important variable here, so you guys can track what var is what:
% U_breve                     The complete bone model.
% U_breve_part                A portion of U_breve around B-mode US 
%                             measurement.
% U_breve_finereg             U_breve, after fine registration
%                             transformation. Note: This variable will only
%                             be used for visualization, since it is
%                             difficult to see the true shape of the bone
%                             if we only plot the US measurement.
% U_breve_ultrafinereg_       U_breve, after ultrafine-registration
% fineregCS                   transformation, in fine-registration CS. The
%                             optimization perform better in local CS.
%                             Note: This variable will only be used for 
%                             visualization purpose
% U_breve_final               U_breve, after fine-registration and
%                             ultrafine-registration transformation.
%                             Note: This variable will only be used for 
%                             visualization purpose
%
% Ua_pointcloud               A-mode US measurement simulation point cloud
% Ua_noised                   Ua_pointcloud, with noise.
% Ua_finereg                  Ua_noised, after fine registration
%                             transformation.
% Ua_ultrafinereg_fineregCS   Ua_noised, after ultrafine registration
%                             transformation, in fine registration CS. The
%                             optimization will perform better in local CS.
% Ua_final                    Ua_noised, after fine registration and
%                             ultrafine registration transformation.
%
% Y_breve                     The transformed U_breve, simulation motion.
% Y_breve_part                A portion of Y_breve around B-mode US. 
%                             Similar to U_breve_part, but Y_breve_part is
%                             a motion transformed version.
% Y_breve

clc; clear; close all;

% current path
path_pointcloudregistration = pwd;
% path to costfunction evaluation project
% please refer ro to : https://github.com/dennisaprilla/costfunction_evaluation
path_costfunctionevaluation = 'D:\DennisChristie\costfunction_evaluation';
% path to a GMMReg project by Bing Jian and Baba C. Vemuri
% please refer to : https://github.com/bing-jian/gmmreg
path_gmmreg       = 'D:\DennisChristie\gmmreg\MATLAB\GaussTransform';
% path to a CMA-ES project
path_cmaes        = 'D:\DennisChristie\pointcloudregistration_evaluations\functions\optimizers\CMAES';

% path to the bone model
path_bone         = strcat(path_pointcloudregistration, filesep, 'data', filesep, 'bone');
% path to the a-mode US measurement simulation
path_amode        = strcat(path_bone, filesep, filesep, 'amode_accessible_sim2');
% path to shifting constant
path_shiftconst   = strcat(path_costfunctionevaluation, filesep, 'results', filesep, 'amode_simulations', filesep, 'accessible_sim2');
% path to cost function for optimization
path_costfunction = 'functions\costfunction';
path_experimental = 'functions\experimental';
% path to store the outputs
path_output       = 'results';

% add all the paths
addpath(path_bone);
addpath(path_amode);
addpath(path_shiftconst);
addpath(path_gmmreg);
addpath(path_experimental);
addpath(path_costfunction);
addpath(path_cmaes);

% debug mode, if displaybone is true, it will (as the name suggests)
% display the process of registration. the simulation will also only run once
displaybone = true;

% clear unneccesary variables
clear path_pointcloudregistration path_boneUSsimple path_gmmreg;

%% Prepare Bone Point Cloud

% read the point cloud (bone) from STL/PLY file
ptCloud          = stlread('data/bone/CT_Tibia_R.stl');
% specify the scale. here the, the bone is measured in meter, it is easier
% for us to work in mm.
ptCloud_scale    = 1000;
ptCloud_Npoints  = size(ptCloud.Points,1);
ptCloud_centroid = mean(ptCloud.Points, 1);
% prepare Ŭ, the noiseless, complete, moving dataset
U_breve          = (ptCloud.Points - ptCloud_centroid);

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
           'Tag', 'plot_U_breve');
    grid on; axis equal; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

%% Prepare the A-mode data

% read the point cloud (A-mode) from the mat file
amode_config       = 15;
filename_amodedata = sprintf('amode_tibia_%d', 15);
filepath_amodedata = sprintf('%s%s%s.mat', path_amode, filesep, filename_amodedata);
load(filepath_amodedata);
Ua_pointcloud = vertcat(amode_all.Position);

% (for debugging only) show figure for sanity check
if(displaybone)
    plot3( axes1, ...
           Ua_pointcloud(:,1), ...
           Ua_pointcloud(:,2), ...
           Ua_pointcloud(:,3), ...
           'or', 'MarkerFaceColor', 'r', ...
           'Tag', 'plot_Ua_pointcloud');
    title('Initial Setup');

    drawnow;
    pause(1);
end

% read the shifting constant, acquired by "calibration"
% please refer to :https://github.com/dennisaprilla/costfunction_evaluation
% in directory 'results'
amode_gmmscale = 40;
filename_shiftingconstant = sprintf('tibia_gmm_scale%d_shiftingconstant', amode_gmmscale);
filepath_shiftingconstant = sprintf('%s%s%s.mat', path_shiftconst, filesep, filename_shiftingconstant);
load(filepath_shiftingconstant);

%% Simulation Setup (Cost function related)

% set up ultrafine registration (GMM) costfunction configuration
ultrafinereg_name       = "cpd-gmm_ab";
ultrafinereg_scaleconst = 1e-4;
ultrafinereg_scale_a    = amode_gmmscale;
ultrafinereg_shiftconst = shiftingconstant(amode_config);
ultrafinereg_useshiftconst = false;

% set up matlab optimizer parameters;
optimizer = 'cmaes';
x0        = [0 0 0 0 0 0];
A         = [];
b         = [];
Aeq       = [];
beq       = [];
lb        = [ deg2rad([-3 -3 -10]), ([-3, -3, -10]/ptCloud_scale) ];
ub        = [ deg2rad([ 3  3  10]), ([ 3,  3,  10]/ptCloud_scale) ];
nonlcon   = [];

if (strcmp(optimizer, 'fmincon'))
    options = optimoptions('fmincon', ...
                           'Display', 'off', ...
                           'SpecifyObjectiveGradient', false, ...
                           'FunctionTolerance', 1e-8, ...
                           'StepTolerance', 1e-6, ...
                           'MaxFunctionEvaluations', 1000);
elseif (strcmp(optimizer, 'ga'))
    options = optimoptions( 'ga', ...
                            'Display', 'iter', ...
                            'FunctionTolerance', 1e-8, ...
                            'PopulationSize', 100, ...
                            'MaxGenerations', 200*6, ...
                            'MaxTime', 30, ...
                            'UseParallel', true);
elseif (strcmp(optimizer, 'cmaes'))
    options.UBounds   = ub';
    options.LBounds   = lb';
    options.ParforRun = 'on';
    options.ParforWorkers = 64;
    options.TolX      = 1e-6;
    options.TolFun    = 1e-8;
    options.MaxIter   = 1000;
    options.DispFinal = 'off';
    options.DispModulo = Inf;
end
                   
%% Simulation Setup (Trials related)
                       
% setup the noise constants
noise_levels            = [1 2 3];
init_poses              = [3 5];
n_trials                = 100;

% naming the filename for result
filepath_results = 'results';

% create a metadata for the simulation
% metadata for algorithm
sim_config.ultrafinereg.algorithm   = ultrafinereg_name;
sim_config.ultrafinereg.scaleconst  = ultrafinereg_scaleconst;
sim_config.ultrafinereg.scale_a     = ultrafinereg_scale_a;
% metadata for noise
sim_config.noise_levels    = noise_levels;
sim_config.init_poses      = init_poses;
sim_config.trials          = n_trials;
sim_config.result_dimdesc  = ["trials", "observation dimensions", "noises", "initial poses"];

%% Simulation Start

% variable which stores the results
errors            = zeros(n_trials, 6, length(noise_levels), length(init_poses));
rmse_measurements = zeros(n_trials, 1, length(noise_levels), length(init_poses));
rmse_trues        = zeros(n_trials, 1, length(noise_levels), length(init_poses));

% start init_pose loop
for init_pose=1:length(init_poses)
    
max_t     = init_poses(init_pose) / ptCloud_scale;
max_theta = init_poses(init_pose);

% start noise_level loop
for noise_level=1:length(noise_levels)

% setup the noise
noise     = noise_levels(noise_level);

% start trial loop
for trial=1:n_trials 
    fprintf('init pose: %d, noise: %d, trial: %d\n', init_pose, noise_level, trial);
    
    %% apply random noise

    % add isotropic zero-mean gaussian noise to U, simulating noise measurement
    random_noise = -noise/ptCloud_scale + (noise/ptCloud_scale + noise/ptCloud_scale) * rand(size(Ua_pointcloud, 1), 3);
    Ua_noised = Ua_pointcloud + random_noise;

    % show figure for sanity check
    if (displaybone)
        delete(findobj('Tag', 'plot_Ua_pointcloud'));
        delete(findobj('Tag', 'plot_Ub_pointcloud'));
        plot3( axes1, ...
               Ua_noised(:,1), ...
               Ua_noised(:,2), ...
               Ua_noised(:,3), ...
               'or', ...
               'Tag', 'plot_Ua_noised');
        title('Noise Measurment Added');
        
        drawnow;
        pause(1);        
    end
    
    %% apply random transformation
    
    % https://www.mathworks.com/matlabcentral/answers/383378-how-do-i-generate-a-random-number-between-two-numbers#answer_305777
    random_trans = -max_t     + (max_t -(-max_t))         .* rand(1, 3);
    random_theta = -max_theta + (max_theta -(-max_theta)) .* rand(1, 3);
    random_R     = eul2rotm(deg2rad(random_theta), 'ZYX');
    DoF6_GT      = [random_trans, random_theta];
    Y_breve      = (random_R * U_breve' + random_trans')';
    
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
        pause(1);
    end
    
    %% Fine-Registration
    
    fprintf('Fine registration...');
    my_tic = tic;
    [T_finereg, ~] = fineregistration(Ua_noised, Y_breve, 'cpd', []);
    exec_time = toc(my_tic);
    fprintf('(%.2fs) ', exec_time);
    
    % extract the R and t from T
    R_finereg       = T_finereg(1:3, 1:3);
    t_finereg       = T_finereg(1:3, 4)';
    % transform Ua_noised and Ub_noised with finereg transformation
    Ua_finereg      = (R_finereg * Ua_noised' + t_finereg')';
    % transform the U_breve just for visualization (not for calculation)
    U_breve_finereg = (R_finereg * U_breve'   + t_finereg')';
    
    % show figure for sanity check
    if (displaybone)
        delete(findobj('Tag', 'plot_U_breve'));
        delete(findobj('Tag', 'plot_U_breve_part'));
        delete(findobj('Tag', 'plot_Ua_noised'));
        delete(findobj('Tag', 'plot_Ub_noised'));
        plot3( axes1, ...
               Ua_finereg(:,1), ...
               Ua_finereg(:,2), ...
               Ua_finereg(:,3), ...
               'or', ...
               'Tag', 'plot_Ua_finereg');
        plot3( axes1, ...
               U_breve_finereg(:,1), ...
               U_breve_finereg(:,2), ...
               U_breve_finereg(:,3), ...
               '.', 'Color', [0.9290 0.6940 0.1250], ...
               'MarkerSize', 0.1, ...
               'Tag', 'U_breve_finereg');
        title('Fine-registration Applied');        

        drawnow;
        pause(1);
    end
    
    %% Ultrafine-registration
    % Ultrafine-registration is based on optimization. It perform better
    % when the opimization occur in local coordinate frame. 
    %
    % Let's illustrate this:
    % Bone -> (T_fine) -> Bone_intermedieate -> (T_ultrafine) -> Bone_GT.
    % after T_fine, Bone_intermedieate is not in the origin anymore. There 
    % will be some DoFs that is more sensitive to a certain transformation. 
    % To make them equally sensitive, we need to consider Bone_fine as the
    % origin, the local coordinate system
    %
    % We can do that by, transforming Bone_intermedieate and Bone_GT, with 
    % the inv(T_fine), so that now, the coordinate system is local to
    % Bone_intermedieate.
    %
    % (*1) However, in this simulation, since inv(T_fine) * Bone_fine is 
    % our original point cloud, i will used the original point cloud, i.e.
    % Bone. (*2) And, i will only do the inv(T_fine) * Bone_GTmotion.
    
    % change coordinate system to Fine Registration Coordinate System, as
    % described in (*2)
    Y_breve_fineregCS      = (R_finereg' * Y_breve' - t_finereg')';
    
    % show figure for sanity check
    if (displaybone)
        % figure two is used for visualizing the registration in FineReg
        % coordinate system. so it is easier for the user to monitor the
        % transformation in this coordinate system.
        figure2 = figure('Name', 'Registration in FineReg Coordinate System', 'Position', [350 0 350 780]);
        axes2 = axes('Parent', figure2);
        plot3( axes2, ...
               Y_breve_fineregCS(:,1), ...
               Y_breve_fineregCS(:,2), ...
               Y_breve_fineregCS(:,3), ...
               '.g', ...
               'MarkerSize', 0.1, ...
               'Tag', 'Y_breve_fineregCS');
        grid on; axis equal; hold on;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        plot3( axes2, ...
               U_breve(:,1), ...
               U_breve(:,2), ...
               U_breve(:,3), ...
               '.', 'Color', [0.9290 0.6940 0.1250], ...
               'MarkerSize', 0.1, ...
               'Tag', 'plot_U_breve_fineregCS');
        plot3( axes2, ...
               Ua_noised(:,1), ...
               Ua_noised(:,2), ...
               Ua_noised(:,3), ...
               'or', ...
               'Tag', 'plot_Ua_noised_fineregCS');
        title('Fine-registration Applied');

        drawnow;
        pause(1);        
    end
   
    fprintf('Ultrafine registration...');
    
    % setup parameter to the optimizer
    config.model_a   = Ua_noised;
    config.scene_a   = Y_breve_fineregCS;
    config.scale_a   = ultrafinereg_scale_a * ultrafinereg_scaleconst;
    
    my_tic = tic;
    % starting the optimization-based registration in Fine Registration Coordinate System
    if (strcmp(optimizer, 'fmincon'))
        param        = fmincon(@gmmL2_R_a, x0, A, b, Aeq, beq, lb, ub, nonlcon, options, config);
        
    elseif (strcmp(optimizer, 'ga'))
        costfunction = @(X) gmmL2_R_a(X, config);
        param        = ga(costfunction, length(x0), A, b, Aeq, beq, lb, ub, nonlcon, options);
        
    elseif (strcmp(optimizer, 'cmaes'))
        cmaes_sigma_r = deg2rad([1.5 1.5 5]);
        cmaes_sigma_t = [1.5 1.5 5] / ptCloud_scale;
        cmaes_sigma   = [cmaes_sigma_r, cmaes_sigma_t]';
        param = cmaes_parfor('gmmL2_R_a', x0, cmaes_sigma, options, config);
        param = param';
    end
    exec_time = toc(my_tic);
    fprintf('(%.2fs) \n', exec_time);
    
    % extract the R and t from param (rotation from costfunction is already in radians)
    % Note: this R and t is in Fine Registration Coordinate System
    if(ultrafinereg_useshiftconst)
        % if we want to use the shifting constant, let's substract the Rz
        % and tz with shifting constant from "calibration";
        r_temp     = [param(1), param(2), param(3)-deg2rad(ultrafinereg_shiftconst(1))];
        t_temp     = [param(4), param(5), param(6)-ultrafinereg_shiftconst(2)];
    else
        r_temp     = [param(1), param(2), param(3)];
        t_temp     = [param(4), param(5), param(6)];
    end
    % now rename our variable
    R_ultrafinereg  = eul2rotm(r_temp, 'ZYX');
    t_ultrafinereg  = t_temp;
    T_ultrafinereg  = [R_ultrafinereg, t_ultrafinereg'; 0 0 0 1];
    % as described in (*1), i will only transform the original position of
    % the point cloud
    Ua_ultrafinereg_fineregCS = (R_ultrafinereg * Ua_noised' + t_ultrafinereg')';
    % transform the U_breve just for visualization (not for calculation)
    U_breve_ultrafinereg_fineregCS = (R_ultrafinereg * U_breve' + t_ultrafinereg')';
    
    % show figure for sanity check
    if (displaybone)
        delete(findobj('Tag', 'plot_U_breve_fineregCS'));
        delete(findobj('Tag', 'plot_Ua_noised_fineregCS'));
        delete(findobj('Tag', 'plot_Ub_noised_fineregCS'));
        plot3( axes2, ...
               U_breve_ultrafinereg_fineregCS(:,1), ...
               U_breve_ultrafinereg_fineregCS(:,2), ...
               U_breve_ultrafinereg_fineregCS(:,3), ...
               '.', 'Color', [0.9290 0.6940 0.1250], ...
               'MarkerSize', 0.1, ...
               'Tag', 'plot_U_breve_fineregCS');
        plot3( axes2, ...
               Ua_ultrafinereg_fineregCS(:,1), ...
               Ua_ultrafinereg_fineregCS(:,2), ...
               Ua_ultrafinereg_fineregCS(:,3), ...
               'or', ...
               'Tag', 'plot_Ua_noised_fineregCS');
        title('Ultrafine-registration Applied');

        drawnow;
        pause(1);
    end
    
    % Nevertheless, we are working in our original coordinate system, so we
    % need to construct the overall transformation.
    T_all    = T_ultrafinereg * T_finereg;
    R_all    = T_all(1:3, 1:3);
    eul_all  = rad2deg(rotm2eul(R_all, 'ZYX'));
    t_all    = T_all(1:3, 4)';
    % store the 6dof value for performance calculation
    DoF6_est = [t_all, eul_all];
    % transform
    Ua_final = (R_all * Ua_noised' + t_all')';
    % transform the U_breve just for visualization (not for calculation)
    U_breve_final = (R_all * U_breve' + t_all')';
    
    % show figure for sanity check
    if (displaybone)
        % We also need to show the overall transformation in our original
        % figure, Figure 1, i.e. the original coordinate system.
        delete(findobj('Tag', 'U_breve_finereg'));
        delete(findobj('Tag', 'plot_Ua_finereg'));
        delete(findobj('Tag', 'plot_Ub_finereg'));
        plot3( axes1, ...
               Ua_final(:,1), ...
               Ua_final(:,2), ...
               Ua_final(:,3), ...
               'or', ...
               'Tag', 'plot_Ua_finereg');
        plot3( axes1, ...
               U_breve_final(:,1), ...
               U_breve_final(:,2), ...
               U_breve_final(:,3), ...
               '.', 'Color', [0.9290 0.6940 0.1250], ...
               'MarkerSize', 0.1, ...
               'Tag', 'U_breve_finereg');
        title('Ultrafine-registration Applied');
        
    end
    
    %% calculate performance
    
    % calculate error
    error = diff([DoF6_GT; DoF6_est]);
    % calculate rmse_measurment
    [~, nearest_dist] = knnsearch(Y_breve, Ua_final);
    rmse_measurement  = mean(nearest_dist);
    % calculate rmse_true
    rmse_true        = mean(sqrt(sum((U_breve_final - Y_breve).^2, 2)));
    
    % display performance
    disp(DoF6_GT);
    disp(DoF6_est);
    disp(error);
    
    errors(trial, :, noise_level, init_pose)             = error;
    rmse_measurements(trial, :, noise_level, init_pose)  = rmse_measurement;
    rmse_trues(trial, :, noise_level, init_pose)         = rmse_true;
    
    % if debug mode go out of the loop
    if( displaybone )
        break;
    end
    
% end trials     
end

% save the trials
save( strcat(filepath_results, filesep, 'atrials_1.mat'), ...
      'errors', 'rmse_measurements', 'rmse_trues', 'sim_config');

% if debug mode go out of the loop
if( displaybone )
    break;
end

% end noise
end

% if debug mode go out of the loop
if( displaybone )
    break;
end

% end init pose
end

% delete data stored by cmaes
delete *.dat
delete variablescmaes.mat










