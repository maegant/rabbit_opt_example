%% Script: main_opt
%
% Description: This script runs the main optimization routine for a loaded
%   behavior. You can specify whether to recompile the behavior
%   expressions, model, and cost functions separately through a true/false
%   flag. You can also load initial guesses from simulation through
%   commenting out the corresponding lines related to the simulation
%   "logger". Users can specify whether to use IPOPT or SNOPT by simply
%   commenting out the corresponding lines. SNOPT requires an additional
%   license. Finally, the "do_save" option at the bottom of the script can
%   be set to "true" if you would like to save the walking gait parameters
%   as a YAML file.
%
% Author: Maegan Tucker, mtucker@caltech.edu
%
% ________________________________________
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Setup Frost / Paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;

%%%% CUSTOMIZE FROST PATH BASED ON YOUR COMPUTER:
frost_path = '../../frost-dev/';

% initialize frost
addpath(frost_path);
frost_addpath;
    
% add necessary folders to your search path:
addpath(genpath('tools'));
addpath('urdf'); %add urdf folder
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Run Startup Script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% choose mex files to compile
compileMex = [1,1]; %export model and/or behavior

% load behavior
behavior = startup_walker(compileMex); 
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Load the behavior specific NLP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Setup optimization
do_compile = [1,1,1]; %model,behavior,cost

% load nlp for behavior
nlp = feval(strcat(behavior.name, '.Constraints.setupOpt'), behavior);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Compile and export optimization functions
%%%% (uncomment the following  lines when run it for the first time.)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
do_export_optModel    = do_compile(1);
do_export_optBehavior = do_compile(2);
do_export_optCost     = do_compile(3);

t1 = tic;
customExportOptimization(behavior, nlp, do_export_optModel, do_export_optBehavior, do_export_optCost);
fprintf('Compilation took %f minutes.\n', toc(t1)/60);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Link the NLP problem to a NLP solver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Log all activities in logger
if exist('log.yaml','file')
    delete('log.yaml');
end
diary log.txt;


% Run IPOPT
options.max_iter = 300;
solver = IpoptApplication(nlp,options);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Run the optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t1 = tic;
[sol, info] = optimize(solver);
fprintf('Elapsed time is %f minutes.\n', toc(t1)/60);
diary off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Check and export the optimization result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

do_save = true;
[logger, edges, params, save_path] = export_optimization( nlp, sol, behavior, do_save);%, label_append );

if do_save
    data = struct();
    data.logger = logger;
    data.edges = edges;
    data.params = params;
    data.sol = sol;
    data.nlp = nlp;
    data.save_path = save_path;
    
    %%% log optimization results
    save(fullfile(save_path, 'optData.mat'), 'data');
    movefile('log.txt', save_path);
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Animate Solution (with slopes)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Close existing simulations
close all;
app2Handle = findall(findall(0,'Type', 'figure'), 'Name', 'UI Figure');
close(app2Handle);

% Run Simulations 
num_steps = 10;
logger_sim = sim_frost(behavior, params, num_steps);
conGUI = Plot.LoadAnimator(behavior, logger_sim);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Make Plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Plot.JointPosition(logger_sim)
Plot.JointVelocities(logger_sim)
Plot.JointTorque(logger_sim)
Plot.OutputTracking(logger_sim)
