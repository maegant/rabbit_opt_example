%% Function: loadBehavior
%
% Description: 
%
% Author: Maegan Tucker, mtucker@caltech.edu
%         Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ________________________________________

function behavior = loadBehavior(behaviorName, is_symmetric, phaseType, ...
                       delay_coriolis, omit_coriolis, do_export_behavior, do_export_model)

t1 = tic;

% hide GCC version errors
warning('off', 'MATLAB:mex:GccVersion_link')

% Remove all other on the export path
p = strread(path,'%s','delimiter',':');
for i = 1:numel(p)
    if ~isempty(strfind(p{i}, 'export'))
        rmpath(p{i});
    end
end

% Ensure the config is on path
config_dir = 'configuration';
addpath(genpath(config_dir));

% Load the behavior
behavior_path = fullfile(config_dir, 'behaviors');
addpath(genpath(behavior_path));

behavior = feval(strcat(behaviorName, '.behavior'));
behavior.init(is_symmetric, phaseType, delay_coriolis, omit_coriolis);

% Export if requested
%%% Dynamics
robot = behavior.robotModel;
baseType = '2D';


% setup dynamics export path:
model_export_path = fullfile('export', 'model', robot.Name, baseType, 'dynamics');
if ~isfolder(model_export_path)
    mkdir(model_export_path);
end
addpath(genpath(model_export_path));

% export dynamics
if do_export_model
    for i = 1:numel(robot.Mmat)
        robot.Mmat{i}.export(model_export_path);
    end
    for i = 1:numel(robot.Fvec)
        robot.Fvec{i}.export(model_export_path);
    end
    robot.exportKinematics(model_export_path);
end

%%% Behavior
behavior_export_path = fullfile('export', 'behavior', behaviorName, phaseType, 'sim');
if ~isfolder(behavior_export_path)
    mkdir(behavior_export_path);
end
addpath(genpath(behavior_export_path));

%%% this can handle odd number of edges
if do_export_behavior
    v = fields(behavior.vertices);
    for i = 1:numel(v)
        customCompileVertex(behavior.vertices.(v{i}), behavior_export_path);
    end
    
    if ~isempty(behavior.edges)
        e = fields(behavior.edges);
        for i =  1:numel(e)
            customCompileEdge(behavior.edges.(e{i}), behavior_export_path);
        end
    end    
end

fprintf('Loading Behavior took %f minutes.\n', toc(t1)/60);
end