%% Function: customExportOptimization
%
% Description:
%   Compile expressions with customizible options
%
% Author: Jenna Reher, jreher@caltech.edu
% ______________________________________
function [ ] = customExportOptimization( behavior, nlp, do_export_optModel,...
                                         do_export_optBehavior,...
                                         do_export_optCost )
%% Model
robot = behavior.robotModel;
baseType = '2D';

model_export_path = fullfile('export', 'model', robot.Name, baseType, 'dynamics');
if ~isfolder(model_export_path)
    mkdir(model_export_path);
end
addpath(genpath(model_export_path));

if do_export_optModel
   compileCoriolis(nlp, model_export_path);
   compileConstraint(nlp, [], {'dynamics_equation'}, model_export_path, []);
   compileDynamicsWithoutCoriolis(nlp, model_export_path); % Compile dynamical functions partially
end


%% Behavior
behaviorName = behavior.name;

behavior_export_path = fullfile('export', 'behavior', behaviorName, behavior.phaseType, 'opt');
if ~isfolder(behavior_export_path)
    mkdir(behavior_export_path);
end
addpath(genpath(behavior_export_path));

if do_export_optBehavior
    compileConstraint(nlp,[],[], behavior_export_path,{'dynamics_equation'});
end

%% Cost Function
if do_export_optCost
    compileObjective(nlp, [],[], behavior_export_path);
end

end