function [ nlp ] = setupOpt( behavior )
% This function defines cost function and NLP constraints.
import([behavior.name, '.Constraints.*']);
    
%% Customize model boundaries (tunable)
model_bounds = behavior.robotModel.getLimits();
model_bounds.states.ddx.lb = -100 * ones(behavior.robotModel.numState,1);
model_bounds.states.ddx.ub =  100 * ones(behavior.robotModel.numState,1);

name = 'BasePosX';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) = -1;
model_bounds.states.x.ub(idx) = 1;
model_bounds.states.dx.lb(idx) = 0.3;
model_bounds.states.dx.ub(idx) = 1;

name = 'BasePosZ';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) = 0.7;
model_bounds.states.x.ub(idx) = 2;
model_bounds.states.dx.lb(idx) = -0.5;
model_bounds.states.dx.ub(idx) = 0.5;

name = 'BaseRotY';
idx = behavior.robotModel.getJointIndices(name);
model_bounds.states.x.lb(idx) = -0.1745;
model_bounds.states.x.ub(idx) = 0.5;
model_bounds.states.dx.lb(idx) = -0.5;
model_bounds.states.dx.ub(idx) = 0.5;

idx = behavior.robotModel.getJointIndices({'RightHip','LeftHip'});
model_bounds.states.x.lb(idx) = -1.1416;
model_bounds.states.x.ub(idx) = 1.8584;
model_bounds.states.dx.lb(idx) = -2;
model_bounds.states.dx.ub(idx) =  2;

idx = behavior.robotModel.getJointIndices({'RightKnee','LeftKnee'});
model_bounds.states.x.lb(idx) = 0.5;
model_bounds.states.x.ub(idx) = 2;
model_bounds.states.dx.lb(idx) = -2;
model_bounds.states.dx.ub(idx) =  2;


idx = behavior.robotModel.getJointIndices({'RightAnkle','LeftAnkle'});
model_bounds.states.x.lb(idx) = -0.5;
model_bounds.states.x.ub(idx) = 1.57;
model_bounds.states.dx.lb(idx) = -2;
model_bounds.states.dx.ub(idx) =  2;

model_bounds.states.x.x0 = zeros(9,1);

model_bounds.step_duration = [0.5, 1.2]; %essential: how long should a step last
model_bounds.step_length   = [0.35, 0.6]; %essential: how long should a step last

%% Domain-specified bounds (Better Not Tune)
bounds = struct();
vNames = fields(behavior.vertices);
eNames = fields(behavior.edges);
bounds.(vNames{1}) = model_bounds;
bounds.(eNames{1}) = model_bounds;

%%% Time
bounds.(vNames{1}).time.t0.lb = 0;
bounds.(vNames{1}).time.t0.ub = 0;
bounds.(vNames{1}).time.tf.lb = bounds.(vNames{1}).step_duration(1); 
bounds.(vNames{1}).time.tf.ub = bounds.(vNames{1}).step_duration(2);

%%% Constraint Wrench Forces 
bounds.(vNames{1}).inputs.ConstraintWrench.fRightSole.lb = [-100, -10, -50]; %x z pitch
bounds.(vNames{1}).inputs.ConstraintWrench.fRightSole.ub = [ 100, 500 , 50]; 
bounds.(vNames{1}).inputs.ConstraintWrench.fRightSole.x0 = [ 0, 300, 20]; 
bounds.(vNames{1}).params.pRightSole.lb = zeros(3,1);
bounds.(vNames{1}).params.pRightSole.ub = zeros(3,1);
bounds.(vNames{1}).params.pRightSole.x0 = zeros(3,1);

%%% actuator bounds for SS domain %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% actuator_indices = 1:4;
bounds.(vNames{1}).inputs.Control.u.lb = -100 * ones(6,1);
bounds.(vNames{1}).inputs.Control.u.ub = 100 * ones(6,1);
bounds.(vNames{1}).inputs.Control.u.x0 = zeros(6,1);

%%% RD2 outputs
bounds.(vNames{1}).position.kp = 400;
bounds.(vNames{1}).position.kd = 60;
bounds.(vNames{1}).velocity.kp = 25;

% trajectories coefficient
bounds.(vNames{1}).params.aposition.lb = -10; 
bounds.(vNames{1}).params.aposition.ub =  10;
bounds.(vNames{1}).params.aposition.x0 =  0;
bounds.(vNames{1}).params.avelocity.lb = 0.4; 
bounds.(vNames{1}).params.avelocity.ub =  1.2;
bounds.(vNames{1}).params.avelocity.x0 =  0.5;

% phase variable: linearized hip position
switch behavior.phaseType 
    case 'TimeBased'
        bounds.(vNames{1}).params.pposition.lb = [bounds.(vNames{1}).time.tf.lb, bounds.(vNames{1}).time.t0.lb]; 
        bounds.(vNames{1}).params.pposition.ub = [bounds.(vNames{1}).time.tf.ub, bounds.(vNames{1}).time.t0.ub];
    case 'StateBased'
        bounds.(vNames{1}).params.pposition.lb = [0.06, -0.4]; %[theta-,theta+]
        bounds.(vNames{1}).params.pposition.ub = [0.4,  -0.06];
        bounds.(vNames{1}).params.pvelocity.lb = [0.06, -0.4]; %[theta-,theta+]
        bounds.(vNames{1}).params.pvelocity.ub = [0.4,  -0.06];
end

%% Setup the NLP
behavior.vertices.(vNames{1}).UserNlpConstraint = str2func('V_constraints');
behavior.edges.(eNames{1}).UserNlpConstraint = str2func('E_constraints');

num_grid.(vNames{1}) = 7;
nlp = HybridTrajectoryOptimization(behavior.name, behavior.hybridSystem, num_grid, ...
                                   [], 'EqualityConstraintBoundary', 1e-4);
nlp.configure(bounds);


%% Add a cost function
weight= 1e1;
CostType = {'mCOT'};   
nlp = Opt.applyCost(behavior, nlp, CostType, weight);

nlp.update;

end