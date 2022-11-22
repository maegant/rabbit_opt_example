% Description: This is the main simulation function for running a FROST
%   based simulation. Simply provide a set of parameters and a behavior.
%
% Author: Jenna Reher, jreher@caltech.edu
%         Wenlong Ma, wma@caltech.edu
% ________________________________________

function logger = sim_frost(behavior, params, nSteps)

if nargin < 3
    nSteps = 2;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Extract the parameters 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1 : height(behavior.hybridSystem.Gamma.Nodes)
    params{i}.kvelocity = 25; %Kp for reldegree 1 outputs
    params{i}.kposition = [400, 60];  % Kp Kd gains on rel degree 2 outputs
%     params{i}.epsilon = 2e-1; %Epsilon for Other Controller Types
    
    behavior.hybridSystem = setVertexProperties(behavior.hybridSystem, ...
                            behavior.hybridSystem.Gamma.Nodes.Name{i}, ...
                            'Param', params{i},...
                            'Control',IOFeedback('IO'));
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Run the simulator 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = params{1}.x0;
behavior.hybridSystem.setOption('OdeSolver', @ode15s);

t0 = 0;
tf = 10;

tic
    logger = behavior.hybridSystem.simulate(t0, x0, tf, [],'NumCycle', nSteps);
toc


end