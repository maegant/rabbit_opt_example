function V_constraints(nlp, bounds, varargin)
% NL constraint on a vertex

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;
u  = domain.Inputs.Control.u;
N = nlp.NumNode;

%% RD 2 output (Better Not Tune)
domain.VirtualConstraints.position.imposeNLPConstraint(nlp, ...
                     [bounds.position.kp, bounds.position.kd], [1, 1]);
domain.VirtualConstraints.velocity.imposeNLPConstraint(nlp, ...
                     bounds.velocity.kp, 1);

%%% RD2 tau boundary [0,1]
tau = domain.VirtualConstraints.position.PhaseFuncs{1};
tauv = domain.VirtualConstraints.velocity.PhaseFuncs{1};

switch domain.VirtualConstraints.position.PhaseType
    case 'TimeBased'
        T_name = nlp.OptVarTable.T(1).Name;
        T  = SymVariable(lower(T_name),[nlp.OptVarTable.T(1).Dimension,1]);
        p_name = nlp.OptVarTable.pposition(1).Name;
        p  = SymVariable(lower(p_name),[nlp.OptVarTable.pposition(1).Dimension,1]);
        tau_0 = SymFunction(['tau_0_',domain.Name], T(1) - p(2), {T,p});
        tau_F = SymFunction(['tau_F_',domain.Name], T(2) - p(1), {T,p});
        addNodeConstraint(nlp, tau_0, {T_name,p_name}, 'first', 0, 0, 'Linear');
        addNodeConstraint(nlp, tau_F, {T_name,p_name}, 'last', 0, 0, 'Linear');
    
    case 'StateBased'
        addNodeConstraint(nlp, tau, {'x','pposition'}, 'first', 0, 0, 'Nonlinear');
        addNodeConstraint(nlp, tau, {'x','pposition'}, 'last',  1, 1, 'Nonlinear');
        addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'first', 0, 0, 'Nonlinear');
        addNodeConstraint(nlp, tauv, {'x','pvelocity'}, 'last',  1, 1, 'Nonlinear');
end

%% Added Constraints
stanceFoot = 'Right';
Constraints.WalkingSpeed(bounds, nlp);
Constraints.SwingFoot(bounds, nlp, stanceFoot)
Constraints.FlatFoot(bounds, nlp, stanceFoot)
end