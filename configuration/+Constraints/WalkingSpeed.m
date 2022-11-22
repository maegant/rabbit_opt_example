function WalkingSpeed(bounds, nlp)
%% Average walking speed (essential )

v_target = [0.4, 1.2];
% v_target = [bounds.averageVel-(delta_percent*0.05),...
%             bounds.averageVel+(delta_percent*0.05)];

domain = nlp.Plant;

T0  = SymVariable('t0',  [2, 1]);
TF  = SymVariable('tf',  [2, 1]);
X0  = SymVariable('x0', [domain.numState,1]);
XF  = SymVariable('xF', [domain.numState,1]);
avg_vel = (XF(1) - X0(1)) / (TF(2) - T0(1)); 
avg_vel_fun = SymFunction('average_velocity', avg_vel, {T0, TF, X0, XF});
avg_vel_cstr = NlpFunction('Name','average_velocity',...
    'Dimension',1, ...
    'lb', v_target(1), ...
    'ub', v_target(2), ...
    'Type','Nonlinear', ...
    'SymFun', avg_vel_fun,...
    'DepVariables', [nlp.OptVarTable.T(1); nlp.OptVarTable.T(end); ...
                     nlp.OptVarTable.x(1); nlp.OptVarTable.x(end)]);
addConstraint(nlp, 'average_velocity', 'last', avg_vel_cstr);


end

