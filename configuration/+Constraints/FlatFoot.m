function FlatFoot(bounds, nlp, whichStance)
% constraints for swing foot clearance


    % constraints on swing foot roll and pitch
    domain = nlp.Plant;
    x = domain.States.x;
    
    if strcmp(whichStance,'Right')
        theta_nsf = domain.getEulerAngles(domain.ContactPoints.LeftSole);
    else
        theta_nsf = domain.getEulerAngles(domain.ContactPoints.RightSole);
    end
    
    pitch_constr_fun = SymFunction(['nsf_pitch_',domain.Name], theta_nsf(2), x);
    
    % For RABBIT:
    pitch_bound = deg2rad(2);
    addNodeConstraint(nlp, pitch_constr_fun, {'x'}, 'all', 0-pitch_bound, 0+pitch_bound, 'Nonlinear');
    addNodeConstraint(nlp, pitch_constr_fun, {'x'}, 'terminal', 0-eps, 0+eps, 'Nonlinear');

end

