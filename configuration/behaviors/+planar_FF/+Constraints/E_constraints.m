function E_constraints(nlp, src, tar, bounds, varargin)
% Apply NL constraint on am defined edge

%% joint-position & joint-velocity stitching
plant = nlp.Plant;
plant.rigidImpactConstraint(nlp, src, tar, bounds, varargin{:});

%% Customize periodic motions for the last domain

    % NOT time-continuous for the last edge
    removeConstraint(nlp,'tContDomain');
    
    % The relabeling of joint coordiante is no longer valid
    removeConstraint(nlp,'xDiscreteMapLeftImpactRelabel');
    
    R = plant.R;
    
    % The configuration only depends on the relabeling matrix
    x = plant.States.x;
    xn = plant.States.xn;
    x_diff = R*x-xn;
        
    x_map = SymFunction(['xDiscreteMap' plant.Name], x_diff(2:end), {x,xn});
    addNodeConstraint(nlp, x_map, {'x','xn'}, 'first', 0, 0, 'Linear');
        
end