function FootHeight(bounds, nlp)
% constraints for swing foot clearance

domain = nlp.Plant;
x = domain.States.x;
dx = domain.States.dx;

min_height = 0.1;

% ONLY WORKS FOR FLAT FOOT RIGHT NOW BECAUSE OF NSF NAME

nsf_height = SymFunction(['nsf_height_', domain.Name], nlp.Plant.EventFuncs.nsf.ConstrExpr, {x});
addNodeConstraint(nlp, nsf_height, {'x'}, 'all',                  0,    0.17, 'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, floor(nlp.NumNode/4),   min_height/2, 0.17, 'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, ceil(nlp.NumNode/2),    min_height, 0.17,'Nonlinear');
addNodeConstraint(nlp, nsf_height, {'x'}, floor(3*nlp.NumNode/4), min_height/2, 0.17, 'Nonlinear');

end

