function TorsoPitch(bounds, nlp)

% constraints for step length
lb = -0.05;
ub = 0.15;

domain = nlp.Plant;
x = domain.States.x;

torsoFun = SymFunction(['torsoAngle_',domain.Name], x(3), {x});
addNodeConstraint(nlp, torsoFun, {'x'}, 'all', lb, ub,'Nonlinear');


end

