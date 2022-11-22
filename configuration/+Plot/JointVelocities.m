function JointVelocities(logger)
figure

nj = size(logger(1).flow.states.x,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        plot(logger(seg).flow.t, logger(seg).flow.states.dx(i,:))
        hold on
        title(sprintf('State %i Velocity',i));
    end
end
end