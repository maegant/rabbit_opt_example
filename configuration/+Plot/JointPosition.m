function JointPosition(logger)
figure
tiledlayout('flow');

nj = size(logger(1).flow.states.x,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        plot(logger(seg).flow.t, logger(seg).flow.states.x(i,:))
        hold on
        title(sprintf('State %i Position',i));
    end
end