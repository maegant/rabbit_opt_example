function JointTorque(logger)
figure
t = tiledlayout('flow');

nj = size(logger(1).flow.inputs.Control.u,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        plot(logger(seg).flow.t, logger(seg).flow.inputs.Control.u(i,:))
        hold on
        title(sprintf('Joint %i',i));
    end
end

title(t,'Joint Torques')
end