function OutputTracking(logger)
figure
tiledlayout('flow');

nj = size(logger(1).flow.yd_position,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        p1 = plot(logger(seg).flow.t, logger(seg).flow.yd_position(i,:),'--');
        hold on
        p2 = plot(logger(seg).flow.t, logger(seg).flow.ya_position(i,:));
        title(sprintf('Output %i',i));
    end
end

legend([p1,p2],{'Desired','Actual'});

%% 
figure; 
t = tiledlayout('flow');

nj = size(logger(1).flow.yd_position,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        plot(logger(seg).flow.t, logger(seg).flow.ya_position(i,:)-logger(seg).flow.yd_position(i,:));
        hold on
    end
end
title(t,'Position Error');
%% 
figure; 
t = tiledlayout('flow');

nj = size(logger(1).flow.yd_position,1);
for i = 1:nj
    nexttile
    for seg = 1:length(logger)
        plot(logger(seg).flow.t, logger(seg).flow.d1ya_position(i,:)-logger(seg).flow.d1yd_position(i,:));
        hold on
    end
end
title(t,'Velocity Error');