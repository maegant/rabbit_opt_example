function plotGaitTiles(anim,logger,folder)
    
t_log = logger(1).flow.t;

% Plot 8 gait tiles
times = linspace(t_log(1),t_log(end),16);

for i = 1:16
anim.Animate(false);
anim.Draw(times(i),anim.GetData(times(i)));
anim.updateWorldPosition = true;
grid(gca,'off')
figname =  fullfile(folder,sprintf('GaitTile%i.png',i));
saveas(gcf,figname)
end