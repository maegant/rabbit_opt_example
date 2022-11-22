function plotStillFrames(behavior,logger,folder)
f = figure(20); clf; hold on;
screenSize = get(0,'ScreenSize');
figSize = [1200/2 280*1]; %half of page width by num rows
f.Position = [(screenSize(3)-figSize(1))/2 (screenSize(4)-figSize(2))/2 figSize];

% choose number of desired still frames
num_stills = 3;

% extract t and q from logger
num_steps = 10;

t_log = []; q_log = [];
for i = 1:num_steps
    t = logger(i).flow.t;
    log_inds = round(linspace(1,length(t),num_stills));
    t_log = cat(2,t_log,t(log_inds));
    q_log = cat(2,q_log,logger(i).flow.states.x(:,log_inds));
end

% Define colors
leg1Color = 'r'; %right leg
leg2Color = 'b'; %left leg
torsoColor = 'k';
groundColor = 'g';
width = 2;
alphas = linspace(0.2,1,num_stills);
alphas = repmat(alphas,1,num_steps);


% plot ground
if isfield(behavior.options,'Slope')
    [terrain.Tx, terrain.Ty] = meshgrid(-10:1:10, -10:1:10);
    slope = behavior.options.Slope;
    terrain.Tz = slope*terrain.Tx;
elseif isfield(behavior.options,'Step')
    [terrain.Tx, terrain.Ty] = meshgrid(-1:0.05:10, -1:0.05:10);
    step = behavior.options.Step;
    step_x = behavior.options.StepX;
    terrain.Tz = ustep(terrain.Tx,step_x)*step;
end
ground = surf(terrain.Tx,terrain.Ty,terrain.Tz); hold on;
hold on;

% set limits
axis equal
xlim(gca,[-1,4]);
zlim(gca,[-0.1,1.5]);

% set view (west view)
view(gca, 0, 0);

% plot static frames
for i = 1:length(t_log)
    pT = p_Torso(q_log(:,i));
    pH = p_RightHip(q_log(:,i));
    pRK = p_RightKnee(q_log(:,i));
    pLK = p_LeftKnee(q_log(:,i));
    pR = p_RightSole(q_log(:,i));
    pL= p_LeftSole(q_log(:,i));
    
    % plot swing leg
    p(3) = plot3([pH(1) pLK(1)],[0 0], [pH(3) pLK(3)],'LineWidth',width,'Color',leg2Color);
    p(4) = plot3([pLK(1) pL(1)],[0 0], [pLK(3) pL(3)],'LineWidth',width,'Color',leg2Color);
    
    % plot stance leg
    p(1) = plot3([pH(1) pRK(1)],[0 0], [pH(3) pRK(3)],'LineWidth',width,'Color',leg1Color);
    p(2) = plot3([pRK(1) pR(1)],[0 0], [pRK(3) pR(3)],'LineWidth',width,'Color',leg1Color);
    
    % plot torso
    p(5) = plot3([pH(1) pT(1)],[0 0], [pH(3) pT(3)],'LineWidth',width,'Color',torsoColor);
    
    for c = 1:length(p)
        p(c).Color(4) = alphas(i);
    end
    grid(gca,'off')
end

labels = {'Stance Leg','Nonstance Leg','Torso'};
l = legend([p(1),p(3),p(5)],labels);
l.Orientation = 'horizontal';
l.Location = 'northoutside';

xlabel('x (m)');
ylabel('y');
zlabel('z (m)');

latexify; fontsize(12);

%% Draw barrier
% copied from SwingFootBarrier
min_sl = 0.3;
min_sh = 0.05;
max_sl = 0.4;
max_sh = 0.1;

x1 = linspace(-min_sl,min_sl,100);
x2 = linspace(-max_sl,max_sl,100);

% t = @(x) 1./(1+(2*x)); 
t = @(x) 1;
% h1 = ((nsf_x.^2)./((min_sl)^2))+((z.^2)./((min_sh)^2)).*t - 1 == 0;
% h2 = ((nsf_x.^2)./((max_sl)^2))+((z.^2)./((max_sh)^2)).*t - 1 == 0;
h1_z = @(x) abs(sqrt((1 - ((x.^2)./((min_sl)^2))).*((((min_sh)^2))./t(x))));
h2_z = @(x) abs(sqrt((1 - ((x.^2)./((max_sl)^2))).*((((max_sh)^2))./t(x))));

X1 = [x2,fliplr(x1)]; Y1 = [h2_z(x2),fliplr(h1_z(x1))];
% fill3(X1,zeros(size(X1)),Y1,'y','FaceAlpha',0.2,'EdgeColor','k'); hold on



% plot3(x1,zeros(size(x1)),h1_z(x1),'LineWidth',2,'Color','k'); hold on;
% plot3(x2,zeros(size(x2)),h2_z(x2),'LineWidth',2,'Color','k');

%% Save Figure
if ~isfolder(folder)
    mkdir(folder);
end

name = fullfile(folder,'stillframes_steps.png');
saveas(f,name);
% f.PaperPositionMode = 'auto';
% print(f,name,'-bestfit','-dpdf','-r300');
% system(['pdfcrop ',name,' ',name]);