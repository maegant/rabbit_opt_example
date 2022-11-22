% xall = -0.5:0.005:0.5;  % plotting range from -5 to 5
% yall = -0.5:0.005:0.5;
r = -0.5:0.001:0.5;
[x, y] = meshgrid(r);  % Get 2-D mesh for x and y based on r

%%%% Attempt 1 with circles/ellipses
slmin = 0.3;
slmax = 0.4;
shmin = 0.05;
shmax = 0.1;
% 
% q = -0.5; %parameter on sharpness
% b = 0.3; %parameter on horizontal displacement
% h = 30; %parameter on height

% curve = (x.^2+(q*h*y.^2)).^2-(2*b.^2*(x.^2-(h*y.^2))) +b^2 > 0;
% curve = x.^2 + (1.4.^x*1.6.*y)^2 - 1 > 0;
% k = 0.3;
% t = (1-(k.*x))./(1+(k.*x));
t = 1;
h1 = (x.^2/(slmin^2)) + (y.^2/(shmin^2))*t - 1 > 0;
h2 = -((x.^2/(slmax^2)) + (y.^2/(shmax^2))*t - 1) > 0;

%%% plotting:
output = ones(length(r)); % Initialize to 1
output(~(h1 & h2)) = 0; % Zero out coordinates not meeting conditions.
imshow(output, 'xdata', r, 'ydata', r); % Display
axis on;
set(gca, 'YDir','normal')

xlabel('x');
ylabel('y');

