function ringOscillator

clf;
hold on;

theta = linspace(0,2*pi,100);
plot(sin(theta),cos(theta),'Color',.4*[1 1 1],'LineWidth',2);

[X1,X2] = meshgrid(1.2*linspace(-1,1,14),1.2*linspace(-1,1,14));
alpha = 2;
R = sqrt(X1.^2 + X2.^2);
X1dot = -alpha*X1.*(1-1./R)+X2;
X2dot = -alpha*X2.*(1-1./R)-X1;
quiver(X1,X2,X1dot,X2dot);

axis(1.2*[-1,1,-1,1]);
axis equal;
addXYAxis('$x_1$','$x_2$');

export2svg(mfilename);


