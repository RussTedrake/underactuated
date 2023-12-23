function transverseCoordinates

clf;
hold on;

theta = linspace(0,2*pi,100);
plot(sin(theta),cos(theta),'Color',.4*[1 1 1],'LineWidth',2);

theta = pi/4;
plot(sin(theta),cos(theta),'b.','MarkerSize',35);

h = [ ...
  axisAnnotation('arrow',sin(theta)+.7*[0,cos(theta)],cos(theta)+.7*[0,-sin(theta)],'LineWidth',1.5); ...
  axisAnnotation('arrow',sin(theta)+.7*[0,sin(theta)],cos(theta)+.7*[0,cos(theta)],'LineWidth',1.5); ...
  ];
set(h,'Color','r');
text(1.2,.2,'$f({\bf x}^*(\tau))$','Interpreter','latex','FontSize',20);

r=1:.1:1.4;
plot(r*sin(theta),r*cos(theta),'Color','k','LineWidth',1.5);
plot(1.4*sin(theta),1.4*cos(theta),'b.','MarkerSize',25);
text(.72,.95,'${\bf x}_\perp$','Interpreter','latex','FontSize',20);
text(1.05,1,'${\bf x}$','Interpreter','latex','FontSize',20);
axisAnnotation('arrow',1.4*sin(theta)+[0,.2],1.4*cos(theta)+[0,-.5],'Color','b','LineWidth',1.5);
text(1.2,.5,'$f({\bf x})$','Interpreter','latex','FontSize',20);


r=[.6,.95];
line(r*sin(theta),r*cos(theta),'Color','k');
r=.8; theta = 0.05:.05:pi/4;
plot(r*sin(theta),r*cos(theta),'Color','k');
axisAnnotation('arrow',r*sin(theta(end-1:end)),r*cos(theta(end-1:end)));
text(.25,.65,'$\tau$','Interpreter','latex','FontSize',20);


axis(1.2*[-1,1,-1,1]);
axis equal;
addXYAxis('$x_1$','$x_2$');

export2svg(mfilename);


