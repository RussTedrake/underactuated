function limitCycleLyapunovCartoon

clf; hold on;

[Theta,R] = meshgrid(linspace(3*pi/4-.5,2*pi+.3,50),.8:.02:1.2);
%[Theta,R] = meshgrid(linspace(0,2*pi,50),.8:.02:1.2);
V = (R-1).^2;

th = linspace(0,2*pi,50);
plot(sin(th),cos(th),'k','LineWidth',4);
surf(R.*sin(Theta),R.*cos(Theta),V);
axis([-1.5,1.5,-1.5,1.5,0,.15]);
axis off;
view(45,30);
h=addXYAxis; set(h,'Marker','none');
line([0 0],[0 0],[-.1,.12],'Color',[0 0 0]);
text(1.6,0,0,'$x_1$','Interpreter','latex','FontSize',18)
text(0,1.6,0,'$x_2$','Interpreter','latex','FontSize',18)
text(.1,0,.11,'$V(x)$','Interpreter','latex','FontSize',18)


colormap('summer');
export2svg(mfilename);
