function mintime_double_int

clf; hold on;

q = linspace(-10,10,200);
qdot = -sign(q).*sqrt(2*sign(q).*q);
fill([q(1),q,q(end),q(end)],[qdot(1),qdot,qdot(end),qdot(1)],MITred,'EdgeColor','none');
fill([q(1),q,q(end),q(end)],[qdot(end),qdot,qdot(end),qdot(end)],MITgray,'EdgeColor','none');

q = linspace(0,10,100);
qdot = -sign(q).*sqrt(2*sign(q).*q);
plot(q,qdot,'LineWidth',2,'Color',.6*MITred);

q = linspace(-10,0,100);
qdot = -sign(q).*sqrt(2*sign(q).*q);
plot(q,qdot,'LineWidth',2,'Color',.6*MITgray);

h = legend('u=-1','u=1');
set(h,'FontName','Times','FontSize',16);

axis equal; axis tight; 
addXYAxis;

text(9.5,-0.6,'$q$','Interpreter','latex','FontName','Times','FontSize',20);
text(-.6,.9*qdot(1),'$\dot{q}$','Interpreter','latex','FontName','Times','FontSize',20);

export2svg('mintime_double_int');

% to do: add arrowheads to the lines?