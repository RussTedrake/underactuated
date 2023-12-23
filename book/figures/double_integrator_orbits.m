function double_integrator_orbits

clf; hold on;

qdot = linspace(-4,4,200);

axis equal; 
axis([-10,10,-4,4]);

for q0=[-1,9]
  qdot0=0;
  q = -.5*qdot.^2 + q0 + .5*qdot0^2;
  plot(q,qdot,'Color',MITred);
  pt=25; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
  pt=175; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
end

addXYAxis;

text(9.5,-0.6,'$q$','Interpreter','latex','FontName','Times','FontSize',20);
text(-.6,.9*max(qdot),'$\dot{q}$','Interpreter','latex','FontName','Times','FontSize',20);

export2svg('double_integrator_orbits');


clf; hold on;

axis equal; 
axis([-10,10,-4,4]);

qdot = linspace(0,4,100);
q = -.5*qdot.^2;
plot(q,qdot,'Color',MITred);
pt=40; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
pt=70; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);

q = .5*qdot.^2;
plot(q,-qdot,'Color',MITgray);
pt=40; axisAnnotation('arrow',[q([pt+1,pt])],[-qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
pt=75; axisAnnotation('arrow',[q([pt+1,pt])],[-qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);

qdot = linspace(-4,4,200);
for q0 = [-5,-9] 
q = .5*qdot.^2 + q0;
idx = qdot<sqrt(-2*q);
plot(q(idx),qdot(idx),'Color',MITgray);
pt=40; axisAnnotation('arrow',[q([pt-1,pt])],[qdot([pt-1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
pt=70; axisAnnotation('arrow',[q([pt-1,pt])],[qdot([pt-1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);

q = -.5*qdot.^2 - q0;
idx = qdot>-sqrt(2*q);
plot(q(idx),qdot(idx),'Color',MITred);
pt=160; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);
pt=70; axisAnnotation('arrow',[q([pt+1,pt])],[qdot([pt+1,pt])],'Color','k','HeadWidth',200,'HeadLength',200);

h = legend('u=-1','u=1');
set(h,'FontName','Times','FontSize',16);

addXYAxis;

text(9.5,-0.6,'$q$','Interpreter','latex','FontName','Times','FontSize',20);
text(-.6,.9*max(qdot),'$\dot{q}$','Interpreter','latex','FontName','Times','FontSize',20);


export2svg('double_integrator_mintime_orbits');


end

