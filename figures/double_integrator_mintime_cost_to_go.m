function double_integrator_mintime_cost_to_go

q = linspace(-10,10,51);
qdot = linspace(-4,4,51);

[Q,Qdot] = meshgrid(q,qdot);

uplus = (Qdot<0 & Q<.5*Qdot.^2) | (Qdot>=0 & Q<-.5*Qdot.^2);
T=0*Q;
T(uplus) = 2*sqrt(.5*Qdot(uplus).^2 - Q(uplus)) - Qdot(uplus);
T(~uplus) = Qdot(~uplus) + 2*sqrt(.5*Qdot(~uplus).^2 + Q(~uplus));

surf(q,qdot,T);
xlabel('$q$','Interpreter','latex');
ylabel('$\dot{q}$','Interpreter','latex');
view(50,60);

export2svg(mfilename);
