function xcubed()

clf
x = -1.5:0.01:1.5;
y = -x+x.^3;

plot(x,y,'LineWidth',2,'Color',[0 0 0]);

axis([x(1) x(end) -1.1 1.1]);
addXYAxis;

text(1.45,-0.1,'$x$','FontName','Times','FontSize',32,'Interpreter','Latex');
text(-.16,1,'$\dot{x}$','FontName','Times','FontSize',32,'Interpreter','Latex');

h=line([1,1],[-0.2,0.2],'Color',[0 0 0]);
h=[h,text(1.05,-0.1,'$1$','Interpreter','latex','FontSize',24)];
h=[h,line([-1,-1],[-0.2,0.2],'Color',[0 0 0])];
h=[h,text(-.95,-0.1,'-$1$','Interpreter','latex','FontSize',24)];

hold on;
h=[h,plot([-1,1],[0 0],'o','MarkerSize',15,'LineWidth',2,'Color',[0 0 0])];
h=[h,plot([0],[0],'o','MarkerSize',15,'MarkerFaceColor',[0 0 0],'LineWidth',2,'Color',[0 0 0])];

export2svg(mfilename);

delete(h); h=[];

d=[.75,1.125,.5,1.5];
for i=1:length(d);
  y = d(i)*x.^3-x;
  h=[h;plot(x,y,'LineWidth',2,'Color',.6*[1 1 1])];
end

r1 = roots([max(d) 0 -1 0]); a=min(r1); b=max(r1);  
h=[h;line([a,b;a,b],1.1*[-1,-1;1,1],'Color','b','LineWidth',2,'LineStyle','--')];
h=[h;patch([a,a,b,b,a],1.2*[1,-1,-1,1,1],-1*ones(1,5),[.8 .8 1])];
%h=[h;text(0,-.8,'Region of Attraction','FontSize',28,'Color','b')];

export2svg('xcubed_gain_uncertainty');


delete(h); h=[];
d=[-.25,.25,-.125,.125];
for i=1:length(d);
  y = x.^3-x+d(i);
  h=[h;plot(x,y,'LineWidth',2,'Color',.6*[1 1 1])];
end

r1 = roots([1 0 -1 min(d)]);
r2 = roots([1 0 -1 max(d)]);
a = min(r1);  b = max(r2); 
h=line([a,b;a,b],1.1*[-1,-1;1,1],'Color','r','LineWidth',2,'LineStyle','--');
h=[h;patch([a,a,b,b,a],1.2*[1,-1,-1,1,1],-1*ones(1,5),[1 .8 .8])];
h=[h;text(-.5,-.8,'Invariant Set','FontSize',28,'Color','r')];

export2svg('xcubed_additive_uncertainty');
