eta = 0.01;
sigma = 0.5;
n = 2;
A = 0; while (rank(A) < n) A = randn(n,n); end, A = A'*A;  % generate rand pos def A 
%A = eye(n);


[w1,w2] = ndgrid(linspace(-5,5,20),linspace(-5,5,20));
Y = reshape( sum([w1(:),w2(:)]'.*(A*[w1(:),w2(:)]'),1) , size(w1));
surf(w1,w2,Y);
colormap gray;

w = 4*ones(n,1);

% run reinforce
for i=1:1000
  z = sigma*randn(n,1);         % Gaussian noise
  
  wn = w - eta*[(w+z)'*A*(w+z) - w'*A*w]*z;

  y = w'*A*w; yn = wn'*A*wn;
  line([w(1),wn(1)],[w(2),wn(2)],[y,yn]+1,'LineWidth',3,'Color',[1 0 0]);

  w = wn;
end
