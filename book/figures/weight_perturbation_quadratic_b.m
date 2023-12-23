eta = 0.1;
sigma = 0.5;
N = 1:10;
m = 100000;

rand('twister',sum(100*clock));
randn('state',sum(100*clock));

%A0 = rand(N(end),N(end));

for n=N

%  A = A0(1:n,1:n); 
  A = eye(n);
  w = 4*ones(n,1);%rand(n,1);
  
  z = sigma*randn(n,m);  % Gaussian noise
  wz = repmat(w,1,m) + z;
  dw = eta*repmat(sum(wz.*(A*wz),1) - repmat(w'*A*w,1,m),n,1).*z;
  mean_dw = mean(dw,2);
  snr_gaussian(n) = mean_dw'*mean_dw / (mean(sum((dw-repmat(mean_dw,1,m)).^2,1),2));

  z = sigma*(2*rand(n,m)-1);  % Uniform noise
  wz = repmat(w,1,m) + z;
  dw = eta*repmat(sum(wz.*(A*wz),1) - repmat(w'*A*w,1,m),n,1).*z;
  mean_dw = mean(dw,2);
  snr_uniform(n) = mean_dw'*mean_dw / (mean(sum((dw-repmat(mean_dw,1,m)).^2,1),2));

end

plot(N,snr_gaussian,N,1./(N+1),N,snr_uniform,N,1./(N-0.2));
xlabel('N (number of parameters)');
ylabel('SNR');
legend('Gaussian (exp)','Gaussian (theory)','Uniform (exp)','Uniform (theory)');

