clc;
clear all;
close all;
A = [1.1269   -0.4940    0.1129,
     1.0000         0         0,
          0    1.0000         0];

B = [-0.3832
      0.5919
      0.5191];

C = [1 0 0];

D = 0;
Q=2.3;
R=1;
t = (0:100)';
u = sin(t/5);
w = sqrt(Q)*randn(length(t),1);
v = sqrt(R)*randn(length(t),1);
sys = ss(A,B,C,D,-1);
y = lsim(sys,u+w);   % w = process noise
yv = y + v;          % v = measured noise
P=B*Q*B';         % Initial error covariance
x=zeros(3,1);     % Initial condition on the state
ye = zeros(length(t),1);
ycov = zeros(length(t),1);
errcov = zeros(length(t),1);

for i=1:length(t)
  % Measurement update
  Mn = P*C'/(C*P*C'+R);
  x = x + Mn*(yv(i)-C*x);  % x[n|n]
  P = (eye(3)-Mn*C)*P;     % P[n|n]

  ye(i) = C*x;
  errcov(i) = C*P*C';

  % Time update
  x = A*x + B*u(i);        % x[n+1|n]
  P = A*P*A' + B*Q*B';     % P[n+1|n]
end
figure;
subplot(211), plot(t,y,'b',t,ye,'r--'),
xlabel('No. of samples'), ylabel('Output')
title('Response with time-varying Kalman filter')
subplot(212), plot(t,y-yv,'b',t,y-ye,'r--'),
xlabel('No. of samples'), ylabel('Error')
figure;
subplot(211)
plot(t,errcov), ylabel('Error Covar')
subplot(212), plot(t,y-yv,'b',t,y-ye,'r--'),
xlabel('No. of samples'), ylabel('Error'),
MeasErr = y-yv;
MeasErrCov = sum(MeasErr.*MeasErr)/length(MeasErr);
EstErr = y-ye;
EstErrCov = sum(EstErr.*EstErr)/length(EstErr);
MeasErrCov
EstErrCov
