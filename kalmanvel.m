clc;
clear all;
close all;


dt = 0.1;
A = expm([0 1;0  0]*dt)
B = [0;0]
Q = diag([0;1])
C = [1 0]
D = 0;
R = 1;
u = 0;
x0 = [0;10]

% For Kalman filter, we use the same model
A1 = A;
B1 = B;
C1 = C;
D1 = D;
Q1 = Q;
R1 = R;
x1 = [0;5];     %but with different initial state estimate
P1 = eye(2);

% Run the simulation for 100 smaples
tspan = [0 200];
[t,x,y1,y2,y3,y4] = sim('kalmanfilter',tspan,[],[0 u]);

Xtrue = y1(:,1);    %actual position
Vtrue = y1(:,2);    %actiual velocity
z = y2;             %measured position
X = y3;             %Kalman filter output
t = t*dt;           % actual time

%   Position analysis 

figure;
subplot(211)
plot(t,Xtrue,'g',t,z,'r',t,X(:,1),'m','linewidth',2);
title('Position estimation results');
xlabel('Time (s)');
ylabel('Position (m)');
legend('True position','Measurements','Kalman estimated displacement','Location','NorthWest');


% Velocity analysis  

% The instantaneous velocity as derived from 2 consecutive position measurements
InstantV = [10;diff(z)/dt];

% The instantaneous velocity as derived from running average with a window of 5 samples from instantaneous velocity
WindowSize = 5;
InstantVAverage = filter(ones(1,WindowSize)/WindowSize,1,InstantV);

% figure;
subplot(212)
plot(t,InstantV,'r',t,InstantVAverage,'c',t,Vtrue,'m',t,X(:,2),'k','linewidth',2);
title('Velocity estimation results');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated velocity by raw consecutive samples','Estimated velocity by running average','True velocity','Estimated velocity by Kalman filter','Location','NorthWest');

set(gcf,'Position', [100 100 600 800]);