close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates

x0 = [0;0;0];     % initial coordinates
N = 3;         % prediction horizon
ts = 0.05;      % sampling time
target = [-1;-1;-1]; % target coordinates
xObst = -0.3*ones(3,N+1);
threshold = 0.1;

x0_set = CreateSphere(x0,0.1,3,3);



% projectile
%p0 = [-0.1;-0.1;0.15;0;0;0];
%simTime = N*ts;
%[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
%xObst = transpose(projPos);
%xdotObst = transpose(projVel);


uopt = FindOptimalInput(x0_set, N, ts, target, xObst, threshold)

