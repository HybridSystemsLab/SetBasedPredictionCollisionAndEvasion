close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates

x0 = [0;0;0];     % initial coordinates
N = 10;         % prediction horizon
ts = 0.05;      % sampling time
target = [-1;-1;-1]; % target coordinates
xObst = -0.5*ones(3,N+1);
threshold = 0.1;

% projectile
%p0 = [-0.1;-0.1;0.15;0;0;0];
%simTime = N*ts;
%[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
%xObst = transpose(projPos);
%xdotObst = transpose(projVel);


uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold)

