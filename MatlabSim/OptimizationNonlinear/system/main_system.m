close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates

x0 = [0;0;0];     % initial coordinates
N = 10;         % prediction horizon
ts = 0.05;      % sampling time
target = [-1;-1;-1]; % target coordinates

p0 = [-0.2;-0.2;0;0;0;0];
simTime = N*ts;
xProj = transpose(ProjectilePredict(p0,simTime));

threshold = 0.1;

uopt = FindOptimalInput(x0, N, ts, target, xProj, threshold)