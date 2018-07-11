close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates

x0 = [0;0];     % initial coordinates
N = 10;         % prediction horizon
ts = 0.05;      % sampling time
target = [1;1]; % target coordinates

uopt = FindOptimalInput(x0, N, ts, target);