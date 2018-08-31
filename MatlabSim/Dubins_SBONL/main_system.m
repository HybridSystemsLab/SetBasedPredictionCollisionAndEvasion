close all;
clc;
clear;

%% Description
%
%
% This file uses a set based predictive optimization frame work to return
% the optimal input for a system the uses Dubins model.

global i;
i = 0;

%% Optimization parameters

% vehicle length
L = 0.4;

% input bounds
speed = 3;
steer = pi/6;

% prediction horizon
N = 8;                      

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE IF YOU CHANGE HERE
ts = 0.05;  

% target coordinates
target = [-0.5;-0.5;0];

% threshold distance value for obstacle
threshold = 0.05;      

% length of prediction simulation for projectile
simTime = N*ts;


%% Optimization

% OBTAIN VEHICLE STATE - (x1, x2, y1, y2, theta)
x0 = [0.0;0.05;0.0;0.05;(5*pi)/4];

% OBTAIN PROJECTILE STATE - (x, y, z, xdot, ydot, zdot)
p0 = [-0.4;-0.07;0.3;1.5;0.0;0.0];

% PREDICT PROJETILE STATE
projPos = ProjectilePredict(p0,simTime);
xObst = transpose(projPos);

% FIND OPTIMAL INPUT

uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer);

uopt

agentPos = Dubin(x0, uopt, ts,L)
PlotOptimalPredicted(agentPos, xObst, threshold, target)

    

