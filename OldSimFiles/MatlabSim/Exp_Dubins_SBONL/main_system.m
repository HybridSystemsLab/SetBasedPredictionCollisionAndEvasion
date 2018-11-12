close all;
clc;
clear;

%% Description
%
%
% This file uses a set based predictive optimization frame work to return
% the optimal input for a system that uses Dubins model.


%% Optimization parameters

% Set EXP to 1 for experiments - approx polytope distance function
% set EXP to 0 for simulations - fmincon polytope distance function
EXP = 1;

% vehicle length
L = 0.4;

% input bounds
speed = 1.52;
steer = pi/6;

% prediction horizon
N = 8;    

% terminal cost weight
terminalWeight = 1;

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE IF YOU CHANGE HERE
ts = 0.025;  

% target coordinates
target = [-0.5;-0.5];

% threshold distance value for obstacle
threshold = 0.05;      

% length of prediction simulation for projectile
simTime = N*ts;


%% Optimization

% This is where the loops would start, basic outline is as follows
%
% 1. obtain vehicle state
% 2. obtain projectile state
% 3. predict projectile
% 4. find optimal input with FindOptimalInput() function
% 5. apply optimal input
% 6. repeat 1-5

x0 = [-0.05;-0.0;-0.05;-0.0;(11*pi)/8];
p0 = [-0.6;-0.65;-0.3;-0.35;0];

%x0 = [-0.17;-0.12;-0.2426;-0.1926;3.9906];
%p0 = [-0.3720;-0.4220;-0.3000;-0.3500;0];
uObst = [speed;0].*ones(2,N);
xObst = Dubin(p0,uObst,ts,L);

% FIND OPTIMAL INPUT
tic
uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer, terminalWeight, EXP);
toc

% THIS IS THE NEXT OPTIMAL INPUT
uopt(:,1)


agentPos = Dubin(x0, uopt, ts,L);
PlotOptimalPredicted(agentPos, xObst, threshold, target)

    

