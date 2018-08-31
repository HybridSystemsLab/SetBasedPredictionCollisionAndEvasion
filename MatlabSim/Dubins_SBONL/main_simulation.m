close all;
clc;
clear;

%% Simulations parameters

% vehicle length
L = 0.4;

% input bounds
speed = 1;
steer = pi/6;


% prediction horizon
N = 5;                      

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
ts = 0.05;  

% number of iterations to run
iterations = 25;

% target coordinates
target = [-0.6;-0.4;0];

% initial condition of the vehicle
% (x1, x2, y1, y2, theta)
x0 = [0.0;0.05;0.0;0.05;12*pi/11];

% initial condition of the projectile
% (x, y, z, xdot, ydot, zdot)
%p0 = [-0.5;-0.1;0.0;1.8;0.15;1.0];
p0 = [-0.2;-0.4;-0.2;0.25;1.4;2.1];
%p0 = [-0.05;-0.05;0;0;0;0.5];

% threshold distance value for obstacle
threshold = 0.05;          


%% Simulation core

% length of prediction simulation for projectile
simTime = N*ts;

% projectile prediction
[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
xObst = transpose(projPos);
xdotObst = transpose(projVel);

% size of vehicle and obstacle arays
[m,n] = size(x0);
[mO,nO,pO] = size(xObst);

% position arrays for plotting
agentPos = zeros(m,iterations+1);
obstPos = zeros(3,iterations+1,pO);

% initial conditions
agentPos(:,1) = x0;
obstPos(:,1,1) = xObst(:,1,1);

for i = 1:iterations
    fprintf('Simulation iteration %d\n',i);

    % find next optimal input
    uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer);
    
    % find next position of agent given the optimal input
    nextState = Dubin(x0,uopt,ts,L);
    x0 = nextState(:,2);

    
    % next prediction for projectile
    p0 = [xObst(:,2);xdotObst(:,2)];
    [projPos, projVel] = SimulationProjectilePredict(p0,simTime);
    xObst = transpose(projPos);
    xdotObst = transpose(projVel);
    
    % store for graphing
    
    % (coord,point,time)
    agentPos(:,i+1) = transpose(x0);
    
    % (coord,time,obst)
    obstPos(:,i+1,:) = xObst(:,1,:); % quick fix to store obst location

end

% plot simulations results
PlotSetBasedSim(agentPos, obstPos, threshold, target);
PlotSimDistance(agentPos, obstPos, threshold, target);





