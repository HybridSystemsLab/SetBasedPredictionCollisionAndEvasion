close all;
clc;
clear;

%% Simulations parameters

% vehicle length
L = 0.4;

% input bounds
speed = 1;
steer = pi/4;


% prediction horizon
N = 8;                      

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
ts = 0.05;  

% number of iterations to run
iterations = 35;

% target coordinates
target = [-0.5;-0.5;0];

% initial condition of the vehicle
% (x1, x2, y1, y2, theta)
x0 = [0.50;0.55;0.5;0.55;(3*pi)/2];

% initial condition of the projectile
% (x, y, z, xdot, ydot, zdot)
%p0 = [-0.5;-0.1;0.0;1.8;0.15;1.0];
p0(1,:) = [0.2;0.03;-2.35;0.0;0.0;6.9];
p0(2,:) = [0.4;0.2;0.1;0;0;1.7];

% threshold distance value for obstacle
threshold = 0.1;          


%% Simulation core

% length of prediction simulation for projectile
simTime = N*ts;

% projectile prediction
[projPos(:,:,1), projVel(:,:,1)] = SimulationProjectilePredict(p0(1,:),simTime);
[projPos(:,:,2), projVel(:,:,2)] = SimulationProjectilePredict(p0(2,:),simTime);
xObst(:,:,1) = transpose(projPos(:,:,1));
xdotObst(:,:,1) = transpose(projVel(:,:,1));
xObst(:,:,2) = transpose(projPos(:,:,2));
xdotObst(:,:,2) = transpose(projVel(:,:,2));
% static obst
xObst(:,:,3) = [-0.2;-0.2;-0].*ones(3,N+1); 

% size of vehicle and obstacle arays
[m,n] = size(x0);
[mO,nO,pO] = size(xObst);

% position arrays for plotting
agentPos = zeros(m,iterations+1);
u = zeros(2,iterations+1);
obstPos = zeros(3,iterations+1,pO);

% initial conditions
agentPos(:,1) = x0;

obstPos(:,1,:) = xObst(:,1,:);

for i = 1:iterations
    fprintf('Simulation iteration %d%% complete\n',ceil((i/iterations)*100));

    % find next optimal input
    uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer);
    u(:,i) = uopt(:,1);
    
    % find next position of agent given the optimal input
    nextState = Dubin(x0,uopt,ts,L);
    x0 = nextState(:,2);
    
    
    % next prediction for projectile
    for j = 1:(pO-1)
        p0 = [xObst(:,2,j);xdotObst(:,2,j)];
        [projPos, projVel] = SimulationProjectilePredict(p0,simTime);
        xObst(:,:,j) = transpose(projPos);
        xdotObst(:,:,j) = transpose(projVel);
    end
    % store for graphing
    
    % (coord,point,time)
    agentPos(:,i+1) = transpose(x0);
    
    % (coord,time,obst)
    obstPos(:,i+1,:) = xObst(:,1,:); % quick fix to store obst location

end

% plot simulations results
PlotSetBasedSim(agentPos, obstPos, threshold, target);
PlotSimDistance(agentPos, obstPos, threshold, target);





