close all;
clc;
clear;

%% Simulations parameters

% vehicle length
L = 0.4;

% input bounds
speed = 1.52;
steer = pi/6;


% prediction horizon
N = 8;             

% terminal cost weight
terminalWeight = 1;

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
ts = 0.05;  

% number of iterations to run
iterations = 25;

% target coordinates
target = [-0.5;-0.5;0];

% initial condition of the vehicle
% (x1, x2, y1, y2, theta)
% initial condition of the projectile
% (x, y, z, xdot, ydot, zdot)

x0(:,1) = [0.0;0.05;0.0;0.05;(5*pi)/4];
p0(:,1) = [-0.4;-0.2;0.2;1.5;0.4;0.0];

x0(:,2) = [0.22;0.27;0.22;0.27;(5*pi)/4];
p0(:,2) = [-0.9;-0.2;0.0;1.8;0.15;2.0];

x0(:,3) = [0.02;0.07;0.02;0.07;(5*pi)/4];
p0(:,3) = [-0.5;-0.3;0.2;1.5;0.4;0.0];

x0(:,4) = [0.1;0.15;0.1;0.15;(5*pi)/4];
p0(:,4) = [-0.5;-0.3;0.2;0.7;0.0;1.2];

x0(:,5) = [0.4;0.45;0.4;0.45;(13*pi)/8];
p0(:,5) = [-1.2;-1.80;1.9;1.5;1.6;1.5];

x0(:,6) = [0.4;0.45;0.4;0.45;(3*pi)/2];
p0(:,6) = [-0.40;-0.8;-0.2;1.5;1.6;2.5];

x0(:,7) = [0.1;0.15;0.1;0.15;(4*pi)/3];
p0(:,7) = [-0.4;-0.1;-0.1;1.5;0.0;1.6];

x0(:,8) = [0.15;0.20;0.15;0.20;(4*pi)/3];
p0(:,8) = [-0.2;-0.2;-0.1;0.2;0.2;1.6];

x0(:,9) = [0.08;0.13;0.08;0.13;(4*pi)/3];
p0(:,9) = [-0.3;-0.33;-0.74;0.2;0.2;4.0];

x0(:,10) = [-0.15;-0.10;-0.15;-0.10;(11*pi)/8];
p0(:,10) = [-0.7;-0.33;-0.74;0.2;0.2;4.0];


% threshold distance value for obstacle
threshold = 0.05;          


%% Simulation core

% length of prediction simulation for projectile
simTime = N*ts;

[m,n] = size(x0);

for i = 1:n
    
    % projectile prediction
    [projPos, projVel] = SimulationProjectilePredict(p0(:,i),simTime);
    xObst = transpose(projPos);
    xdotObst = transpose(projVel);

    % size of vehicle and obstacle arays
    [m,n] = size(x0);
    [mO,nO] = size(xObst);

    % position arrays for plotting
    agentPos = zeros(m,iterations+1,n);
    obstPos = zeros(3,iterations+1,n);

    % initial conditions
    agentPos(:,1,i) = x0(:,i);
    obstPos(:,1,i) = xObst(:,1);
    
    for j = 1:iterations
        fprintf('Simulation iteration %d%% complete\n',ceil((j/iterations)*100));

        % find next optimal input
        uopt = FindOptimalInput(x0(:,i), N, ts, target, xObst, threshold, L, speed, steer, terminalWeight);

        % find next position of agent given the optimal input
        nextState = Dubin(x0(:,i),uopt,ts,L);
        x0(:,i) = nextState(:,2);


        % next prediction for projectile
        p0(:,i) = [xObst(:,2);xdotObst(:,2)];
        [projPos, projVel] = SimulationProjectilePredict(p0(:,i),simTime);
        xObst = transpose(projPos);
        xdotObst = transpose(projVel);

        % store for graphing

        % (coord,point,time)
        agentPos(:,j+1,i) = transpose(x0(:,i));

        % (coord,time)
        obstPos(:,j+1,i) = xObst(:,1);

    end
    % plot simulations results
    PlotSimDistanceSuccessive(agentPos(:,:,i), obstPos(:,:,i), threshold, target);
end







