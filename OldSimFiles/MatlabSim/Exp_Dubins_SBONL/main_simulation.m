close all;
clc;
clear;

%% Simulations parameters

% Set EXP to 1 for experiments - approx polytope distance function
% set EXP to 0 for simulations - fmincon polytope distance function
EXP = 1;

% vehicle length
L = 0.4;

% input bounds
%speed = 1.52;
speed = 0.4;
steer = pi/6;

% terminal cost weight
terminalWeight = 1e8;

% prediction horizon
N = 7;                      

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
ts = 0.05;  

% number of iterations to run
iterations = 45;

% target coordinates
target = [1.25;-1.25];

x0 = [-1.55;-1.25;0.75;1.0;0];
p0 = [0.0;0.2;-0.1;0.1;(pi)/1];
uObst = [0;steer].*ones(2,N);

% initial condition of the vehicle
% (x1, x2, y1, y2, theta)
% initial condition of the obstace
% (x1, x2, y1, y2, theta)

%x0 = [-0.05;-0.0;-0.05;-0.0;(5*pi)/4];
%p0 = [0.0;0.05;-0.25;-0.20;(pi)/1];
%uObst = [speed;-steer].*ones(2,N);

%x0 = [-0.05;-0.0;-0.05;-0.0;(5*pi)/4];
%p0 = [0.0;0.05;-0.25;-0.20;(pi)/1];
%uObst = [speed;steer].*ones(2,N);

%x0 = [-0.05;-0.0;-0.05;-0.0;(5*pi)/4];
%p0 = [0.1;0.15;-0.25;-0.20;(pi)/1];
%uObst = [speed;steer].*ones(2,N);

%x0 = [-0.15;-0.1;-0.05;-0.0;(5*pi)/4];
%p0 = [-0.4;-0.45;-0.65;-0.6;(7*pi)/16];
%uObst = [speed;-steer].*ones(2,N);

%x0 = [-0.05;-0.0;-0.2;-0.25;(5*pi)/4];
%p0 = [-0.1;-0.15;0.1;0.05;(23*pi)/16];
%uObst = [speed;0].*ones(2,N);

%x0 = [-0.05;-0.0;-0.2;-0.25;(5*pi)/4];
%p0 = [-0.1;-0.15;0.1;0.05;(23*pi)/16];
%uObst = [0;0].*ones(2,N);


xObst = Dubin(p0,uObst,ts,L);


% threshold distance value for obstacle
threshold = 0.05;          


%% Simulation core

% length of prediction simulation for projectile
simTime = N*ts;

% size of vehicle and obstacle arays
[m,n] = size(x0);
[mO,nO] = size(xObst);

% position arrays for plotting
agentPos = zeros(m,iterations+1);
obstPos = zeros(5,iterations+1);
u = zeros(2,iterations);

% initial conditions
agentPos(:,1) = x0;
obstPos(:,1) = xObst(:,1);


for i = 1:iterations
    fprintf('Simulation iteration %d%% complete\n',ceil((i/iterations)*100));

    % find next optimal input
    uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer, terminalWeight, EXP);
    u(:,i) = uopt(:,1);
    % find next position of agent given the optimal input
    nextState = Dubin(x0,uopt,ts,L);
    %x0 = nextState(:,2);
    x0 = nextState(:,2);

    
    % next prediction for projectile
    p0 = xObst(:,2);
    xObst = Dubin(p0,uObst,ts,L);
    

    % store for graphing
    
    % (coord,point,time)
    agentPos(:,i+1) = transpose(x0);
    
    % (coord,time,obst)
    obstPos(:,i+1) = xObst(:,1); % quick fix to store obst location

end

% plot simulations results
PlotSetBasedSim(agentPos, obstPos, threshold, target);
PlotSimDistance(agentPos, obstPos, threshold, target);
PlotInputs(u,speed,steer)





