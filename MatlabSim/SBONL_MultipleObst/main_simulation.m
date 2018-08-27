close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates
x0 = [0;0;0];               % initial coordinates
N = 3;                      % prediction horizon
ts = 0.05;                  % sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
target = [-0.1;-0.1;-0.1];  % target coordinates
threshold = 0.05;          % threshold distance value
%{
xObst = zeros(3,N+1,2);  % obstacle coordinate - stationary
xObst(1,:,1) = -0.13*ones(1,N+1);
xObst(2,:,1) = -0.12*ones(1,N+1);
xObst(3,:,1) = -0.15*ones(1,N+1);
xObst(1,:,2) = -0.05*ones(1,N+1);
xObst(2,:,2) = -0.07*ones(1,N+1);
xObst(3,:,2) = -0.05*ones(1,N+1);
%}


x0_set = [0.0 0.02 0.02 0.0 0.0 0.02 0.02 0.0;
          0.0 0.0 0.02 0.02 0.0 0.0 0.02 0.02;
          0.0 0.0 0.0 0.0 0.02 0.02 0.02 0.02];
      


% projectile
%p0 = [-0.5;-0.1;0.0;1.8;0.15;1.0];
p0 = [0.15;-0.15;-0.2;-0.9;0.7;2.1];
%p0 = [-0.05;-0.05;0;0;0;0.5];


simTime = N*ts;
[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
xObst = transpose(projPos);
xdotObst = transpose(projVel);

[m,n] = size(x0_set);
[mO,nO,pO] = size(xObst);

iterations = 10;
agentPos = zeros(3,n,iterations+1);
obstPos = zeros(3,iterations+1,pO);

agentPos(:,:,1) = x0_set;
obstPos(:,1,1) = xObst(:,1,1);
%obstPos(:,1,2) = xObst(:,1,2);
for i = 1:iterations
    fprintf('Simulation iteration %d\n',i);
    transpose(projPos);
    % find next optimal input
    uopt = FindOptimalInput(x0_set, N, ts, target, xObst, threshold);
    
    % find next position of agent given the optimal input
    nextState = SingleIntegrator(x0_set,uopt,ts);
    %x0_set = nextState(:,2,:);
    
    %PlotOptimalPredicted(nextState, transpose(projPos), threshold, target)
    
    
    % format x0_set 
    for j = 1:n
        x0_set(:,j) = nextState(:,2,j);
    end
    
    
    % next prediction for projectile
    p0 = [xObst(:,2);xdotObst(:,2)];
    %p0 = [xObst(:,1);xdotObst(:,1)];
    [projPos, projVel] = SimulationProjectilePredict(p0,simTime);
    xObst = transpose(projPos);
    xdotObst = transpose(projVel);
    
    % store for graphing
    
    % (coord,point,time)
    agentPos(:,:,i+1) = x0_set;
    
    % (coord,time,obst)
    obstPos(:,i+1,:) = xObst(:,1,:); % quick fix to store obst location

end

% plot simulations results
PlotSetBasedSim(agentPos, obstPos, threshold, target);
PlotSimDistance(agentPos, obstPos, threshold, target);





