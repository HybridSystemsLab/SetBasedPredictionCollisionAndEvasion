close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates

x0 = [0;0;0];           % initial coordinates
N = 10;                 % prediction horizon
ts = 0.05;              % sampling time
target = [-1;-1;-1];    % target coordinates
threshold = 0.4;        % threshold distance value

% static obstacle for testing at (-0.5, -0.5, -0.5)
xObst = -0.5*ones(3,N+1);


% projectile
%p0 = [-0.1;-0.1;0.15;0;0;0];
%simTime = N*ts;
%[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
%xObst = transpose(projPos);
%xdotObst = transpose(projVel);

iterations = 40;
agentPos = zeros(3,iterations);
for i = 1:iterations
    
    % find next optimal input
    uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold);
    
    % find next position of agent given the optimal input
    nextState = SingleIntegrator(x0,uopt,ts);
    x0 = nextState(:,end);
    agentPos(:,i) = x0;

end

% plot agent path
scatter3(agentPos(1,:), agentPos(2,:), agentPos(3,:))



