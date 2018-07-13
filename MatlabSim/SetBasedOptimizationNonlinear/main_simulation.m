%close all;
clc;
clear;

%% find the next optimal input given a initial conditions and target coordinates
x0 = [0;0;0];               % initial coordinates
N = 3;                      % prediction horizon
ts = 0.05;                  % sampling time
target = [-0.25;-0.25;-0.25];  % target coordinates
xObst = -0.13*ones(3,N+1);  % obstacle coordinate - stationary
xObst(1,:) = -0.25*ones(1,N+1);
threshold = 0.1;          % threshold distance value



%x0_set = CreateSphere(x0,0.02,4,4);
x0_set = [0.0 0.02 0.02 0.0 0.0 0.02 0.02 0.0;
          0.0 0.0 0.02 0.02 0.0 0.0 0.02 0.02;
          0.0 0.0 0.0 0.0 0.02 0.02 0.02 0.02];
      

%x0_set = x0;
[m,n] = size(x0_set);



% projectile
%p0 = [-0.1;-0.1;0.15;0;0;0];
%simTime = N*ts;
%[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
%xObst = transpose(projPos);
%xdotObst = transpose(projVel);

iterations = 20;
agentPos = zeros(3,n,iterations+1);
obstPos = zeros(3,iterations+1);

agentPos(:,:,1) = x0_set;
obstPos(:,1) = xObst(:,1);
for i = 1:iterations
    fprintf('Simulation iteration %d\n',i);
    % find next optimal input
    uopt = FindOptimalInput(x0_set, N, ts, target, xObst, threshold);
    
    % find next position of agent given the optimal input
    nextState = SingleIntegrator(x0_set,uopt,ts);
    x0_set = nextState(:,2,:);
    
    % format x0_set 
    for j = 1:n
        x0_set(:,j) = nextState(:,end,j);
    end
    
    % store for graphing
    agentPos(:,:,i+1) = x0_set;
    obstPos(:,i+1) = xObst(:,1); % quick fix to store obst location

end

% plot simulations results
PlotSetBasedSim(agentPos, obstPos, threshold, target);



