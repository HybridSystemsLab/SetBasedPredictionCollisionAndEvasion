close all;
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
      
[m,n] = size(x0_set);


% projectile
%p0 = [-0.1;-0.1;0.15;0;0;0];
%simTime = N*ts;
%[projPos, projVel] = SimulationProjectilePredict(p0,simTime);
%xObst = transpose(projPos);
%xdotObst = transpose(projVel);


uopt = FindOptimalInput(x0_set, N, ts, target, xObst, threshold);

x_set = SingleIntegrator(x0_set, uopt, ts);
[m,n,p] = size(x_set);

ObstConstraint(x0_set, uopt, ts, xObst, threshold)

% reform
plotSet = zeros(3,p,N);
for i = 1:p
    for j = 1:n
        plotSet(:,i,j) = x_set(:,j,i);
    end
end

PlotSetBasedSim(plotSet, xObst, threshold, target)


