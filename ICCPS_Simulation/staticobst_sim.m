close all;
clc;
clear;



%% Simulations parameters

% Set EXP to 1 for experiments - approx polytope distance function
% set EXP to 0 for simulations - fmincon polytope distance function
EXP = 1;

% vehicle length
L = 0.451;
%Vehicle Width
W = 0.3306;

% input bounds
speed = 0.78;
steer = pi/6;

% terminal cost weight
terminalWeight = 1;

% prediction horizon
N = 8;    

% control horizon
M = 2;

% sampling time
ts = 0.05;  

% number of iterations to run
iterations = 20;

% target coordinates
target = [-0.50172; 0.059935];

% set box length
box_len = L/2;% +0.01;%0.15;
% set box width
box_width= W/2;% +0.01;% 0.1;
% set obstacle box length
box_len_obst =0.05;
% set obstacle box width
box_width_obst= 0.05;


xinit = [0.565 1.0496 -2.0999;
         0.67101 0.757591 -3.0443;
         0.7134 -0.71302 2.3485;
         0.15151 1.01 -1.7065;
         0.51736 -0.20566 3.6136;
         -1.1639 1.1533 -0.69361;
         -1.2713 0.578 -0.3821;
         -1.2645 -0.28164 0.16576;
         -1.1064 -0.81979 0.4722];

pinit = [0.02473; 0.083428; 0];


% configure obstacle set
p0 = [pinit(1)-box_len_obst; pinit(1)+box_len_obst; ...
      pinit(2)-box_width_obst; pinit(2)+box_width_obst; pinit(3)];
uObst = [0;steer].*ones(2,N);
xObst = Dubin(p0,uObst,ts,L);


% threshold distance value for obstacle
threshold = 0.05;          


%% Simulation core

% length of prediction simulation for projectile
simTime = N*ts;

% size of vehicle and obstacle arays
[m,n] = size(xinit);
%m = 1
[mO,nO] = size(xObst);

% position arrays for plotting
predictions = zeros(5,N+1,iterations,m);
agentPos = zeros(5,iterations+1,m);
obstPos = zeros(5,iterations+1);
u = zeros(2,iterations*M);


for j = 1:m
    
    % box
    x0 = [xinit(j,1)-box_len; xinit(j,1)+box_len; ...
          xinit(j,2)-box_width; xinit(j,2)+box_width; xinit(j,3)];


    % initial conditions
    agentPos(:,1,j) = x0;
    obstPos(:,1) = xObst(:,1);

    uGuess = zeros(2,N);
    uGuess(1,:) = zeros(1,N); 

    for i = 1:iterations
        fprintf('Simulation iteration %d%% complete\n',ceil((i/iterations)*100));

        % find next optimal input
        uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer, terminalWeight, uGuess,EXP);

        % find next position of agent given the optimal input
        nextState = Dubin(x0,uopt,ts,L);
        
        predictions(:,:,i,j) = nextState;
        x0 = nextState(:,M+1);

        % next prediction for projectile
        p0 = xObst(:,M+1);
        xObst = Dubin(p0,uObst,ts,L);


        % store for graphing
        % (coord,point,time)
        agentPos(:,i+1,j) = transpose(x0);
        % (coord,time,obst)
        obstPos(:,i+1) = xObst(:,1);
    
        % update initial guess for next optimization
        uGuess = zeros(2,N);
        uGuess(1,:) = zeros(1,N);
        uGuess(:,1:N-M+1) = uopt(:,M:N);

    end

end

% plot simulations results
PlotSimDistance_SO(agentPos, obstPos, threshold, target);
PlotSetBasedSim_SO(agentPos, obstPos, threshold, target, predictions);





