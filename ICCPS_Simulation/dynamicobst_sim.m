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
speed = 1.48;
steer = pi/6;

% terminal cost weight
terminalWeight = 1e8;

% prediction horizon
N = 8;    

% control horizon
M = 2;

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE ALSO
ts = 0.05;  

% number of iterations to run
iterations = 25;

% target coordinates
target = [0;0];

% set box length
box_len = L/2 +0.01;%0.15;
% set box width
box_width= W/2 +0.01;% 0.1;
% set obstacle box length
box_len_obst =0.2;
% set obstacle box width
box_width_obst= 0.2;


% threshold distance value for obstacle
threshold = 0.05;    

% length of prediction simulation for projectile
simTime = N*ts;


% paper dynamic obst
xinit = [1.5 -1.1  -1.7   1.7;
         1.5  1.8  -1.8  -1.6;
         pi  -pi/6 pi/2 (3*pi)/4];
     
pinit = [1.1  0.0  -0.5   0.5;
         0.9  0.4  -0.1  -1.3;
         pi  pi/2  -pi/2 (pi)/4];
     
uObst = ones(2,N,4);
uObst(:,:,1) = [speed/3;-steer/2].*uObst(:,:,1);
uObst(:,:,2) = [speed/4;-steer].*uObst(:,:,2);
uObst(:,:,3) = [speed/3;steer].*uObst(:,:,3);
uObst(:,:,4) = [speed/4;0].*uObst(:,:,4);



% position arrays for plotting
predictions = zeros(5,N+1,iterations,4);
agentPos = zeros(5,iterations+1,4);
obstPos = zeros(5,iterations+1,4);

for j = 1:4
    
    % create polytope for obstacle and vehicle
    p0 = [pinit(1,j)-box_len_obst; pinit(1,j)+box_len_obst; ...
          pinit(2,j)-box_width_obst; pinit(2,j)+box_width_obst; pinit(3,j)];

    xObst = Dubin(p0,uObst(:,:,j),ts,L);

    x0 = [xinit(1,j)-box_len; xinit(1,j)+box_len; ...
          xinit(2,j)-box_width; xinit(2,j)+box_width; xinit(3,j)];


    %% Simulation core
    
    % size of vehicle and obstacle arays
    [m,n] = size(x0);
    [mO,nO] = size(xObst);


    % initial conditions
    agentPos(:,1,j) = x0;
    obstPos(:,1,j) = xObst(:,1);

    uGuess = zeros(2,N);
    uGuess(1,:) = (0)*ones(1,N);

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
        xObst = Dubin(p0,uObst(:,:,j),ts,L);


        % store for graphing

        % (coord,point,time)
        agentPos(:,i+1,j) = transpose(x0);

        % (coord,time,obst)
        obstPos(:,i+1,j) = xObst(:,1);
    
        % update input guess for next optimization
        uGuess = zeros(2,N);
        uGuess(1,:) = 0*ones(1,N);
        uGuess(:,1:N-M+1) = uopt(:,M:N);
    end

end

% plot simulation results
PlotSetBasedSim_DO(agentPos, obstPos, threshold, target, predictions);
PlotSimDistance_DO(agentPos, obstPos, threshold, target);






