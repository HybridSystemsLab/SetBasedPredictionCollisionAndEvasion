

close all;
clc;
clear;


%% Description
%
%
% This file uses a set based predictive optimization frame work to return
% the optimal input for a system that uses Dubins model.

%% experiment
% set exp to one in order to use the fast algorithm
EXP = 1;


%% setup communication channel as well as the motion capture system


%set DEBUG to 1 if you like to do debuging;
DEBUG =0;

if DEBUG==0
    %delete any related port that is com6 from matlab workstation
    delete(instrfind({'Port'},{'COM6'}))

    % Setup for motive....no need to change this
    dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
    assemblyInfo = NET.addAssembly(dllPath);
    Client = NatNetML.NatNetClientML(0);
    HostIP = char('127.0.0.1');
    Client.Initialize(HostIP, HostIP);

    % Initialize Arduino port
    ppmValues = [1500,1500,1500,1500,1500,1500];
    [s,flag] = initSerial_copy();     % initSerial('COM3', 6);
    transmitSerial(s, ppmValues);
    theta =0;thrust=0;
end
%% Optimization parameters
% vehicle length
L = 0.451;
%Vehicle Width
W= 0.3306;

% input bounds
speed = 0.48;
steer = pi/6;

% prediction horizon
N = 8; 

% Control horizon
M=2;

% terminal cost
terminalWeight = 1e8;

% sampling time - REMEMBER TO CHANGE IN SIMULINK FILE IF YOU CHANGE HERE
ts = 0.05;  

% threshold distance value for obstacle
threshold = 0.05;      

% length of prediction simulation for projectile
simTime = N*ts;

% set box length
box_len = L/2 +0.01;%0.15;

% set box width
box_width= W/2 +0.01;% 0.1;

% set obstacle box length
box_len_obst =0.02;

% set obstacle box width
box_width_obst= 0.02;

%simulation time
sim_time =4.2;



%compute MPC offline? set to 1 means optimization is run offline
compute_offline = 1;

%motive regid body
Vehicle_ID =1;
Target_ID = 2;
Obstacle_ID =3;


%% Optimization

% This is where the loops would start, basic outline is as follows
%
% 1. obtain vehicle state
% 2. obtain projectile state
% 3. predict projectile
% 4. find optimal input with FindOptimalInput() function
% 5. apply optimal input
% 6. repeat 1-5

% OBTAIN VEHICLE STATE - (x1, x2, y1, y2, theta)

%set initial condition
x0 = [0.0;0.05;0.0;0.05;(5*pi)/4];

if DEBUG==0
    xinit =zeros(1,3);
    [xinit(1),xinit(2),xinit(3)]= GetVehiclePose(Client, Vehicle_ID)
    x0 = [xinit(1)-box_len; xinit(1)+box_len; xinit(2)-box_width; xinit(2)+box_width; xinit(3)];
   
end

%x0 = [0.0;0.05;0.0;0.05;(5*pi)/4];

%% OBTAIN PROJECTILE STATE - (x, y, z, xdot, ydot, zdot)

% %use this for predicting obstacle dynamics
% p0 = [-0.4;-0.2;0.2;1.5;0.4;0.0];
% 
% % PREDICT PROJETILE STATE
% projPos = ProjectilePredict(p0,simTime);
% xObst = transpose(projPos);
 
% get obstacle information from motion capture system and assign in
% obstacle struct

if DEBUG==0
    
    obstacle_info = zeros(1,3);
    [obstacle_info(1), obstacle_info(2), obstacle_info(3)]= GetObjPose(Client, Obstacle_ID)
    
    % box for obstacle
    p0=  [obstacle_info(1)-box_len_obst; obstacle_info(1)+box_len_obst; obstacle_info(2)-box_width_obst; obstacle_info(2)+box_width_obst; (pi)/1];
    	
    uObst = [0;steer].*ones(2,N);		% input (zero speed, any steer)
    xObst = Dubin(p0,uObst,ts,L);		% predict
    
%     xObst = ones(3,N+1,1);
%     xObst(1,:)= obstacle_info(1)*xObst(1,:);
%     xObst(2,:)= obstacle_info(2)*xObst(2,:);
%     xObst(3,:)= obstacle_info(3)*xObst(3,:);
    
    % target coordinates
    target = zeros(1,3);
    [target(1), target(2), target(3)] =  GetObjPose(Client, Target_ID)
end



%xObst =  transpose(xObst);
%%
T = 0:ts:sim_time;

vehicle_info = zeros(1,3);
pose = [xinit(1) xinit(2)];
input =[0 0];

%set rudder angel for 0 degree for initial start
rudder_cmd=1500; 

%save predicted vehicle dynamics
x_pred= zeros(5,N+1,length(T));

uGuess = zeros(2,N);
uGuess(1,:) = (speed/2)*ones(1,N);

for k = 1:length(T)
    %tic
    
    if compute_offline ==1
        ppmValues = [1500,rudder_cmd,1500,1500,1500,1500];
        flag = transmitSerial(s, ppmValues); 
    end
    
    uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer, terminalWeight, uGuess, EXP);
    %toc
    
    %get predicted vehicle dynamics
   % x_pred_box=  [x0(1)-box_len_obst; x0(1)+box_len_obst; x0(2)-box_width_obst; x0(2)+box_width_obst; x0(3)];
    x_pred(:,:,k)=  Dubin(x0,uopt,ts,L);
    
    % THIS IS THE NEXT OPTIMAL INPUT
    uopt(:,1);%

    %put vehicle model here to calculate the next x after input u
   % x0 = 
    if DEBUG==0
        for i =1:M
            thrust=3.28 * uopt(1,i); %3.28 is conversion factor from m/s to ft/s

            theta = radtodeg(uopt(2,i));

            rudder_cmd = Torudder(theta);
            thrust_cmd = ToThrust(thrust);
            ppmValues = [thrust_cmd,rudder_cmd,1500,1500,1500,1500];
            flag = transmitSerial(s, ppmValues);                       % send current signal

           [vehicle_info(1),vehicle_info(2),vehicle_info(3)]=  GetVehiclePose(Client, Vehicle_ID);
            
            pose  = vertcat(pose, [vehicle_info(1) vehicle_info(2)]);
            input = vertcat(input,uopt(:,i)');

            x0 = [vehicle_info(1)-box_len; vehicle_info(1)+box_len; vehicle_info(2)-box_width; vehicle_info(2)+box_width; vehicle_info(3)];
            pause(ts);
            
        end
        

        uGuess= zeros(2,N);
        uGuess(1,:) = speed/2*ones(1,N);%speedBound*ones(1,N);
        uGuess(:,1:N-M+1) =uopt(:,M:N);
   end
end

%% %% PLOT Result
ppmValues = [1500,1500,1500,1500,1500,1500];
transmitSerial(s, ppmValues);                       % send current signal


figure(1)

for i= 1:length(pose)
    time_stamp(i) = (i-1)*ts;
end

subplot(5,1,1)
plot(time_stamp,pose(:,1));
xlabel('t')
ylabel('x')
hold on
grid on


subplot(5,1,2)
plot(time_stamp,pose(:,1));
xlabel('t')
ylabel('y')
hold on
grid on


subplot(5,1,3)
plot(time_stamp,180/pi*input(:,2));
xlabel('t')
ylabel('steering')
hold on
grid on

subplot(5,1,4)
plot(time_stamp,input(:,1));
xlabel('t')
ylabel('speed')
hold on
grid on

subplot(5,1,5)
plot(pose(:,1),pose(:,2));
xlabel('x')
ylabel('y')
hold on
grid on

%plot vehicle maneuver and prediction
f= plotManeuver(xinit,obstacle_info,target, pose,x_pred,...
    L,W,2*box_width_obst,2*box_len_obst,threshold);
hold on



%close motive client
Client.Uninitialize();

%%

% %% FIND OPTIMAL INPUT
% tic
% uopt = FindOptimalInput(x0, N, ts, target, xObst, threshold, L, speed, steer);
% toc
% 
% % THIS IS THE NEXT OPTIMAL INPUT
% uopt(:,1)
% 
% agentPos = Dubin(x0, uopt, ts,L);
% PlotOptimalPredicted(agentPos, xObst, threshold, target)

    

