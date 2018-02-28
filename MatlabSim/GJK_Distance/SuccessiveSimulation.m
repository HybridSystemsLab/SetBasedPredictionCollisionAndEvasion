%
% simulation for set based predictive control
%

clc;clear all;close all

tic

% Paper example - multiple simulations distance graphs
p_0 = [1.3, 0.4, 0.5, -1.15, 0.59, 1.9;
       2.0, 3.0, 0.3, -2.3, -3.5, 3.3;
       1.7, 0.26, 0.5, -3.22, 1.5, 1.9;
       1.3, 0.3, 0.5, -1.3, 1.1, 2.2;
       1.1, 0.56, 0.5, -0.62, 0.9, 1.9;
       1.2, 0.2, 0.5, -0.9, 1.2, 2.0;
       1.33, 0.37, 0.5, -0.55, 0.59, 3.9;
       1.5, 3.3, 0.3, -1.1, -2.8, 2.9];
   
x_0 = [-0.35, -0.04, 0.5;
       -0.56, -0.39, 0.5;
       0.18, 0.21, 0.5;
       0.45, 0.39, 0.5;
       0.6, 0.57, 0.5;
       0.49, 0.1, 0.5;
       0.75, -0.92, 0.5;
       2.04, 1.81, 0.5];
   
target = [1.0, 1.0, 0.5];
initialStates = [x_0, p_0];

% set based prediction constants
% QR - quad radius
% PR - projectile radius
% TDIS - theta discretization param (n-1)
% PDIS - phi discretixation param (n-1)

QR = 0.1;
PR = 0.1;
TDIS = 5;
PDIS = 5;
sigma = 0.1;

% mpc contants
% N - prediction horizon
% K - discretization parameter

N = 5;
K = 9;

% fixed time step for prediction
TIMESTEP = 0.1; % sec

% velocity input constraint
R = 5;
Rmax = 1; % m/s
VELOCITY = logspace(0,log10(Rmax+1),R)-1; % m/s

% length of simulation = 
iterations = 20;

[m,n] = size(initialStates);
figure(1)
for i = 1:m
    
    newState(1,:) = initialStates(i,:);
    state = initialStates(i,:);

    
    for j = 1:iterations

        newState(j+1,:) = SimulationSBPC(state,target,sigma,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY);

        state = newState(j+1,:);

        fprintf('Simulation %d is %d%% complete\n',i,double(j/iterations)*100);
    end

    [x y z] = sphere(32);

    % plot 3d cartesian coordinates of quad and projectile 
    % throughout the whole simulation
    
    figure()
    scatter3(initialStates(i,1),initialStates(i,2),initialStates(i,3),'*','blue');
    hold on
    scatter3(initialStates(i,4),initialStates(i,5),initialStates(i,6),'*','blue');
    scatter3(target(1), target(2), target(3),'blue');
    scatter3(newState(:,1), newState(:,2), newState(:,3),'filled','green');
    scatter3(newState(:,4), newState(:,5) ,newState(:,6),'filled','red');

    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    grid on
    legend('Quadrotor I.C.','Projectile I.C.','Target','Quadrotor','Projectile','Location','northeast')
    

    for j = 1:iterations+1
        projDistance(j) = pdist([newState(j,1:3); newState(j,4:6)], 'euclidean');
        targDistance(j) = pdist([newState(j,1:3); target(1:3)], 'euclidean');
    end

    t = linspace(0,iterations*TIMESTEP,iterations+1);

    % distance plots to ensure the quadrotor satisfied constraints and
    % successfully optimized the trajectories
    figure(1)
    subplot(1,2,1);
    hold on
    plot(t, projDistance)
    
    subplot(1,2,2);
    hold on
    plot(t,targDistance)
    
    
    drawnow
end

figure(1)
subplot(1,2,1);
plot([0 iterations*TIMESTEP],[sigma sigma])
grid on
xlabel('Time [s]');
ylabel('Distance [m]');
title('Distance Between Quadrotor and Projectile');
ylim([0.0 2.0]);
xlim([0.0 iterations*TIMESTEP]);
    
subplot(1,2,2);
plot([0 iterations*TIMESTEP],[QR+0.001 QR+0.001])
grid on
xlabel('Time [s]');
ylabel('Distance [m]');
title('Distance Between Quadrotor and Target');
ylim([0.0 2.0]);
xlim([0.0 iterations*TIMESTEP]);

toc