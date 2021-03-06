%
% simulation for set based predictive control
%

clc;clear all;close all

tic
% Early collision examples
% p_0 = [1.0, 0.5, 0.5, -1.2, -0.2, 1.5];
% x_0 = [0.0, 0.0, 0.0];
% target = [1.0, 1.0, 0.0];
% state = [x_0, p_0];

% Late collision example
% p_0 = [1.1, 0.56, 0.5, -0.62, 0.8, 1.9];
% x_0 = [0.68, 0.72, 0.5];
% target = [1.0, 1.0, 0.5];
% state = [x_0, p_0];

% Paper example - simulation visualation
p_0 = [1.3, 0.4, 0.5, -0.55, 0.59, 3.9];
x_0 = [0.35, 0.4, 0.5];
target = [1.0, 1.0, 0.5];
state = [x_0, p_0];

% Paper example - algo visualization
% p_0 = [2.0, 3.0, 0.3, -1.4, -3.2, 3.3];
% x_0 = [0.45, 0.39, 0.5];
% target = [1.0, 1.0, 0.5];
% state = [x_0, p_0];

%p_0 = [1.1, 0.56, 0.5, 0.3, 1.1, 2.2];
%x_0 = [0.45, 0.39, 0.5];
%target = [1.0, 1.0, 0.5];
%state = [x_0, p_0];

%x_0 = [0.9641, 0.8541, 0.5000];
%p_0 = [0.4811, 1.3583, -1.4983, -0.6178, 0.7964, -6.8530];
%target = [1.0, 1.0, 0.5];
%state = [x_0, p_0];

% set based prediction constants
% QR - quad radius
% PR - projectile radius
% TDIS - theta discretization param (n-1)
% PDIS - phi discretixation param (n-1)

QR = 0.1;
PR = 0.1;
TDIS = 5;
PDIS = 5;


% algorithm visualization
% QR = 0.05;
% PR = 0.05;
% TDIS = 9;
% PDIS = 9;


% mpc contants
% N - prediction horizon
% K - discretization parameter

N = 10;
K = 17;

% algorithm visualization
% N = 15;
% K = 9;

TIMESTEP = 0.05; % sec
VELOCITY = logspace(0,log10(2),10)-1; % m/s
sigma = 0.1;
%TIMESTEP = 0.1; % sec
%VELOCITY = logspace(0,log10(2),2)-1; % m/s

%SBPC(state,target,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY);



newState(1,:) = state;

iterations = 30;
for i = 1:iterations
    newState(i+1,:) = SimulationSBPC(state,target,sigma,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY);
    
    state = newState(i+1,:);

    fprintf('Simulation is %d%% complete\n', double(i/iterations)*100);
end

[x y z] = sphere(32);

% plot 3d cartesian coordinates of quad and projectile 
% throughout the whole simulation
scatter3(x_0(1),x_0(2),x_0(3),'*','blue');
hold on
scatter3(p_0(1),p_0(2),p_0(3),'*','blue');
scatter3(target(1), target(2), target(3),'blue');
scatter3(newState(:,1), newState(:,2), newState(:,3),'filled','green');
scatter3(newState(:,4), newState(:,5) ,newState(:,6),'filled','red');

xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on
legend('Quadrotor I.C.','Projectile I.C.','Target','Quadrotor','Projectile','Location','northeast')

%{
% plots final set for quadrotor
h = surfl(QR*x+newState(end,1),QR*y+newState(end,2),QR*z+newState(end,3)); 
set(h, 'FaceAlpha', 0.5)
shading interp
h = surfl(0.001*x+target(1),0.001*y+target(2),0.001*z+target(3)); 
set(h, 'FaceAlpha', 0.5)
shading interp

% Plots a transparent plane
threshold = 0.5;
% Obtain the limits of the axes
%zp = get(gca,'Zlim');
yp = [0 1.5];
xp = [0 1.5];
% Use the axes x and Y limits to find the co-ordinates for the patch
x1 = [ xp(1) xp(2) xp(2) xp(1)];
y1 = [ yp(1) yp(1) yp(2) yp(2)];z1 = ones(1,numel(y1))* threshold;
v = patch(x1,y1,z1, 'g');
set(v,'facealpha',0.4);
set(v,'edgealpha',0.1);
set(gcf,'renderer','opengl') ;
%}

for i = 1:iterations+1
    projDistance(i) = pdist([newState(i,1:3); newState(i,4:6)], 'euclidean');
    targDistance(i) = pdist([newState(i,1:3); target(1:3)], 'euclidean');
end

t = linspace(0,iterations*TIMESTEP,iterations+1);

% distance plots to ensure the quadrotor satisfied constraints and
% successfully optimized the trajectories
figure()
subplot(1,2,1);
plot(t, projDistance)
hold on
plot([0 iterations*TIMESTEP],[QR+PR QR+PR])
xlabel('Time [s]');
ylabel('Distance [m]');
title('Distance Between Quadrotor and Projectile');
grid on
ylim([0.0 2.0]);
xlim([0.0 iterations*TIMESTEP]);

subplot(1,2,2);
plot(t,targDistance)
hold on
plot([0 iterations*TIMESTEP],[QR+0.001 QR+0.001])
xlabel('Time [s]');
ylabel('Distance [m]');
title('Distance Between Quadrotor and Target');
grid on
ylim([0.0 2.0]);
xlim([0.0 iterations*TIMESTEP]);

toc
