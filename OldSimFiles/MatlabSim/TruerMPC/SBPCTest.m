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

QR = 0.01;
PR = 0.01;
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

N = 3;
K = 4;

% algorithm visualization
% N = 15;
% K = 9;

TIMESTEP = 0.05; % sec
VELOCITY = logspace(0,log10(2),2); % m/s
sigma = 0.1;


SBPC(state,target,sigma,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY);




%SimulationSBPC(state,target,sigma,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY);



