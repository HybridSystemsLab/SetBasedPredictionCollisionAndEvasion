%
% simulation for set based predictive control
%

clc;clear all;close all


% Early collision examples
%{
p_0 = [1.0, 0.5, 0.5, -1.0, 0.0, 1.5];
x_0 = [0.0, 0.0, 0.0];
target = [-2.0, -2.0, 0.0];

state = [p_0, x_0];
SetBasedPredictiveControl(state, target)
%}