close all;
clc;
clear;

%% Finds minimum distance between polytopes X1 and X2

% Declare vertice sets
X1 = [0.0 0.0 1.0 1.0;
      0.0 1.0 0.0 1.0];
  
X2 = [2.0 3.0 2.0 3.0 1.1;
      0.0 0.0 1.0 1.0 1.0];

% Evaluate min distance
minDist = PolytopeMinDist(X2,X1);

fprintf('Minimization Complete - \n\n');
fprintf('Minimum distance from X1 to X2 is %0.2f\n',minDist);