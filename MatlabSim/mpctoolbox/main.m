
% Declare vertice sets
X1 = [1.0 1.0 1.0 1.5;
      1.0 1.0 1.0 1.0;
      1.0 1.0 1.0 1.0];
  
X2 = [2.5 2.0 2.0 2.0 2.0 1.5;
      2.5 2.0 2.0 2.0 2.0 1.0;
      2.5 2.0 2.0 2.0 2.0 1.0];

% Evaluate min distance
minDist = PolytopeMinDist(X2,X1);

fprintf('Minimization Complete - \n\n');
fprintf('Minimum distance from X1 to X2 is %0.2f\n',minDist);