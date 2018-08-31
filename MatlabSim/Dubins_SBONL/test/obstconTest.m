

N = 10;                      % prediction horizon
ts = 0.05;                  % sampling time
target = [-0.1;-0.1;-0.1];  % target coordinates
xObst = -0.5*ones(3,N+1);  % obstacle coordinate - stationary
threshold = 0.002;          % threshold distance value

x0_set = [0.0 0.02 0.02 0.0 0.0 0.02 0.02 0.0;
          0.0 0.0 0.02 0.02 0.0 0.0 0.02 0.02;
          0.0 0.0 0.0 0.0 0.02 0.02 0.02 0.02];
      
u = -0.5*ones(3,N);

[c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold)