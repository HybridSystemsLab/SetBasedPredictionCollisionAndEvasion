% function u0 = FindOptimalInput(x0, N, ts, target)
%
% uses fmincon to minimize cost function given system dynamics and
% nonlinear constraints, returns optimal input sequence

function u0 = FindOptimalInput(x0_set, N, ts, target, xObst, threshold)
 
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % set lower and upper bounds on inputs to integrator
    bound = 0.1;
    lb = -bound*ones(3,N);
    ub = bound*ones(3,N);
    
    uInit = zeros(3,N);

    % solve optimization
    options = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set','MaxFunEvals',1000,'ConstraintTolerance',1e-04);
    
    uopt = fmincon(@(u) Cost(x0_set,u,ts,target),uInit,A,b,Aeq,beq,lb,ub, @(u) ObstConstraint(x0_set,u,ts,xObst,threshold),options);

    % return optimal input sequence
    u0 = uopt; 
end
