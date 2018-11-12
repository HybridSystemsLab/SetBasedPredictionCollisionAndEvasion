% function u0 = FindOptimalInput(x0, N, ts, target)
%
% uses fmincon to minimize cost function given system dynamics and
% nonlinear constraints

function u0 = FindOptimalInput(x0, N, ts, target, xObst, threshold)
 
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % set lower and upper bounds on inputs to integrator
    lb = -1*ones(3,N);
    ub = ones(3,N);
    
    uInit = ones(3,N);

    % solve optimization
    uopt = fmincon(@(u) Cost(x0,u,ts,target),uInit,A,b,Aeq,beq,lb,ub, @(u) ObstConstraint(x0,u,ts,xObst,threshold));

    % return first input
	u0 = uopt(:,1); 
end
