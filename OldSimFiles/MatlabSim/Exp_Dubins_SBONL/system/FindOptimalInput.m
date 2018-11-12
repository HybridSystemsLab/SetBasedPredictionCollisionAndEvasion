% function u0 = FindOptimalInput(x0, N, ts, target)
%
% uses fmincon to minimize cost function given system dynamics and
% nonlinear constraints, returns optimal input sequence

function uopt = FindOptimalInput(x0_set, N, ts, target, xObst, threshold, L, speedBound, steeringBound, terminalWeight, EXP)
 
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % set lower and upper bounds on inputs to dubins model
    lb(1,:) = zeros(1,N);               % lower bound is zero (speed)
    lb(2,:) = -steeringBound*ones(1,N);
    ub(1,:) = speedBound*ones(1,N);
    ub(2,:) = steeringBound*ones(1,N);
    
    
    % initial input guess
    uInit = zeros(2,N);
    uInit(1,:) = (0)*ones(1,N);
    

    % solve optimization
    options = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set','MaxFunEvals',1000,'ConstraintTolerance',1e-04);
    polyOptions = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set');
    
    
    [uopt ,fval,exitflag,output] = fmincon(@(u) Cost(x0_set,u,ts,target,L,terminalWeight),uInit,A,b,Aeq,beq,lb,ub, @(u) ObstConstraint(x0_set,u,ts,xObst,threshold,L,polyOptions,EXP),options);
    output
    
    % return optimal input sequence
    
end
