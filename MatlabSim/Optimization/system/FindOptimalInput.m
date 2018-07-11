% function u0 = FindOptimalInput(x0, N, ts, target)
%
% uses fmincon to minimize cost function given system dynamics

function u0 = FindOptimalInput(x0, N, ts, target)
 
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % set lower and upper bounds on inputs to integrator
    lb = -1*ones(2,N);
    ub = ones(2,N);
    
    u_init = ones(2,N);

    % solve optimization
    uopt = fmincon(@(u) Cost(x0,u,ts,target),u_init,A,b,Aeq,beq,lb,ub);

    % return first input
	u0 = uopt(:,1); 
end
