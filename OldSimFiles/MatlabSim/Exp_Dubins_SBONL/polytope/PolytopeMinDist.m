% minDist = PolytopeMinDist(X1,X2)
%
% finds the minimum distance between two polytopes X1 and X2

function minDist = PolytopeMinDist(X1,X2,options)
    
    % declare constraints for fmincon
    lb = [];
    ub = [];
    
    % get sizes of vertices for polytopes
    [m1,n1] = size(X1);
    [m2,n2] = size(X2);
    
    if(m1 ~= m2)
        error('USER ERROR: Dimensions mistmatch');
    end
    
    n = n1+n2;
    
    A = [eye(n); -eye(n)];
    b = [ones(n,1); zeros(n,1)];
    
    Aeq = [ones(1,n1) zeros(1,n2);
           zeros(1,n1) ones(1,n2)];
	beq = [1;1];
    
    nonlcon = [];
    
    % create lambda vectors
    x0 = zeros(n,1);
    x0(1) = 1;
    x0(n1+1) = 1;
    
    % declare function to be minimized
    %fun = @(lambda)(norm((X1 * lambda(1:n1))-(X2 * lambda(n1+1:n)))^2);
    
    % evaluate fmincon
    %x = fmincon(@(lambda) PolytopeDist(X1,X2,lambda,n1,n2,n),x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    
    [x,fval,exitflag,output] = fmincon(@(lambda) PolytopeDist(X1,X2,lambda,n1,n2,n),x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
    %output
    
    % return min distance
    minDist = sqrt(PolytopeDist(X1,X2,x,n1,n2,n));
    
    
end

