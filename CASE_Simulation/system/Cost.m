% c = Cost(x0_set, u, ts, target)
%
% custom cost function - sum of distance from each vertex to target squared

function c = Cost(x0_set, u, ts, target, L, terminalWeight)

    % predict system state with set based dynamics
    x_set = Dubin(x0_set,u,ts,L);
    
    % calculate cost of prediction for set based dynamics
    % - iterate through each vertex at each time step and calculate the 
    % distance between that point and the target
    
    [m,n] = size(x_set);
    
    c = 0;
    % stage cost
    for i = 1:n-1
        c = c + norm([x_set(1,i);x_set(3,i)]-target)^2 ...
        + norm([x_set(1,i);x_set(4,i)]-target)^2 ...
        + norm([x_set(2,i);x_set(3,i)]-target)^2 ...
        + norm([x_set(2,i);x_set(4,i)]-target)^2;
    end
    
    % terminal cost
    c = c + terminalWeight*norm([x_set(1,n);x_set(3,n)]-target)^2 ...
    + terminalWeight*norm([x_set(1,n);x_set(4,n)]-target)^2 ...
    + terminalWeight*norm([x_set(2,n);x_set(3,n)]-target)^2 ...
    + terminalWeight*norm([x_set(2,n);x_set(4,n)]-target)^2;
end

