% c = Cost(x0_set, u, ts, target)
%
% custom cost function - sum of distance from each vertex to target squared

function c = Cost(x0_set, u, ts, target,L)

    % predict system state with set based dynamics
    x_set = Dubin(x0_set,u,ts,L);

    % calculate cost of prediction for set based dynamics
    % - iterate through each vertex at each time step and calculate the 
    % distance between that point and the target
    
    [m,n] = size(x_set);
    
    c = 0;
    
    for i = 1:n
        c = c + norm([x_set(1,i);x_set(3,i);0]-target)^2 ...
        + norm([x_set(1,i);x_set(4,i);0]-target)^2 ...
        + norm([x_set(2,i);x_set(3,i);0]-target)^2 ...
        + norm([x_set(2,i);x_set(4,i);0]-target)^2;
    end
        
end

