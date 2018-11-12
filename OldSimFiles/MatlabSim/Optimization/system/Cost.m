% function c = Cost(x0, u, ts, target)
%
% custom cost function - distance to target squared

function c = Cost(x0, u, ts, target)
    
    % predict system state
    x = SingleIntegrator(x0,u,ts);
   
    % calculate cost of prediction
    [m,n] = size(x);
    
    c = norm(x-target*ones(1,n))^2  
     
end

