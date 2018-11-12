% c = Cost(x0_set, u, ts, target)
%
% custom cost function - sum of distance from each vertex to target squared

function c = Cost(x0_set, u, ts, target)
    
    % predict system state with set based dynamics
    x_set = SingleIntegrator(x0_set,u,ts);
   
    % calculate cost of prediction for set based dynamics
    % - iterate through each vertex at each time step and calculate the 
    % distance between that point and the target
    
    [m,n,p] = size(x_set);
    
    c = 0;
    for i = 1:p
        for j = 1:n
            c = c + norm(x_set(:,j,i)-target)^2;
        end
    end
    
    %{
    c = 0;
    for i = 1:n
        polySet = zeros(3,p);
        for j = 1:p
            polySet(:,j) = x_set(:,i,j);
        end
        c = c + PolytopeMinDist(polySet, target);
    end
    %}
 
end

