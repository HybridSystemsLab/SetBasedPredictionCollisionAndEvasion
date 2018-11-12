% [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold)
%
% defines the non linear constraint - agent polytope to maintain
% a distance from the obstacle position above threshold 

function [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold)

    % predict agent with set based dynamics
    x_set = SingleIntegrator(x0_set, u, ts);
    
    % find distance between agent polytope and obstacle
    [m,n,p] = size(x_set);
    ObstDist = zeros(1,n);
    for i = 1:n
        
        % format the set for polytope minimization for time step i  
        xPolytope = zeros(3,p);
        for j = 1:p
            xPolytope(:,j) = x_set(:,i,j);
        end
        
        % calculate distance between agent and obstacle
        % xPolytope is a 3xp matrix
        % xObst(:,i) is a 3x1 matrix
        ObstDist(i) = PolytopeMinDist(xPolytope,xObst(:,1:2));
    end
    
	% define constraints
    c = -ObstDist+threshold;
    ceq = [];
    
end