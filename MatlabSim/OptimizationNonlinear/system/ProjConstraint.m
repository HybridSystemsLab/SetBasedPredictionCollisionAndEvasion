% [c,ceq] = ObstConstraint(x0, u, ts, xObst, threshold)
%
% defines the non linear constraint - maintain a distance above threshold
% from the obstacle position

function [c,ceq] = ObstConstraint(x0, u, ts, xObst, threshold)

    % predict agent
    x = SingleIntegrator(x0, u, ts);
    
    % calculate distance between agent and obstacle
    [m,n] = size(x);
    ObstDist = zeros(1,n);
    for i = 1:n
        ObstDist(i) = norm(x(:,i)-xObst(:,i));
    end
    
	% define constraints
    c = -ObstDist+threshold;
    ceq = [];
    
end