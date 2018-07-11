

function [c,ceq] = ProjConstraint(x0, u, ts, xProj, threshold)

    x = SingleIntegrator(x0, u, ts);
    
    [m,n] = size(x);
    
    projDist = zeros(1,n);
    for i = 1:n
        projDist(i) = norm(x(:,i)-xProj(:,i));
    end
    
        
    c = projDist-threshold;
    ceq = [];
    

end