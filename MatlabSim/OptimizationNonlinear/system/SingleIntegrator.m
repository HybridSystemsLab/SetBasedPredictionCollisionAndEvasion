% function x = SingleIntegrator(x0, u, ts)
%
% dynamics of single integrator

function x = SingleIntegrator(x0, u, ts)

    [m,n] = size(u);
    
    x = zeros(3,n+1);
    x(:,1) = x0;
    
    
    % apply integrator dynamics
    for i = 1:n
        x(:,i+1) = x(:,i) + ts*u(:,i);
    end
end