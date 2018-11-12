% x = SingleIntegrator(x0_set, u, ts)
%
% set based dynamics of single integrator

function x = SingleIntegrator(x0_set, u, ts)

    [mP,nP] = size(x0_set);
    [mH,nH] = size(u);
    
    x = zeros(3,nH+1,nP);
    x(:,1,:) = x0_set;
    
    % apply integrator dynamics
    for j = 1:nP
        for i = 1:nH
            x(:,i+1,j) = x(:,i,j) + ts*u(:,i);
        end
    end
end