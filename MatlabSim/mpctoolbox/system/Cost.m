% function c = Cost(x0, u, ts, target)

function c = Cost(x0, u, ts, target)

    
    x = SingleIntegrator(x0,u,ts)
   
    [m,n] = size(x);
    
    c = 0;
    for i = 1:n
        c = c + norm(x(:,i)-target);
    end
    
    
end
