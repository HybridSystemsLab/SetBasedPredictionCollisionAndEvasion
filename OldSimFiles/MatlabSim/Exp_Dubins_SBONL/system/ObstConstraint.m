% [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold)
%
% defines the non linear constraint - agent polytope to maintain
% a distance from the obstacle position above threshold 

function [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold,L,options,EXP)

    
    % predict agent with set based dynamics
    % coords X time X set points
    
    x_set = Dubin(x0_set, u, ts,L);
    
    % find distance between agent polytope and obstacle
    [mA,nA] = size(x_set);
    [mO,nO,pO] = size(xObst);
    ObstDist = zeros(1,pO*nA);
    
    obstDistCount = 1;
    for i = 1:nA        % time
        for j = 1:pO    % number of obstacles
            
            % format the agents set for polytope minimization for time step i  
            xPolytope = zeros(2,4);
            xPolytope(:,:) = [x_set(1,i), x_set(1,i), x_set(2,i), x_set(2,i);
                              x_set(3,i), x_set(4,i), x_set(3,i), x_set(4,i)];

            
            oPolytope = zeros(2,4);
            oPolytope(:,:) = [xObst(1,i), xObst(1,i), xObst(2,i), xObst(2,i);
                              xObst(3,i), xObst(4,i), xObst(3,i), xObst(4,i)];
            
            if(EXP)
                ObstDist(obstDistCount) = PolytopeApproxDist(xPolytope,oPolytope);
            else
                ObstDist(obstDistCount) = PolytopeMinDist(xPolytope,oPolytope,options);
            end
            
            obstDistCount = obstDistCount+1;
        end
    end
    
	% calculate constraints
    c = -ObstDist+threshold;
    ceq = [];
      
end