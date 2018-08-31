% [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold)
%
% defines the non linear constraint - agent polytope to maintain
% a distance from the obstacle position above threshold 

function [c,ceq] = ObstConstraint(x0_set, u, ts, xObst, threshold,L,options)

    
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
            xPolytope = zeros(3,4);
            
            xPolytope(:,:) = [x_set(1,i), x_set(1,i), x_set(2,i), x_set(2,i);
                              x_set(3,i), x_set(4,i), x_set(3,i), x_set(4,i);
                              0, 0, 0, 0];

            
            % calculate distance between agent and obstacle
            % xPolytope is a 3xp matrix
            % xObst(:,i,j) is a 3x1 matrix
            temp_xObst = [xObst(:,i,j),xObst(:,i,j)]; % polytope dist function has trouble with only one point
            ObstDist(obstDistCount) = PolytopeMinDist(xPolytope,temp_xObst,options);
            obstDistCount = obstDistCount+1;
        end
    end
    
	% calculate constraints
    c = -ObstDist+threshold;
    ceq = [];
      
end