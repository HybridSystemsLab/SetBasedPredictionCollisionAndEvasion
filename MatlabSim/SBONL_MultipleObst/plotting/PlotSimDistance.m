% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance(agentPos, obst, threshold, target)

    figure()
    hold on
    
    
    % mA - coordinate (usually size 3)
    % nA - number of points in each set
    % pA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    % pO - obstacle number
    [mO,nO,pO] = size(obst);
    
    
    ObstDist = zeros(pA,pO);
    
    
    for i = 1:pO
        for j = 1:pA
            
            % format points for MakeObj function
            curSet = zeros(mA,nA);
            for k = 1:nA
                curSet(:,k) = agentPos(:,k,j);
            end

            % min distance to projectile
            temp_xObst = [obst(:,j,i),obst(:,j,i)]; % polytope dist function has trouble with only one point
            ObstDist(j,i) = PolytopeMinDist(curSet,temp_xObst);

        end
        plot(0:1:(pA-1),transpose(ObstDist(:,i)));
        plot(0:1:(pA-1),threshold*ones(1,(pA)));
    end
    
    title('Minimum distance between vehicle polytope and obstacle polytope');
    xlabel('time');
    ylabel('distance');
    
    figure()
    hold on;
    dist = zeros(pA,1);
    
    for i = 1:pA
        for j = 1:nA

            dist(i) = dist(i) + norm(agentPos(:,j,i)-target)^2;

        end
        
    end
    
    plot(0:1:(pA-1),dist);
    title('Minimum distance between vehicle polytope and target polytope');
    xlabel('time');
    ylabel('distance');
    

    
    
    
    


end