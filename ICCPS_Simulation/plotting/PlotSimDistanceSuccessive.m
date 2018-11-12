% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance(agentPos, obst, threshold, target)

    figure(1)
    hold on
    
    
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    % pO - obstacle number
    [mO,nO,pO] = size(obst);
    
    
    ObstDist = zeros(nA,pO);
    
    
    for i = 1:pO
         
    
    % create objects/convex hulls for each set at each time step
        for j = 1:nA

            % format points for MakeObj function
            curSet = zeros(3,4);

            curSet(:,1) = [agentPos(1,j);agentPos(3,j);0];
            curSet(:,2) = [agentPos(1,j);agentPos(4,j);0];
            curSet(:,3) = [agentPos(2,j);agentPos(3,j);0];
            curSet(:,4) = [agentPos(2,j);agentPos(4,j);0];

            % min distance to projectile
            temp_xObst = [obst(:,j,i),obst(:,j,i)]; % polytope dist function has trouble with only one point
            polyOptions = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set');
            ObstDist(j,i) = PolytopeMinDist(curSet,temp_xObst,polyOptions);

        end
        plot(0:1:(nA-1),transpose(ObstDist(:,i)));
        plot(0:1:(nA-1),threshold*ones(1,(nA)));
    end
    
    title('Minimum distance between vehicle polytope and obstacle polytope');
    xlabel('time');
    ylabel('distance');
    axis([0 nA-1 0 2]);
    drawnow;
    
    figure(2)
    hold on;
    dist = zeros(nA,1);
    
    for i = 1:nA
        

        dist(i) = dist(i) + norm([agentPos(1,i);agentPos(3,i);0]-target)^2 ...
        + norm([agentPos(1,i);agentPos(4,i);0]-target)^2 ...
        + norm([agentPos(2,i);agentPos(3,i);0]-target)^2 ...
        + norm([agentPos(2,i);agentPos(4,i);0]-target)^2;

        
        
    end
    
    plot(0:1:(nA-1),dist);
    title('Minimum distance between vehicle polytope and target polytope');
    xlabel('time');
    ylabel('distance');
    drawnow;
    

    
    
    
    


end