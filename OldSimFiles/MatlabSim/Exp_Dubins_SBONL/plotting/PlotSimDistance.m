% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance(agentPos, obst, threshold, target)

    figure()
    hold on
    
    
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    [mO,nO] = size(obst);
    
    
    ObstDist = zeros(nA,1);
    
   
    
    % create objects/convex hulls for each set at each time step
    for i = 1:nA

        % format points for MakeObj function
        curSet = zeros(2,4);

        curSet(:,1) = [agentPos(1,i);agentPos(3,i)];
        curSet(:,2) = [agentPos(1,i);agentPos(4,i)];
        curSet(:,3) = [agentPos(2,i);agentPos(3,i)];
        curSet(:,4) = [agentPos(2,i);agentPos(4,i)];

        % format points for MakeObj function
        curObstSet = zeros(2,4);

        curObstSet(:,1) = [obst(1,i);obst(3,i)];
        curObstSet(:,2) = [obst(1,i);obst(4,i)];
        curObstSet(:,3) = [obst(2,i);obst(3,i)];
        curObstSet(:,4) = [obst(2,i);obst(4,i)];

        % min distance to projectile
        polyOptions = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set');
        ObstDist(i) = PolytopeMinDist(curSet,curObstSet,polyOptions);

    end
    plot(0:1:(nA-1),transpose(ObstDist));
    plot(0:1:(nA-1),threshold*ones(1,(nA)));
    
    
    title('Minimum distance between vehicle polytope and obstacle polytope');
    xlabel('time');
    ylabel('distance');
    axis([0 nA-1 0 2]);
    figure()
    hold on;
    dist = zeros(nA,1);
    
    for i = 1:nA
        

        dist(i) = dist(i) + norm([agentPos(1,i);agentPos(3,i)]-target)^2 ...
        + norm([agentPos(1,i);agentPos(4,i)]-target)^2 ...
        + norm([agentPos(2,i);agentPos(3,i)]-target)^2 ...
        + norm([agentPos(2,i);agentPos(4,i)]-target)^2;

        
        
    end
    
    plot(0:1:(nA-1),dist);
    title('Minimum distance between vehicle polytope and target polytope');
    xlabel('time');
    ylabel('distance');
    

    
    
    
    


end