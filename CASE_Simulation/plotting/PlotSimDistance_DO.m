% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance_DO(agentPos, obst, threshold, target)

    figure()
    hold on
    
    
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    [mO,nO,pO] = size(obst);
    
    
    ObstDist = zeros(nA,pA);
   
    
    % create objects/convex hulls for each set at each time step
    for j = 1:pA
        for i = 1:nA

            % format points for MakeObj function
            curSet = zeros(2,4);

            curSet(:,1) = [agentPos(1,i,j);agentPos(3,i,j)];
            curSet(:,2) = [agentPos(1,i,j);agentPos(4,i,j)];
            curSet(:,3) = [agentPos(2,i,j);agentPos(3,i,j)];
            curSet(:,4) = [agentPos(2,i,j);agentPos(4,i,j)];

            % format points for MakeObj function
            curObstSet = zeros(2,4);

            curObstSet(:,1) = [obst(1,i,j);obst(3,i,j)];
            curObstSet(:,2) = [obst(1,i,j);obst(4,i,j)];
            curObstSet(:,3) = [obst(2,i,j);obst(3,i,j)];
            curObstSet(:,4) = [obst(2,i,j);obst(4,i,j)];

            % min distance to projectile
            polyOptions = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set');
            %ObstDist(i,j) = PolytopeApproxDist(curSet,curObstSet);
            ObstDist(i,j) = PolytopeMinDist(curSet,curObstSet,polyOptions);

        end
        plot(0:1:(nA-1),transpose(ObstDist(:,j)));
 
    end

    plot([0 (nA-1)],[threshold threshold],'--r');


    FS = 50;
    h = gcf;
    set(gca,'FontSize',FS)
    axis([0 nA-1 0 1.5]);
    box on
    

end