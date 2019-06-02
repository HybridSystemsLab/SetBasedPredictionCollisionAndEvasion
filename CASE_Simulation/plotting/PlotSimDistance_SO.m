% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance_SO(agentPos, obst, threshold, target)

    
    
    
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    [mO,nO] = size(obst);
    
    
    L = abs(agentPos(1,1,1)-agentPos(2,1,1));
    W = abs(agentPos(3,1,1)-agentPos(4,1,1));
    
    minDist = 2*sqrt(L^2 + W^2);
    
    
    ObstDist = zeros(nA,pA);
    
    figure()
    hold on
    
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

            curObstSet(:,1) = [obst(1,i);obst(3,i)];
            curObstSet(:,2) = [obst(1,i);obst(4,i)];
            curObstSet(:,3) = [obst(2,i);obst(3,i)];
            curObstSet(:,4) = [obst(2,i);obst(4,i)];

            % min distance to projectile
            polyOptions = optimoptions('fmincon','Display','notify-detailed','algorithm','active-set');
            %ObstDist(i,j) = PolytopeMinDist(curSet,curObstSet,polyOptions);
            ObstDist(i,j) = PolytopeApproxDist(curSet,curObstSet);

        end
        %plot(0:1:(nA-1),transpose(ObstDist(:,j)));
        
        %if(j==16)
        %    plot(0:1:(nA-1),threshold*ones(1,(nA)),'r');
        %    subplot(2,1,2)
        %    hold on
        %end
    end
    plot([0 (nA-1)],[threshold threshold],'--r');
    
    
    [y,ind] = min(ObstDist);
    
    for i = 1:pA
        if(y(i) < 0.2)
            plot(0:1:(nA-1),transpose(ObstDist(:,i)));
        end
    end
    
    FS = 50;
    h = gcf;
    set(gca,'FontSize',FS)
    print(h,'DistToObst','-depsc','-r0')
    box on
    grid on
    

    axis([0 nA-1 0 1.5]);
    figure()
    hold on
    dist = zeros(nA,pA);
    for j = 1:pA
        for i = 1:nA
            dist(i,j) = dist(i,j) + sqrt(norm([agentPos(1,i,j);agentPos(3,i,j)]-target)^2) ...
            + sqrt(norm([agentPos(1,i,j);agentPos(4,i,j)]-target)^2) ...
            + sqrt(norm([agentPos(2,i,j);agentPos(3,i,j)]-target)^2) ...
            + sqrt(norm([agentPos(2,i,j);agentPos(4,i,j)]-target)^2);
        end
        plot(0:1:(nA-1),dist(:,j));
    end  
    plot([0 (nA-1)],[minDist minDist],'--r');

    
    
    h = gcf;
    set(gca,'FontSize',FS)
    print(h,'DistToTarg','-depsc','-r0')
    box on
    grid on
    
    
end