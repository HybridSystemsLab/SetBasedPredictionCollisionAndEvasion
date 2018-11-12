% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots max distance to target and min distance to projectile throughout
% the simulation

function PlotSimDistance(agentPos, obst, threshold, target)

    
    
    
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % mO - coordinate (usually size 3)
    % nO - time step, equal to iterations in simulation
    [mO,nO] = size(obst);
    
    
    ObstDist = zeros(nA,pA);
    
    figure(5)
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
    
    FS = 12;
    h = gcf;
    set(h,'Units','inches','Position',[2 2 3.4 2])
    xlabel('Time [s]','FontName','Times','FontSize',FS)
    ylabel('Distance [m]','FontName','Times','FontSize',FS)
    set(gca,'FontName','Times','FontSize',FS)
    pos = h.PaperPosition;
    h.PaperPositionMode = 'auto';
    set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    print(h,'filename','-dpdf','-r0')
    
    %title('Minimum Distance Between Vehicle and Obstacle');

    axis([0 nA-1 0 2.5]);
    figure(6)
    subplot(2,1,1)
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
        
        if(j==16)
            %plot(0:1:(nA-1),threshold*ones(1,(nA)),'r');
            
            h = gcf;
            set(h,'Units','inches','Position',[2 2 3.4 2])
            xlabel('Time [s]','FontName','Times','FontSize',FS)
            ylabel('Distance [m]','FontName','Times','FontSize',FS)
            set(gca,'FontName','Times','FontSize',FS)
            pos = h.PaperPosition;
            h.PaperPositionMode = 'auto';
            set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
            %print(h,'filename','-dpdf','-r0')
            subplot(2,1,2)
            hold on
        end
    end  

    h = gcf;
    set(h,'Units','inches','Position',[2 2 3.4 2])
    xlabel('Time [s]','FontName','Times','FontSize',FS)
    ylabel('Distance [m]','FontName','Times','FontSize',FS)
    set(gca,'FontName','Times','FontSize',FS)
    pos = h.PaperPosition;
    h.PaperPositionMode = 'auto';
    set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    %print(h,'filename','-dpdf','-r0')
    
end