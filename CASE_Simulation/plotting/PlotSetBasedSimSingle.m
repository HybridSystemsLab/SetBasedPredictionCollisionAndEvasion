% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots associated sets with the simulation

function PlotSetBasedSim(agentPos, obst, threshold, target)
    
    
    
  
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % create objects/convex hulls for each set at each time step
    for j = 1:pA
        figure()
        hold on
        for i = 1:nA

            % format points for MakeObj function
            curSet = zeros(2,4);

            curSet(:,1) = [agentPos(1,i,j);agentPos(3,i,j)];
            curSet(:,2) = [agentPos(1,i,j);agentPos(4,i,j)];
            curSet(:,3) = [agentPos(2,i,j);agentPos(3,i,j)];
            curSet(:,4) = [agentPos(2,i,j);agentPos(4,i,j)];

            % make the object
            MakeObj(curSet, 'g');

            % format points for MakeObj function
            curObstSet = zeros(2,4);

            curObstSet(:,1) = [obst(1,i,j);obst(3,i,j)];
            curObstSet(:,2) = [obst(1,i,j);obst(4,i,j)];
            curObstSet(:,3) = [obst(2,i,j);obst(3,i,j)];
            curObstSet(:,4) = [obst(2,i,j);obst(4,i,j)];
            % make the object
            MakeObj(curObstSet, 'r');
        end
        
        FS = 16;

        h = gcf;
        %set(h,'Units','inches','Position',[2 2 3.4 2])
        xlabel('q^1 [m]','FontName','Times','FontSize',FS)
        ylabel('q^2 [m]','FontName','Times','FontSize',FS)
        set(gca,'FontName','Times','FontSize',FS)
        h.PaperPositionMode = 'auto';
        axis([-2 2 -2 2])
        grid on
    end
    
   
    
    %scatter(target(1), target(2), '*')
    
    
    
end