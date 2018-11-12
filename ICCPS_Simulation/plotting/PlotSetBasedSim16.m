% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots associated sets with the simulation

function PlotSetBasedSim(agentPos, obst, threshold, target)
    
    
    
    
  
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA,pA] = size(agentPos);
    
    % create objects/convex hulls for each set at each time step
    for j = 1:pA
        
        if(j<=16)
            if(j==6 || j==7 || j==9 || j==12 || j==13 || j==14 || j==15 || j==16)
                %subplot(2,2,1)
                figure(2)
            else
                %subplot(2,2,2)
                figure(1)
            end
        else
            if(j==22 || j==23 || j==25 || j==28 || j==29 || j==30 || j==31 || j==32)
                %subplot(2,2,3)
                figure(3)
            else
                %subplot(2,2,4)
                figure(4)
            end
        end
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

            curObstSet(:,1) = [obst(1,i);obst(3,i)];
            curObstSet(:,2) = [obst(1,i);obst(4,i)];
            curObstSet(:,3) = [obst(2,i);obst(3,i)];
            curObstSet(:,4) = [obst(2,i);obst(4,i)];
            % make the object
            MakeObj(curObstSet, 'r');
        end

        
        
        figure(1)
        FS = 24;
        h = gcf;
        xlabel('q^1 [m]','FontName','Times','FontSize',FS)
        ylabel('q^2 [m]','FontName','Times','FontSize',FS)
        set(gca,'FontName','Times','FontSize',FS)
        h.PaperPositionMode = 'auto';
        axis([-4 4 -4 4])
        grid on
        
        figure(2)
        
        h = gcf;
        xlabel('q^1 [m]','FontName','Times','FontSize',FS)
        ylabel('q^2 [m]','FontName','Times','FontSize',FS)
        set(gca,'FontName','Times','FontSize',FS)
        h.PaperPositionMode = 'auto';
        axis([-4 4 -4 4])
        grid on
        
        figure(3)

        h = gcf;
        xlabel('q^1 [m]','FontName','Times','FontSize',FS)
        ylabel('q^2 [m]','FontName','Times','FontSize',FS)
        set(gca,'FontName','Times','FontSize',FS)
        h.PaperPositionMode = 'auto';
        axis([-4 4 -4 4])
        grid on
        
        figure(4)
        
        h = gcf;
        xlabel('q^1 [m]','FontName','Times','FontSize',FS)
        ylabel('q^2 [m]','FontName','Times','FontSize',FS)
        set(gca,'FontName','Times','FontSize',FS)
        h.PaperPositionMode = 'auto';
        axis([-4 4 -4 4])
        grid on
        
    end    
    
    
    
end