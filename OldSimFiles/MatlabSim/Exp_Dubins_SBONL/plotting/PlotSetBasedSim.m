% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots associated sets with the simulation

function PlotSetBasedSim(agentPos, obst, threshold, target)
    
    figure()
    hold on
    
  
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA] = size(agentPos);
    
    % create objects/convex hulls for each set at each time step
    for i = 1:nA
        
        % format points for MakeObj function
        curSet = zeros(2,4);
        
        curSet(:,1) = [agentPos(1,i);agentPos(3,i)];
        curSet(:,2) = [agentPos(1,i);agentPos(4,i)];
        curSet(:,3) = [agentPos(2,i);agentPos(3,i)];
        curSet(:,4) = [agentPos(2,i);agentPos(4,i)];

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
    
   
    
    scatter(target(1), target(2), '*')
    
    xlabel('x axis')
    ylabel('y axis')
    axis([-1 1 -1 1])
    grid on
    
end