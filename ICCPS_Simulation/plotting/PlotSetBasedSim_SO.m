% PlotSetBasedSim(agentPos, obst, threshold)
%
% plots associated sets with the simulation

function PlotSetBasedSim_SO(agentPos, obst, threshold, target, predictions)
    
    
    
    figure()
    FS = 50;
    h = gcf;
    set(gca,'FontSize',FS)
    axis([-1.5 1.0 -1.5 1.5]);
    grid on
    box on
  
    % mA - coordinate 
    % nA - time step, equal to iterations in simulation
    % pA - numSim
    [mA,nA,pA] = size(agentPos);
    
    L = abs(agentPos(1,1,1)-agentPos(2,1,1));
    W = abs(agentPos(3,1,1)-agentPos(4,1,1));
    
    % 5 X N+1 X iterations X numSim
    [mP,nP,pP,qP] = size(predictions);

    
    cc = jet(pP);
    
    % sim num
    for i = 1:qP
        hold on
        % sim iteration
        for j = 1:pP   
            % prediction horizon
            for k = 1:nP
                rectangle('Position',[predictions(1,k,j,i) predictions(3,k,j,i) L W],...
                    'EdgeColor',cc(mod(j,pP)+1,:),'LineWidth',0.6);    
            end

            % format points for MakeObj function
            curObstSet = zeros(2,4);

            curObstSet(:,1) = [obst(1,i);obst(3,i)];
            curObstSet(:,2) = [obst(1,i);obst(4,i)];
            curObstSet(:,3) = [obst(2,i);obst(3,i)];
            curObstSet(:,4) = [obst(2,i);obst(4,i)];
            % make the object
            MakeObj(curObstSet, 'r');
        end
        
        plot(agentPos(1,1,i)+L/2,agentPos(3,1,i)+W/2,'hk','MarkerSize',19);
    end    
    
    for i = 1:pA
        plot(agentPos(1,:,i)+L/2,agentPos(3,:,i)+W/2,'LineWidth',2,'color','red');
    end
    
    plot(target(1),target(2),'k*','MarkerSize',19);
    
    print(h,'sim_SO','-depsc','-r0')
    
    
    
end