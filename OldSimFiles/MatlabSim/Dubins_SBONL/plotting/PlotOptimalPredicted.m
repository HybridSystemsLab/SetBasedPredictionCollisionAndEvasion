
function PlotOptimalPredicted(agentPos, obst, threshold, target)

   figure()
    hold on
    
  
    % mA - coordinate (usually size 3)
    % nA - time step, equal to iterations in simulation
    [mA,nA] = size(agentPos);
    
    % create objects/convex hulls for each set at each time step
    for i = 1:nA
        
        % format points for MakeObj function
        curSet = zeros(3,4);
        
        curSet(:,1) = [agentPos(1,i);agentPos(3,i);0];
        curSet(:,2) = [agentPos(1,i);agentPos(4,i);0];
        curSet(:,3) = [agentPos(2,i);agentPos(3,i);0];
        curSet(:,4) = [agentPos(2,i);agentPos(4,i);0];


        % make the object
        MakeObj(curSet, 'g');
    end
    
    
    [mO,nO,pO] = size(obst);
    
    % NOTE: pA == nO
    
    % plot obstacle with threshold
    for i = 1:nO
        for j = 1:pO
            % create sphere for obstacle (threshold and obstacle location)
            [x,y,z] = sphere;
            x = threshold*x+obst(1,i,j);
            y = threshold*y+obst(2,i,j);
            z = threshold*z+obst(3,i,j);

            [mS,nS] = size(z);

            % plot obst
            C = zeros(mS,nS,3);
            C(:,:,1) = C(:,:,1) + 1;
            s = surf(x,y,z,C);
            scatter3(obst(1,i,j), obst(2,i,j), obst(3,i,j), '*')
        end
    end
    
    scatter3(target(1), target(2), target(3), '*')
    
    xlabel('x axis')
    ylabel('y axis')
    zlabel('z axis')
    grid on
    
end