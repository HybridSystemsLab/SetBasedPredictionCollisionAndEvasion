
% prediction algorithm
% state - [x, y, z, px, py, pz, pxdot, pydot, pzdot]
function input = SimulationSBPC(state,target,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY)
    
    % number of iterations to allow for collision detection.
    iterationsAllowed = 6;
    
    % get possible velocities
    S = length(VELOCITY);
    
    %% projectile set based prediction
    
    % get initial coordinates of projectile
    projcntr = state(4:6);
    velcntr = state(7:9);
    
    % create point cloud from initial condition 
    s_0 = CreateSphere(projcntr, PR, TDIS, PDIS);    % position
    v_0 = CreateSphere(velcntr, PR, TDIS, PDIS);     % velocity
    
    % middle point with simulink
    simLength = double(N*TIMESTEP);
    [projtraj, projvel] = SimulationProjectilePredict(state(4:9), simLength);
    
    % set points
    [m,n] = size(s_0);
    for i = 1:m
        
        % create initial condition for point in set
        setState = [s_0(i,:), v_0(i,:)];
        
        % predict trajectory
        [projtraj(:,:,i+1), projvel(:,:,i+1)] = SimulationProjectilePredict(setState, simLength);
        
    end
   
    
    %% quadrotor set based prediction
    
    % get initial coordinates of quad
    quadcntr = state(1:3);
    % create point cloud from initial condition
    s_1 = CreateSphere(quadcntr, QR, TDIS, PDIS);
    [m,n] = size(s_1);
    
    % N+1 because of initial condition
    % K-1 because 0 and 2pi given equivalent trajectories
    quadtraj = ones(N+1,3,S,K-1);
    theta = linspace(0, 2*pi, K);
    j = linspace(0,N,N+1);
    
    
    for i = 1:m+1
        for k = 1:K-1
            
            % first is middle point then the set
            if(i == 1)
                % integrator dynamics
                x = quadcntr(1)+transpose(j)*TIMESTEP*VELOCITY*cos(theta(k));
                y = quadcntr(2)+transpose(j)*TIMESTEP*VELOCITY*sin(theta(k));
                z = quadcntr(3)*ones(N+1,S);
                
                quadtraj(:,1,:,k) = x;
                quadtraj(:,2,:,k) = y;
                quadtraj(:,3,:,k) = z;
            else
                % integrator dynamics
                x = s_1(i-1,1)+transpose(j)*TIMESTEP*VELOCITY*cos(theta(k));
                y = s_1(i-1,2)+transpose(j)*TIMESTEP*VELOCITY*sin(theta(k));
                z = s_1(i-1,3)*ones(N+1,S);
                quadtraj(:,1,:,k) = x;
                quadtraj(:,2,:,k) = y;
                quadtraj(:,3,:,k) = z;
            end
        end
        
        setquadtraj(:,:,:,:,i) = quadtraj;
    end
    
    %% collision detection
    
    safeTraj = ones(S,K-1);
    for i = 1:N-1
        
        % put data in right form for making convex hull
        for j = 1:m+1
            projset1(j,:) = projtraj(i,:,j);
            projset2(j,:) = projtraj(i+1,:,j);
        end

        % create intersample convex hull for projectile
        projintersampleSet = [projset1; projset2];
        projectileConvexHull = SimulationMakeObj(projintersampleSet);
        
        % check which trajectories are safe
        
        for k = 1:K-1
            
            for s = 1:S
            
                % only continue to calculate for the trajectory if no previous
                % collision - saves computation time

                if(safeTraj(s,k))
                    % put data in right form for making convex hull
                    for j = 1:m
                        quadset1(j,:) = setquadtraj(i,:,s,k,j);
                        quadset2(j,:) = setquadtraj(i+1,:,s,k,j);
                    end  
                    
                    % create intersample convex hull for trajectory k for quadrotor
                    quadIntersampleSet = [quadset1; quadset2];
                    quadrotorConvexHull = SimulationMakeObj(quadIntersampleSet);

                    % run collision detection algorithm
                    collisionFlag = GJK(projectileConvexHull, quadrotorConvexHull, iterationsAllowed);

                    if(collisionFlag)
                        fprintf('Collision in trajectory %d  at speed %d\n\r', k,VELOCITY(s));
                        safeTraj(s,k) = 0;
                    end
                    
                end
            end
        end     
    end
  
    
    %% soft constraint - optimization
    
    trajCost = zeros(S,K-1);
    
    % check each trajectory
    for k = 1:K-1
        for s = 1:S
            % calculate cost of trajectory
            % cost is inf if the trajectory results in collsion
            if(safeTraj(s,k))
                trajCost(s,k) = CostSum(setquadtraj(:,:,s,k,1), target, N);
            else
                trajCost(s,k) = inf;
            end
        end
    end

    % throw error if all trajectories have infinity cost 
    if(range(range(trajCost)) == 0) % make sure all costs are the same
        if(trajCost(1,1) == inf)    % make sure the costs are inf (collision)
            msg = 'No collision free trajectories available';
            error(msg)
        end
        
    end
    
    % find minimum cost and return first coordinate in that trajectory
    [minCostList, minCostIndexList] = min(trajCost);
    [minCost, minCostKIndex] = min(minCostList);
    minCostSIndex = minCostIndexList(minCostKIndex);
    
    u_opt = setquadtraj(2,:,minCostSIndex,minCostKIndex,1);
    input = [u_opt,projtraj(2,:,1), projvel(2,:,1)];
    
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    grid on
    
    
end