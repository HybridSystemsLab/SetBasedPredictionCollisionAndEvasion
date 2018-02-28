
% prediction algorithm
% state - [x, y, z, px, py, pz, pxdot, pydot, pzdot]
function input = SBPC(state,target,sigma,QR,PR,TDIS,PDIS,N,K,TIMESTEP,VELOCITY)

    % number of iterations to allow for collision detection.
    iterationsAllowed = 6;
    
    % target object
    targetSet = CreateSphere(target, 0.001, 5, 5);
    targetObj = MakeObj(targetSet, 'none');
    
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
    projtraj = ProjectilePredict(state(4:9), simLength);
    
    % set points
    [m,n] = size(s_0);
    for i = 1:m
        
        % create initial condition for point in set
        setState = [s_0(i,:), v_0(i,:)];
        
        % predict trajectory
        projtraj(:,:,i+1) = ProjectilePredict(setState, simLength);
        
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
    
    %% collision detection and trajectory optimization
    
    safeTraj = ones(S,K-1);
    for i = 1:N-1
        
        % put data in right form for making convex hull
        for j = 1:m+1
            projset1(j,:) = projtraj(i,:,j);
            projset2(j,:) = projtraj(i+1,:,j);
        end

        % create intersample convex hull for projectile
        projintersampleSet = [projset1; projset2];
        projectileConvexHull(i) = MakeObj(projintersampleSet, 'red');
    end
        
    % evaluate each trajectory 
    trajCost = zeros(S,K-1);
    for k = 1:K-1

        for s = 1:S

            % run collision detection algorithm
            for i = 1:N-1
    
                % put data in right form for making convex hull
                % all points from successive sets
                for j = 1:m
                    quadset1(j,:) = setquadtraj(i,:,s,k,j);
                    quadset2(j,:) = setquadtraj(i+1,:,s,k,j);
                end

                % create intersample convex hull for trajectory k for quadrotor
                quadIntersampleSet = [quadset1; quadset2];
                quadrotorConvexHull = MakeObj(quadIntersampleSet, 'green');

                %collisionFlag = GJK(projectileConvexHull(i), quadrotorConvexHull, iterationsAllowed);
                [dist,~,~,~]=GJK_dist(projectileConvexHull(i),quadrotorConvexHull);
                
                if(dist < sigma)
                    fprintf('Collision in trajectory %d  at speed %d at time step %d\n\r', k,VELOCITY(s),i);
                    safeTraj(s,k) = 0;
                    trajCost(s,k) = inf;
                    break;
                else
                    [cost,~,~,~] = GJK_dist(targetObj,quadrotorConvexHull);
                    collisionFlag = GJK(targetObj,quadrotorConvexHull,iterationsAllowed);
                    if(cost < 0.1) 
                        cost = 0;
                    elseif(cost > 0.1 && collisionFlag)
                        cost
                        error('must increase minimum cost');
                    end
                    trajCost(s,k) = trajCost(s,k) + cost;
                end
            end
        end
    end
    
    
    
    %% find optimal input
    
    
    % find minimum cost and return first coordinate in that trajectory
    [minCostList, minCostIndexList] = min(trajCost);
    [minCost, minCostKIndex] = min(minCostList);
    minCostSIndex = minCostIndexList(minCostKIndex);
    
    u_opt = setquadtraj(2,:,minCostSIndex,minCostKIndex,1);
    input = u_opt;
    
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    grid on
    
    
end