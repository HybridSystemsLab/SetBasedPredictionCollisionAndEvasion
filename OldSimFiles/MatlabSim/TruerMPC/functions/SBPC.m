
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
    %quadtraj = ones(N+1,3,S,K-1);
    theta = linspace(0, 2*pi, K);
    
           
     
    velPerm = permn(VELOCITY,N);
    angPerm = permn(theta(1:end-1),N);
    
    [vm, vn] = size(velPerm);
    [am, an] = size(angPerm);
    
    
    
    quadtraj = zeros(N,3,m+1,N*S);
    
    trajCount = 1;
    
    % iterate though each set of speeds and each set of angles
    
    for i = 1:vm
        for j = 1:am
            for p = 0:m

                % initialize coordinate 
                % location of quad or one of the points in set X_0
                if(p == 0)
                    lastX = state(1);
                    lastY = state(2);
                    lastZ = state(3);
                else
                    lastX = s_1(p,1);
                    lastY = s_1(p,2);
                    lastZ = s_1(p,3);
                end

                curTraj = zeros(N,3);
                for k = 1:N

                    curTraj(k,:) = [lastX,lastY,lastZ];

                    % propagate the quad one time step with velocity and angle
                    x = lastX + TIMESTEP * velPerm(i,k) * cos(angPerm(j,k));
                    y = lastY + TIMESTEP * velPerm(i,k) * sin(angPerm(j,k));
                    z = lastZ;

                    lastX = x;
                    lastY = y;
                    lastZ = z;

                end

                % store trajectory
                
                quadtraj(:,:,trajCount,p+1) = curTraj;
 
            end 
            % adjust trajectory count after each set X_n
            trajCount = trajCount+1;
        end
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
    trajCost = zeros(trajCount);
    
    % iterate through each trajectory
    for t = 1:trajCount-1

        % run collision detection algorithm
        for i = 1:N-1

            % put data in right form for making convex hull
            % all points from successive sets
            for j = 1:m
                quadset1(j,:) = quadtraj(i,:,t,j);
                quadset2(j,:) = quadtraj(i+1,:,t,j);
            end

            % create intersample convex hull for trajectory k for quadrotor
            quadIntersampleSet = [quadset1; quadset2];
            quadrotorConvexHull = MakeObj(quadIntersampleSet, 'green');

            %collisionFlag = GJK(projectileConvexHull(i), quadrotorConvexHull, iterationsAllowed);
            [dist,~,~,~]=GJK_dist(projectileConvexHull(i),quadrotorConvexHull);

            if(dist < sigma)
                fprintf('Collision in trajectory %d  at speed %d at time step %d\n\r', k,VELOCITY(s),i);
                safeTraj(t) = 0;
                trajCost(t) = inf;
                break;
            else
                [cost,~,~,~] = GJK_dist(targetObj,quadrotorConvexHull);
                collisionFlag = GJK(targetObj,quadrotorConvexHull,iterationsAllowed);

                if(collisionFlag)
                    cost
                    cost = 0;
                    error('must increase minimum cost');
                end
                trajCost(t) = trajCost(t) + cost;
            end
        end
        t
    end
    
    
    
    %% find optimal input
    
    
    % find minimum cost and return first coordinate in that trajectory
    %[minCostList, minCostIndexList] = min(trajCost);
    %[minCost, minCostKIndex] = min(minCostList);
    %minCostSIndex = minCostIndexList(minCostKIndex);
    
    
    
    %u_opt = quadtraj(2,:,minCostSIndex,minCostKIndex,1);
    %input = u_opt;
    
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    grid on
    %}
    
    input = 0;
end