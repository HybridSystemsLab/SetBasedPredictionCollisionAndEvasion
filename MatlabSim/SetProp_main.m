%                              %
% set based predictive control %
%                              %

clc;clear all;close all

% initial conditions

% collision at end of one trajectory
%p_0 = [3.5, 4.4, 2.0, -3.0, -2.0, 4.0];
%x_0 = [0.0, 0.0, 0.0];


% 
p_0 = [1.0, 0.5, 0.5, -1.0, 0.0, 1.5];
x_0 = [0.0, 0.0, 0.0];
target = [-2.0, -2.0, 0.0];


state = [p_0, x_0];
mpc(state, target)



% returns convex hull from point cloud
function obj = makeObj(points)
    %figure()
    % create face representation and create convex hull
    F = convhull(points(:,1), points(:,2), points(:,3));
    S.Vertices = points;
    S.Faces = F;
    S.FaceVertexCData = jet(size(points,1));
    S.FaceColor = 'interp';
    obj = patch(S);

end


% creates a point cloud in a sphere around the center
function points = CreateSphere(center, r, thetadis, phidis)

	% angle discretization
	thetas = linspace(0,2*pi,thetadis);
	phis = linspace(0,pi,phidis);
    
	% point calculation
	points = [];
    x = [];
    y = [];
    z = [];
	for i = 1:length(phis)
		for j = 1:length(thetas)
            
            % removes duplicate point at theta = 2*pi
            if(thetas(j) == 2*pi)
                break
            end
                
            x = (r * sin(phis(i)) * cos(thetas(j))) + center(1);
			y = (r * sin(phis(i)) * sin(thetas(j))) + center(2);
			z = (r * cos(phis(i))) + center(3);

            points = [points; x, y, z];
            
            % removes duplicate points at the top and bottom of sphere
			if(phis(i) == 0 || phis(i) == pi)
				break
            end
        end
    end   
end


function trajectory = ProjectilePredict(p_0, simTime)

    % set up simulink
    set_param('projectile/rx','Value',num2str(p_0(1)));
    set_param('projectile/ry','Value',num2str(p_0(2)));
    set_param('projectile/rz','Value',num2str(p_0(3)));
    set_param('projectile/vx','Value',num2str(p_0(4)));
    set_param('projectile/vy','Value',num2str(p_0(5)));
    set_param('projectile/vz','Value',num2str(p_0(6)));
    
    set_param('projectile', 'StopTime', num2str(simTime));

    % run simulation
    sim('projectile');
    
    trajectory = projectilePos;
    
end


function totalCost = CostSum(trajectory, target, N)
    
    totalCost = 0;
    
    % sum distances between each point in trajectory and target
    for i = 1:N
        cost = pdist([trajectory(i,:); target], 'euclidean');
        totalCost = totalCost + cost;
    end
end

% prediction algorithm
% state - [x, y, z, px, py, pz, pxdot, pydot, pzdot]
function input = mpc(state, target)

    % set based prediction constants
    % R - radius
    % TDIS - theta discretization param (n-1)
    % PDIS - phi discretixation param (n-1)
    R = 0.1;
    TDIS = 5;
    PDIS = 5;
    
    % number of iterations to allow for collision detection.
    iterationsAllowed = 6;

    % mpc contants
    % N - prediction horizon
    % K - discretization parameter
    N = 25;
    K = 9;
    TIMESTEP = 0.05; % sec
    VELOCITY = 2; %m/s
    
    %% projectile set based prediction
    
    % get initial coordinates of projectile
    projcntr = state(1:3);
    velcntr = state(4:6);
    
    % create point cloud from initial condition 
    s_0 = CreateSphere(projcntr, R, TDIS, PDIS);    % position
    v_0 = CreateSphere(velcntr, R, TDIS, PDIS);     % velocity
    
    % middle point with simulink
    simLength = double(N*TIMESTEP);
    projtraj = ProjectilePredict(state(1:6), simLength);
    
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
    quadcntr = state(7:9);
    % create point cloud from initial condition
    s_1 = CreateSphere(quadcntr, R, TDIS, PDIS);
    [m,n] = size(s_1);
    
    % N+1 because of initial condition
    % K-1 because 0 and 2pi given equivalent trajectories
    quadtraj = ones(N+1,3,K-1);
    theta = linspace(0, 2*pi, K);
    j = linspace(0,N,N+1);
    
    
    for i = 1:m+1
        for k = 1:K-1
            
            % first is middle point then the set
            if(i == 1)
                % integrator dynamics
                x = quadcntr(1)+transpose(j)*TIMESTEP*VELOCITY*cos(theta(k));
                y = quadcntr(2)+transpose(j)*TIMESTEP*VELOCITY*sin(theta(k));
                z = quadcntr(3)*ones(N+1,1);
                quadtraj(:,:,k) = [x,y,z];
            else
                % integrator dynamics
                x = s_1(i-1,1)+transpose(j)*TIMESTEP*VELOCITY*cos(theta(k));
                y = s_1(i-1,2)+transpose(j)*TIMESTEP*VELOCITY*sin(theta(k));
                z = s_1(i-1,3)*ones(N+1,1);
                quadtraj(:,:,k) = [x,y,z];
            end
        end
        
        setquadtraj(:,:,:,i) = quadtraj;
    end
    

    %% collision detection
    
    safeTraj = ones(1,K-1);
    for i = 1:N-1
        
        % put data in right form for making convex hull
        for j = 1:m+1
            projset1(j,:) = projtraj(i,:,j);
            projset2(j,:) = projtraj(i+1,:,j);
        end

        % create intersample convex hull for projectile
        projintersampleSet = [projset1; projset2];
        projectileConvexHull = makeObj(projintersampleSet);
        

        % check which trajectories are safe
        
        for k = 1:K-1
            
            % only continue to calculate for the trajectory if no previous
            % collision - saves computation time
            
            if(safeTraj(k))
                % put data in right form for making convex hull
                for j = 1:m
                    quadset1(j,:) = setquadtraj(i,:,k,j);
                    quadset2(j,:) = setquadtraj(i+1,:,k,j);
                end

                % create intersample convex hull for trajectory k for quadrotor
                quadIntersampleSet = [quadset1; quadset2];
                quadrotorConvexHull = makeObj(quadIntersampleSet);

                % run collision detection algorithm
                collisionFlag = GJK(projectileConvexHull, quadrotorConvexHull, iterationsAllowed);


                if(collisionFlag)
                    fprintf('Collision in trajectory %d\n\r', k);
                    safeTraj(k) = 0;
                end
            end
        end     
    end
    safeTraj
    
    
    %% soft constraint - optimization
    
    trajCost = zeros(1,K-1);
    
    % check each trajectory
    for k = 1:K-1
        
        % calculate cost of trajectory
        % cost is inf if the trajectory results in collsion
        if(safeTraj(k))
            trajCost(k) = CostSum(setquadtraj(:,:,k,1), target, N);
        else
            trajCost(k) = inf;
        end
    end
    
    % find minimum cost and return first coordinate in that trajectory
    [minCost, minCostTraj] = min(trajCost);
    input = setquadtraj(2,:,minCostTraj,1);
    
    xlabel('x axis');
    ylabel('y axis');
    zlabel('z axis');
    grid on
    
end
