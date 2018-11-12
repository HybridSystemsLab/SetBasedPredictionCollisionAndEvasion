%How many iterations to allow for collision detection.
iterationsAllowed = 6;

% Make a figure
figure(1)
hold on

% constants for set making
cntr_1 = [0.0, 0.0, 0.0];
cntr_2 = [1.0, 0.0, 0.0];

r_1 = 0.5;
r_2 = 0.2;

tdis = 11;
pdis = 6;

% create point cloud
sphere_1 = CreateSphere(cntr_1, r_1, tdis, pdis);
sphere_2 = CreateSphere(cntr_2, r_2, tdis, pdis);

% make individual convex hulls 
S1Obj = makeObj(sphere_1);
S2Obj = makeObj(sphere_2);

% Make tube
figure(2)

% create points cloud
sphere_3 = [sphere_1; sphere_2];

% make combined convex hull
S3Obj = makeObj(sphere_3);

% check for collision
flag = GJK(S1Obj, S2Obj, iterationsAllowed)


% returns convex hull from point cloud
function obj = makeObj(points)

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