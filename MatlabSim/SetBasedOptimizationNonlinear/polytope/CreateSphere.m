
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
            
            nextPoint = [x;y;z];
            points = [points, nextPoint];
            
            % removes duplicate points at the top and bottom of sphere
			if(phis(i) == 0 || phis(i) == pi)
				break
            end
        end
    end   
end
