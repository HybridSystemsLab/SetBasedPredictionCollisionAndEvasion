function f = plotManeuver(x0,obstacle,target, pose,xpred,...
            carLength,carWidth,ObstacleWidth,ObstacleLength,ObstacleSafedist)
% Create figure
f = figure;

% Plot the Ego vehicle.
% carLength = 0.48;
% carWidth = 0.25;
% 
% ObstacleWidth = 0.20;
% ObstacleLength= 0.20;
% 
 ObstacleSafety_W = ObstacleWidth+ObstacleSafedist;
 ObstacleSafety_L = ObstacleLength+ObstacleSafedist;

X0 = x0(1);
Y0 = x0(2);
plot(X0,Y0,'gx'); hold on; grid on;
rectangle('Position',[X0-carLength/2,Y0 - carWidth/2,carLength,carWidth]);


%plot the Target
plot(target(1),target(2),'bx');

% Plot the static obstacle.
plot(obstacle(1),obstacle(2),'rx');
rectangle('Position',[obstacle(1)-ObstacleLength/2,obstacle(2) - ObstacleWidth/2,ObstacleLength,ObstacleWidth]);


% Plot the safe zone around obstacle.
rectangle('Position',[obstacle(1)-ObstacleSafety_L/2,obstacle(2) - ObstacleSafety_W/2,ObstacleSafety_L,ObstacleSafety_W],...
    'LineStyle','--','EdgeColor','r');

% Reset the axis.
axis([-2 2 -2 2]);
xlabel('X');
ylabel('Y');
title('Obstacle Avoidance Maneuver');


plot(pose(:,1),pose(:,2));
xlabel('x')
ylabel('y')
hold on
grid on


%%plot predited vehicle path

%get the size of time step
n = size(xpred,3);
%get the prediction horizon used
N = size(xpred,2);


cc = jet(15);
for i =1:n  
    for j=1:N
        if xpred(1,j,i)==0  && xpred(2,j,i)==0 %skip if vector is all zeros
            continue;
        end
        x_pose= xpred(1,j,i);
        y_pose= xpred(3,j,i);
        h= abs(xpred(4,j,i)-y_pose);
        w= abs(xpred(2,j,i)-x_pose);
     rectangle('Position',[x_pose,y_pose,w,h],...
    'EdgeColor',cc(mod(i,15)+1,:));%C{mod(i,length(C))+1});
    end
end
end