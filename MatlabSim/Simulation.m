%
% simulation for set based predictive control
%

clc;clear all;close all


% Early collision examples

p_0 = [1.0, 0.5, 0.5, -1.2, -0.2, 1.5];
x_0 = [0.0, 0.0, 0.0];
target = [1.0, 1.0, 0.0];
state = [x_0, p_0];

%SetBasedPredictiveControl(state, target);


newState(1,:) = state;

iterations = 35;
for i = 1:iterations
    
    newState(i+1,:) = SimulationSetBasedPredictiveControl(state, target);
    
    state = newState(i+1,:);

    disp(i)
end


scatter3(x_0(1),x_0(2),x_0(3),'*');
hold on
scatter3(target(1), target(2), target(3));
scatter3(newState(:,1), newState(:,2), newState(:,3));
scatter3(newState(:,4), newState(:,5) ,newState(:,6));
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
grid on

for i = 1:iterations+1
    projDistance(i) = pdist([newState(i,1:3); newState(i,4:6)], 'euclidean');
    targDistance(i) = pdist([newState(i,1:3); target(1:3)], 'euclidean');
end

t = linspace(0,iterations*0.05,iterations+1);
figure()
plot(t, projDistance)
xlabel('x axis');
ylabel('y axis');
ylim([0.0 2.0]);
xlim([0.0 2.0]);
title('Distance Between Quadrotor and Projectile');
grid on

figure()
plot(t,targDistance)
xlabel('x axis');
ylabel('y axis');
ylim([0.0 2.0]);
xlim([0.0 2.0]);
title('Distance Between Quadrotor and Target');
grid on



