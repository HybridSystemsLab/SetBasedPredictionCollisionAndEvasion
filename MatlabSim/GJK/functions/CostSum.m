
% calculates total cost of a trajectory
function totalCost = CostSum(trajectory, target, N)
    
    totalCost = 0;
    
    % sum distances between each point in trajectory and target
    for i = 1:N
        cost = pdist([trajectory(i,:); target], 'euclidean');
        totalCost = totalCost + cost;
    end
end