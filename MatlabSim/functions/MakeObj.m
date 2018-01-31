
% returns convex hull from point cloud
function obj = MakeObj(points)
    %figure()
    % create face representation and create convex hull
    F = convhull(points(:,1), points(:,2), points(:,3));
    S.Vertices = points;
    S.Faces = F;
    S.FaceVertexCData = jet(size(points,1));
    S.FaceColor = 'interp';
    obj = patch(S);
    
end