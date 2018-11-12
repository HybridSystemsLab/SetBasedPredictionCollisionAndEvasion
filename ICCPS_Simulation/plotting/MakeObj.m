
% returns convex hull from point cloud
function obj = MakeObj(points, color)
    %figure()
    % create face representation and create convex hull
    
    F = convhull(points(1,:), points(2,:));
    fill(points(1,F),points(2,F),color);
    
    %{
    S.Vertices = transpose(points(1:2,:));
    S.Faces = F;
    S.FaceVertexCData = jet(size(points,1));
    S.FaceColor = 'interp';
    
    
    
    if(strcmp(color,'red'))
        obj = patch('Faces',S.Faces,'Vertices',S.Vertices,'FaceColor','red');
    elseif(strcmp(color,'green'))
        obj = patch('Faces',S.Faces,'Vertices',S.Vertices,'FaceColor','green');
    else
        obj = patch(S);
    end
    %}
        
    
end