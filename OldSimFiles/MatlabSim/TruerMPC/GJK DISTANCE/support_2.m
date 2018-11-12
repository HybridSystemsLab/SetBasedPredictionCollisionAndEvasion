function [point,point1 ,point2] = support_2(shape1,shape2,v)
%Support function to get the Minkowski difference.
if size(shape1.Vertices,1)==1
    point1=shape1.Vertices(1,:)';
else
    point1 = getFarthestInDir(shape1, v);
end
if size(shape2.Vertices,1)==1
    point2=shape2.Vertices(1,:)';
else
    point2 = getFarthestInDir(shape2, -v);
end
point = point1 - point2;
end