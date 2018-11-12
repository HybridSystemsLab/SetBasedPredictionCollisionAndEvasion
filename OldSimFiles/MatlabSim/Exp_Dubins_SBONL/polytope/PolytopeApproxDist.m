

function dist = PolytopeApproxDist(X1,X2)

    [m1,n1] = size(X1);
    [m2,n2] = size(X2);
    
    c1 = transpose(mean(transpose(X1)));
    c2 = transpose(mean(transpose(X2)));
    
    dist1 = zeros(n1,1);
    for i = 1:n1
        dist1(i) = norm(X1(:,i)-c1);
    end
    
    dist2 = zeros(n2,1);
    for i = 1:n2
        dist2(i) = norm(X2(:,i)-c2);
    end
    
    cDist = norm(c1-c2);
   
    d1 = max(dist1);
    d2 = max(dist2);
    
    dist = cDist-d1-d2;


end