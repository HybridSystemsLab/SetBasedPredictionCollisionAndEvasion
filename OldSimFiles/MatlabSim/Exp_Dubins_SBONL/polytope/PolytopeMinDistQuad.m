function minDist = PolytopeMinDistQuad(X1,X2,options)
    
    % get sizes of vertices for polytopes
    [m1,n1] = size(X1);
    [m2,n2] = size(X2);
    
    % assuming m1 == m2
    X = [X1 -X2];
    X = X'*X;
    
    
    
   [x,minDist] = quadprog(X,[],[],[],[ones(1,n1) zeros(1,n2);zeros(1,n1) ones(1,n2)],ones(2,1),zeros(n1+n2,1),ones(n1+n2,1),[],options);
    
   minDist = sqrt(minDist);
end