
function minDist = PolytopeMinDist(X1,X2)
    
    % declare constraints for fmincon
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
    
    % get sizes of vertices for polytopes
    [m1,n1] = size(X1);
    [m2,n2] = size(X2);
    
    % create lambda vectors
        lamy1 = (1/n1)*ones(n1,1);
        lamy2 = (1/n2)*ones(n2,1);  
    
    
	% X1 has more vertices than X2
    if(n1 > n2)
        
        % pad n2 with -1
        for i = 1:(n1-n2)
            lamy2 = [lamy2; -1];
        end
        
        % create n1 x 2 matrix
        x0 = [lamy1 lamy2];
        
        % function uses trimmed lambda1 vector -- x0(1:end-(n1-n2),2))
        fun = @(x0)(norm((X1*x0(:,1))-(X2*x0(1:end-(n1-n2),2))));
      
    % X2 has more vertices than X1
    elseif(n2 > n1)
        
        % pad n2 with -1
        for i = 1:(n2-n1)
            lamy1 = [lamy1; -1];
        end
        
        % create n2 x 2 matrix
        x0 = [lamy1 lamy2];
        
        % function uses trimmed lambda1 vector -- x0(1:end-(n2-n1),1)
        fun = @(x0)(norm((X1*x0(1:end-(n2-n1),1))-(X2*x0(:,2))));
    
    % X1 and X2 have equal number of vertices
    else
        x0 = [lamy1 lamy2];
        fun = @(x0)(norm((X1 * x0(:,1))-(X2 * x0(:,2))));
    end
    
    % evaluate fmincon
    x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,@Constraints);
    
    % return min distance
    minDist = fun(x);
end

