function [c,ceq] = Constraints(x)
    
    % seperate lambda vectors
    lamy1 = x(:,1);
    lamy2 = x(:,2);
    
    [m1,n1] = size(lamy1);
    [m2,n2] = size(lamy2);
    
    % find -1 in lambda1 if any
    y1End = 0;
    for i = 1:m1
        if(lamy1(i) < 0)
            y1End = i;
            break;
        end
    end
    
    % find -1 in lambda2 if any
    y2End = 0;
    for i = 1:m2
        if(lamy2(i) < 0)
            y2End = i;
            break;
        end
    end
    
    
    % shorten lambdas if necessary
    if(y1End)
        lamy1 = lamy1(1:y1End-1);
    elseif(y2End)
        lamy2 = lamy2(1:y2End-1);
    else
        % no shortening necessary  
    end
    
    % redefine sizes for trimmed lambda vectors
    [m1,n1] = size(lamy1);
    [m2,n2] = size(lamy2);
    
    % create contraints vector c(x)
    % 0 < lambda_i < 1
    c = [];
    for i = 1:m1
        c = [c -lamy1(i)]; 
    end
    for i = 1:m1
        c = [c lamy1(i)-1]; 
    end
    for i = 1:m2
        c = [c -lamy2(i)]; 
    end
    for i = 1:m2
        c = [c lamy2(i)-1]; 
    end
    c = transpose(c);
    
    % create contraints vector ceq(x)
    % sum (lambda_i) = 1
    lam1sum = 0;
    for i = 1:m1
        lam1sum = lam1sum + lamy1(i);
    end
    
    lam2sum = 0;
    for i = 1:m2
        lam2sum = lam2sum + lamy2(i);
    end
    
    ceq = [lam1sum-1;
           lam2sum-1];
    
end