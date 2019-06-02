

function [f,g] = PolytopeDist(X1,X2,lambda,n1,n2,n)
    
    f = norm((X1 * lambda(1:n1))-(X2 * lambda(n1+1:n)))^2;
    %{
    g = zeros(n,1);
    
    for i = 1:n1
        g(i) = 2*((X1(1,:)*lambda(1:n1))-(X2(1,:)*lambda(n1+1:n)))*X1(1,i) ...
        + 2*((X1(2,:)*lambda(1:n1))-(X2(2,:)*lambda(n1+1:n)))*X1(2,i) ...
        + 2*((X1(3,:)*lambda(1:n1))-(X2(3,:)*lambda(n1+1:n)))*X1(3,i);
    end

    for i = 1:n2
        g(i) = 2*((X1(1,:)*lambda(1:n1))-(X2(1,:)*lambda(n1+1:n)))*(-X2(1,i)) ...
        + 2*((X1(2,:)*lambda(1:n1))-(X2(2,:)*lambda(n1+1:n)))*(-X2(2,i)) ...
        + 2*((X1(3,:)*lambda(1:n1))-(X2(3,:)*lambda(n1+1:n)))*(-X2(3,i));
    end
    %}
end