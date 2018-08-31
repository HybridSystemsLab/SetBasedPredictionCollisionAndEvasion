function x = Dubin(x0, u, ts)

    [mP,nP] = size(x0);
    [mH,nH] = size(u);
    
    % state X time
    % state = (x1,x2,y1,y2,theta)
    x = zeros(nP,nH+1);
    x(:,1) = x0;
    
    % apply integrator dynamics
    for i = 1:nH
        if(u(2,i) ~= 0)
            x(1,i+1) = x(1,i) + ts*u(1,i)*(((cos(x(5,i)+u(2,i)))*sin(u(2,i)))/u(2,i));
            x(2,i+1) = x(2,i) + ts*u(1,i)*(((cos(x(5,i)+u(2,i)))*sin(u(2,i)))/u(2,i));
            x(3,i+1) = x(3,i) + ts*u(1,i)*(((sin(x(5,i)+u(2,i)))*sin(u(2,i)))/u(2,i));
            x(4,i+1) = x(4,i) + ts*u(1,i)*(((sin(x(5,i)+u(2,i)))*sin(u(2,i)))/u(2,i));
            x(5,i+1) = x(5,i) + 2*u(2,i);
        else
            x(1,i+1) = x(1,i) + ts*u(1,i)*cos(x(5,i));
            x(2,i+1) = x(2,i) + ts*u(1,i)*cos(x(5,i));
            x(3,i+1) = x(3,i) + ts*u(1,i)*sin(x(5,i));
            x(4,i+1) = x(4,i) + ts*u(1,i)*sin(x(5,i));
            x(5,i+1) = x(5,i);
        end
    end
    

end