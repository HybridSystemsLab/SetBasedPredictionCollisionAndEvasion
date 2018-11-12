function x = GenVehicleIC(min,max,sizeSquare,target)

    numSquare = ((max-min)/sizeSquare)^2;
    x = zeros(numSquare*2,3);
    
    sqCount = 1;
    for i = 1:sqrt(numSquare)
        for j = 1:sqrt(numSquare)
            
            xV = min+i*sizeSquare+rand-1;
            yV = min+j*sizeSquare+rand-1;
            xD = target(1) - xV;
            yD = target(2) - yV;

            x(sqCount,:) = [xV,yV,atan2(yD,xD)];
            sqCount = sqCount+1;
        end
    end
    
    for i = 1:sqrt(numSquare)
        for j = 1:sqrt(numSquare)
            x(sqCount,:) = [min+i*sizeSquare+rand-1,min+j*sizeSquare+rand-1,((2*pi)*rand)];
            sqCount = sqCount+1;
        end
    end
end