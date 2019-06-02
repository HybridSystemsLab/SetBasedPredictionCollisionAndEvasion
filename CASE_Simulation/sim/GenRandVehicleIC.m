function x = GenRandVehicleIC(min,max,sizeSquare)

    numSquare = ((max-min)/sizeSquare)^2;
    x = zeros(numSquare,3);
    
    sqCount = 1;
    for i = 1:sqrt(numSquare)
        for j = 1:sqrt(numSquare)
            x(sqCount,:) = [min+i*sizeSquare+rand-1,min+j*sizeSquare+rand-1,((2*pi)*rand)];
            sqCount = sqCount+1;
        end
    end
end