function thrust = ToThrust(thrust)
    % Trust  for RC is from 1000-2000 on arduino controller therfore
    % has to be changed to fit that range. Moreover the steps are 10
    % between trust, meaning it works in increament of 10.
    
    %Rough test was conducted and the vehicle max spped is 6ft/s
    
    %according to exp the speed is linear with slop of 500/6 
    m = 500/6;
    
    if thrust>0
        thrust = m*(thrust-6)+2000;
    else
        thrust = m*(thrust+6)+1000;
    end
    %rudder = angel + 1500;
    %rudder = roundn(rudder, 2) % round to 100th
    
    if thrust > 2000
        thrust =2000;
    elseif thrust <1000
            thrust =1000;
    else
           %since the steering angel of the vehicle only works in increment of 25
        thrust = round(thrust/10)*10; 
    end


end