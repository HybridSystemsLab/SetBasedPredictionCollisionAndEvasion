function rudder = Torudder(angel)
    % Rudder angel for RC is from 1000-2000 on arduino controller therfore
    % has to be changed to fit that range. Moreover the steps are 25
    % between angels, meaning it works in increament of 25.
  
    if angel>0
        m = -50/3; %according to exp the angel is linear
        rudder = m*(angel-30)+1000;
    else
         m = -50/3; %according to exp the angel is linear
        rudder = m*(angel+30)+2000;
    end
    %rudder = angel + 1500;
    %rudder = roundn(rudder, 2) % round to 100th
    
    if rudder > 2000
        rudder =2000;
    elseif rudder <1000
            rudder =1000;
    else
           %since the steering angel of the vehicle only works in increment of 25
        rudder = round(rudder/25)*25; 
    end


end