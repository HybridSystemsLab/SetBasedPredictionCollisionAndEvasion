function [x,y,theta]=GetVehiclePose(Client, obj_ID)
    

    %% set debug to 1 if Would like to run the code by itself
    
    DEBUG =0;
    if DEBUG ==1
        % Setup for motive....no need to change this
        dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
        assemblyInfo = NET.addAssembly(dllPath);
        Client = NatNetML.NatNetClientML(0);
        HostIP = char('127.0.0.1');
        Client.Initialize(HostIP, HostIP);
        obj_ID = 1;
    end
    
    
    frameOfData = Client.GetLastFrameOfData();
    vehicle = frameOfData.RigidBodies(obj_ID);
    
    r =4;
    l=2;
    f=1;
    b=3;
    
    right_node = [vehicle.Markers(r).x -vehicle.Markers(r).z -vehicle.Markers(r).y];
    left_node = [vehicle.Markers(l).x -vehicle.Markers(l).z -vehicle.Markers(l).y];
    front_node = [vehicle.Markers(f).x -vehicle.Markers(f).z -vehicle.Markers(f).y];
    back_node = [vehicle.Markers(b).x -vehicle.Markers(b).z -vehicle.Markers(b).y];
    
    
   if DEBUG ==1
        front_node
        left_node
        right_node
        back_node
   end
   
    
    % sensor output = x,y, and heading 
    x = (front_node(1,1) + back_node(1,1))/2;
    y = (front_node(1,2) + back_node(1,2))/2;
    
 
    
    % controller output = v,w
    x_d = front_node(1) - back_node(1);
    y_d = front_node(2) - back_node(2);
    theta = 1*atan2(y_d,x_d);
   
    
    if DEBUG ==1
        Client.Uninitialize();
    end
    

end