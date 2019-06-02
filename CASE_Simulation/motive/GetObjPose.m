function [x,y,theta]=GetObjPose(Client, obj_ID)
    

    %% set debug to 1 if Would like to run the code by itself
    
    DEBUG =0;
    if DEBUG ==1
        % Setup for motive....no need to change this
        dllPath = fullfile('c:','Users','yzeleke','Desktop','HSL_exp','NatNetSDK','lib','x64','NatNetML.dll');
        assemblyInfo = NET.addAssembly(dllPath);
        Client = NatNetML.NatNetClientML(0);
        HostIP = char('127.0.0.1');
        Client.Initialize(HostIP, HostIP);
        obj_ID = 2
    end
    
    
    frameOfData = Client.GetLastFrameOfData();
    obj_pose = frameOfData.RigidBodies(obj_ID);
    
    

   
    
    % sensor output = x,y, and heading 
    x = obj_pose.x;
    y = -1*obj_pose.z;
    % set the objects heading angel to zero since not used
    theta =0;
        
   if DEBUG ==1
        [x,y,theta]
   end
    
 
   
   
    
    if DEBUG ==1
        Client.Uninitialize();
    end
    

end