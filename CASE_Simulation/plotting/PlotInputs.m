function PlotInputs(u,speed,steer)

    [m,n] = size(u);
    
    figure()
    
    subplot(1,2,1)
    stairs(0:1:n-1,u(1,:));
    axis([0 n-1 -0.1*speed (speed+0.1*speed)])
    xlabel('Time')
    ylabel('Speed - m/s')
    
    subplot(1,2,2)
    stairs(0:1:n-1,u(2,:));
    axis([0 n-1 (-steer-0.1*steer) (steer+0.1*steer)])
    xlabel('Time')
    ylabel('Steering angle - theta')

    
    title('Sequence of Inputs')
end