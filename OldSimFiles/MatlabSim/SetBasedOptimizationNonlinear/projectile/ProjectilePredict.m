function trajectory = ProjectilePredict(p_0, simTime)

    % set up simulink
    set_param('projectile/rx','Value',num2str(p_0(1)));
    set_param('projectile/ry','Value',num2str(p_0(2)));
    set_param('projectile/rz','Value',num2str(p_0(3)));
    set_param('projectile/vx','Value',num2str(p_0(4)));
    set_param('projectile/vy','Value',num2str(p_0(5)));
    set_param('projectile/vz','Value',num2str(p_0(6)));
    
    set_param('projectile', 'StopTime', num2str(simTime));

    % run simulation
    sim('projectile');
    
    trajectory = projectilePos;
    
end