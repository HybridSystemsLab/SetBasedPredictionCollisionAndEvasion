close all;

ts = 0.05;
L = 0.4;

x0 = [0.0,0.01,0.0,0.01,pi/2];

u = [2 2 2 2 2 2 2 2 2;
     0 0 0 0 0 0 0 0 0];
 
phi = -1*[0 0 0 0 pi/4 pi/4 pi/4 pi/4 pi/4];

w = (u(1,:)/L).*tan(phi);
u(2,:) = (ts*w)/2;



x = Dubin(x0,u,ts);

scatter(x(1,:),x(3,:))
hold on
scatter(x(1,:),x(4,:))
scatter(x(2,:),x(3,:))
scatter(x(2,:),x(4,:))
axis([-1 1 -1 1])

x(5,:)