% graphing data

close all;

% safety radius
r = 0.2422/2;

%% projectile data 
projFile = fopen('projData.txt', 'r');

data = fscanf(projFile, '%f %f %f\n');

xp = zeros(1,length(data)/3);
yp = zeros(1,length(data)/3);
zp = zeros(1,length(data)/3);


i = 1;
j = 1;
while(i < length(data))
   
    xp(j) = data(i);
    yp(j) = data(i+1);
    zp(j) = data(i+2);
    
    i = i+3;
    j = j+1;
end

%% quad data 
quadFile = fopen('quadData.txt', 'r');

data = fscanf(quadFile, '%f %f %f\n');

xq = zeros(1,length(data)/3);
yq = zeros(1,length(data)/3);
zq = zeros(1,length(data)/3);


i = 1;
j = 1;
while(i < length(data))
   
    xq(j) = data(i);
    yq(j) = data(i+1);
    zq(j) = data(i+2);
    
    i = i+3;
    j = j+1;
end

%% time
timeFile = fopen('timeData.txt', 'r');

data = fscanf(timeFile, '%f\n');

time = data;

%% target distance
tarDistFile = fopen('targetDistance.txt','r');

data = fscanf(tarDistFile, '%f\n');
tDist = data;

%% projectile distance
projDistFile = fopen('projDistance.txt','r');

data = fscanf(projDistFile, '%f\n');
pDist = data;

%% planned trajectories

trajplanFile0 = fopen('trajdata0.txt','r');
trajplanFile1 = fopen('trajdata1.txt','r');
trajplanFile2 = fopen('trajdata2.txt','r');
trajplanFile3 = fopen('trajdata3.txt','r');
trajplanFile4 = fopen('trajdata4.txt','r');
trajplanFile5 = fopen('trajdata5.txt','r');
trajplanFile6 = fopen('trajdata6.txt','r');
trajplanFile7 = fopen('trajdata7.txt','r');


data0 = fscanf(trajplanFile0, '%f %f %f \n');
data1 = fscanf(trajplanFile1, '%f %f %f \n');
data2 = fscanf(trajplanFile2, '%f %f %f \n');
data3 = fscanf(trajplanFile3, '%f %f %f \n');
data4 = fscanf(trajplanFile4, '%f %f %f \n');
data5 = fscanf(trajplanFile5, '%f %f %f \n');
data6 = fscanf(trajplanFile6, '%f %f %f \n');
data7 = fscanf(trajplanFile7, '%f %f %f \n');

tx0 = zeros(1,length(data0)/3);
ty0 = zeros(1,length(data0)/3);
tz0 = zeros(1,length(data0)/3);

tx1 = zeros(1,length(data0)/3);
ty1 = zeros(1,length(data0)/3);
tz1 = zeros(1,length(data0)/3);

tx2 = zeros(1,length(data0)/3);
ty2 = zeros(1,length(data0)/3);
tz2 = zeros(1,length(data0)/3);

tx3 = zeros(1,length(data0)/3);
ty3 = zeros(1,length(data0)/3);
tz3 = zeros(1,length(data0)/3);

tx4 = zeros(1,length(data0)/3);
ty4 = zeros(1,length(data0)/3);
tz4 = zeros(1,length(data0)/3);

tx5 = zeros(1,length(data0)/3);
ty5 = zeros(1,length(data0)/3);
tz5 = zeros(1,length(data0)/3);

tx6 = zeros(1,length(data0)/3);
ty6 = zeros(1,length(data0)/3);
tz6 = zeros(1,length(data0)/3);

tx7 = zeros(1,length(data0)/3);
ty7 = zeros(1,length(data0)/3);
tz7 = zeros(1,length(data0)/3);


i = 1;
j = 1;
while(i < length(data0))
   
    tx0(j) = data0(i);
    ty0(j) = data0(i+1);
    tz0(j) = data0(i+2);
    
    tx1(j) = data1(i);
    ty1(j) = data1(i+1);
    tz1(j) = data1(i+2);
    
    tx2(j) = data2(i);
    ty2(j) = data2(i+1);
    tz2(j) = data2(i+2);
    
    tx3(j) = data3(i);
    ty3(j) = data3(i+1);
    tz3(j) = data3(i+2);
    
    tx4(j) = data4(i);
    ty4(j) = data4(i+1);
    tz4(j) = data4(i+2);
    
    tx5(j) = data5(i);
    ty5(j) = data5(i+1);
    tz5(j) = data5(i+2);
    
    tx6(j) = data6(i);
    ty6(j) = data6(i+1);
    tz6(j) = data6(i+2);
    
    tx7(j) = data7(i);
    ty7(j) = data7(i+1);
    tz7(j) = data7(i+2);
    
    
    i = i+3;
    j = j+1;
end

%% projectile predict data 
projpredFile = fopen('pPredData.txt', 'r');



data = fscanf(projpredFile, '%f %f %f\n');

xpp = zeros(1,length(data)/3);
ypp = zeros(1,length(data)/3);
zpp = zeros(1,length(data)/3);


i = 1;
j = 1;
while(i < length(data))
   
    xpp(j) = data(i);
    ypp(j) = data(i+1);
    zpp(j) = data(i+2);
    
    i = i+3;
    j = j+1;
end

%% graphing


figure(1)

% projectile
scatter3(xp, yp, zp,'b','*')
hold on

% quad
scatter3(xq, yq, zq,'filled','r')
hold on

% target
scatter3(0.0,0.6,0.4,100,'filled','g')
hold on

%{
x = [2 2 -2 -2];
y = [-2 2 2 -2];
z = [0.4 0.4 0.4 0.4];
p  = patch(x,y,z,'g');
set(p,'FaceAlpha',0.3);
%}


text(xq(1)+0.3, yq(1), zq(1),'t=0')
%text(xq(10)+0.4, yq(10), zq(10),'t=10')
text(xq(20)+0.35, yq(20), zq(20),'t=20')
%text(xq(30), yq(30)-0.1, zq(30),'t=30')
text(xq(40), yq(40)-0.1, zq(40),'t=40')
%text(xq(50), yq(50)-0.1, zq(50),'t=50')


text(xp(1)-0.1, yp(1), zp(1)+0.2,'t=0')
%text(xp(10)-0.1, yp(10), zp(10)+0.2,'t=10')
text(xp(20)-0.05, yp(20), zp(20)+0.2,'t=20')
%text(xp(30), yp(30), zp(30)+0.2,'t=30')
text(xp(40), yp(40)-0.1, zp(40),'t=40')
%text(xp(50), yp(50)-0.1, zp(50),'t=50')


% boundary
%plot3([-1.6 -1.6 1.6 1.6 -1.6], [-0.4 1.6 1.6 -0.4 -0.4], [0.4 0.4 0.4 0.4 0.4])
hold on

xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis([-2 2 -2 2 -5 7])

legend('Projectile','Quadcopter','Goal','Location','northeast');

%{
figure(2)

subplot(1,3,1)
plot(time, xq)
hold on 
plot(time, xp)
hold on
plot(time,zeros(length(time)),'--')
xlabel('time - seconds')
ylabel('x position - meters')
legend('Quadcopter','Projectile')
grid on

subplot(1,3,2)
plot(time,yq)
hold on 
plot(time, yp)
hold on
plot(time,0.6*ones(length(time)),'--')
title('Quadcopter position and projectile position over time')
xlabel('time - seconds')
ylabel('y position - meters')
grid on

subplot(1,3,3)
plot(time,zq)
hold on 
plot(time, zp)
xlabel('time - seconds')
ylabel('z position - meters')
grid on
%}

figure(3)
subplot(1,2,1)
plot(time,tDist);
hold on
title('Distance to Target')
xlabel('Time [s]')
ylabel('Distance [m]')
axis([0 time(end) 0 1.5])

subplot(1,2,2)
plot(time,pDist)
hold on
plot(time,0.2422*ones(length(time)),'Color',[1,0,0],'LineStyle','--')
axis([0 time(end) 0 7])
title('Distance to Projectile')
xlabel('Time [s]')
ylabel('Distance [m]')

%{
figure(4)
scatter3(tx0,ty0,tz0,'g');
hold on
scatter3(tx3,ty3,tz3,'r');
scatter3(xpp, ypp, zpp,'b');

scatter3(tx1,ty1,tz1,'g');
scatter3(tx2,ty2,tz2,'g');
scatter3(tx4,ty4,tz4,'g');
scatter3(tx5,ty5,tz5,'g');
scatter3(tx6,ty6,tz6,'g');
scatter3(tx7,ty7,tz7,'g');

scatter3(xpp, ypp, zpp,'b')

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')

legend('Safe trajectories','Unsafe trajectories', 'Projectile')


figure(5)

% projectile
scatter3(xp, yp, zp,'b','*')
hold on

% quad
%scatter3(xq, yq, zq,'r','+')

% target
scatter3(0.0,0.6,0.4,100,'filled','g')



scatter3(xpp, ypp, zpp,'b','.');
scatter3(tx0,ty0,tz0,'g','.');
scatter3(tx1,ty1,tz1,'g','.');
scatter3(tx2,ty2,tz2,'g','.');
scatter3(tx3,ty3,tz3,'r','.');
scatter3(tx4,ty4,tz4,'g','.');
scatter3(tx5,ty5,tz5,'g','.');
scatter3(tx6,ty6,tz6,'g','.');
scatter3(tx7,ty7,tz7,'g','.');


[x y z] = sphere(128)
tsafety = surf(r*x+tx3(end), r*y+ty3(end), r*z+tz3(end)); 
set(tsafety, 'FaceColor', [1 0 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')
tsafety_1 = surf(r*x+tx3(end), r*y+ty3(end-1), r*z+tz3(end-1)); 
set(tsafety_1, 'FaceColor', [1 0 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')

[x_c,y_c,z_c] = cylinder2P(r,20,[tx3(end),ty3(end),tz3(end)], [tx3(end-1),ty3(end-1),tz3(end-1)]);
tsafety_2 = surf(x_c,y_c,z_c);
set(tsafety_2, 'FaceColor', [1 0 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')



psafety_0 = surf(r*x+xpp(end), r*y+ypp(end), r*z+zpp(end)); 
set(psafety_0, 'FaceColor', [0 1 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')
psafety_1 = surf(r*x+xpp(end), r*y+ypp(end-1), r*z+zpp(end-1)); 
set(psafety_1, 'FaceColor', [0 1 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')

[x,y,z] = cylinder2P(r,20,[xpp(end),ypp(end),zpp(end)], [xpp(end-1),ypp(end-1),zpp(end-1)]);
psafety_2 = surf(x,y,z);
set(psafety_2, 'FaceColor', [0 1 0],'FaceAlpha', 0.2,'FaceLighting','gouraud','EdgeColor','none')

xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
%}
