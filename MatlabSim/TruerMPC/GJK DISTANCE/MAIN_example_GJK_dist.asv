
% Example script for GJK function
%   Animates two objects on a collision course and terminates animation
%   when they hit each other. Loads vertex and face data from
%   SampleShapeData.m. See the comments of GJK.m for more information
%
%   Most of this script just sets up the animation and transformations of
%   the shapes. The only key line is:
%   collisionFlag = GJK(S1Obj,S2Obj,iterationsAllowed)
%
%   Matthew Sheen, 2016
%
%
%%%%%%%%%%%%%%%%%%%%% MODIFICTATION NOTICE: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script test has been modified and renamed by PHILIPPE LEBEL 
% April 2017
%
% The author of the original script is Matthew Sheen
% published on
% https://www.mathworks.com/matlabcentral/fileexchange/57907-fast-3d-collision-detection-gjk-algorithm
% 
%
%I (Philippe Lebel) modified it to accommodate tests for my implmenetation of GJK distance
%algorithm. The commented section at the end of the code allows to see the
%minkowski difference and the verticies member of the simplex evolving
%durring the simulation.
%
%The function I wrote is [dist,pts,G,H]=GJK_dist(S1Obj,S2Obj);
% I only take credit for this function.
% I also modified some sections of the other matlab files to accommodate my
% new function.
% I want to personnally thank Matthew Sheen for his work. This allowed me
% to understand a lot concerning the implementation of such algorithm.
%
% The algorithm works in the vast majority of cases, but some times, in
% close proximity, the closest distance points are a little off. There is
% work to be done concerning this bug. Feel free to contribute if you find
% something.




clc;clear all;close all

%How many iterations to allow for collision detection.
iterationsAllowed = 6;

% Make a figure
fig=figure('units','normalized','outerposition',[0 0 1 1]);
set(fig, 'renderer', 'OpenGL');

% fig = figure;
hold on

% Load sample vertex and face data for two convex polyhedra
% SampleShapeData;
[V1,F1]=platonic_solid(5,1);
[V2,F2]=platonic_solid(3,1);
% V2=[0 0 0];
% F2=[1 1 1];
% Make shape 1
S1.Vertices = V1;
S1.Faces = F1;
S1.FaceVertexCData = jet(size(V1,1));
S1.FaceColor = 'interp';
S1Obj = patch(S1);

% Make shape 2
S2.Vertices = V2;
S2.Faces = F2;
S2.FaceVertexCData = jet(size(V2,1));
S2.FaceColor = 'interp';
S2Obj = patch(S2);

hold off
axis equal
axis([-5 5 -5 5 -5 5])
fig.Children.Visible = 'off'; % Turn off the axis for more pleasant viewing.
fig.Color = [1 1 1];
rotate3d on;

%Move them through space arbitrarily.
S1Coords = S1Obj.Vertices;
S2Coords = S2Obj.Vertices;

S1Rot = eye(3,3); % Accumulate angle changes

% Make a random rotation matix to rotate shape 1 by every step
S1Angs = 0.1*rand(3,1); % Euler angles
% S1Angs=[0 0 0.1]';
sang1 = sin(S1Angs);
cang1 = cos(S1Angs);
cx = cang1(1); cy = cang1(2); cz = cang1(3);
sx = sang1(1); sy = sang1(2);  sz = sang1(3);

S1RotDiff = ...
    [          cy*cz,          cy*sz,            -sy
    sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx
    sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx];

S2Rot = eye(3,3);

% Make a random rotation matix to rotate shape 2 by every step
S2Angs = 0.1*rand(3,1); % Euler angles
% S2Angs=[0 0 0]';
sang2 = sin(S2Angs);
cang2 = cos(S2Angs);
cx = cang2(1); cy = cang2(2); cz = cang2(3);
sx = sang2(1); sy = sang2(2); sz = sang2(3);

S2RotDiff = ...
    [          cy*cz,          cy*sz,            -sy
    sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx
    sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx];


% Animation loop. Terminates on collision.
n=1;
dist=-1000;
for i = 2:-0.005:0.02;
%     if n==60
%         break
%     end
    S1Rot = S1RotDiff*S1Rot;
    S2Rot = S2RotDiff*S2Rot;
    
    S1Obj.Vertices = (S1Rot*S1Coords')' + i;
    S2Obj.Vertices = (S2Rot*S2Coords')' + -i;
    [S,D]=minksum(S1Obj.Vertices,-S2Obj.Vertices);
    stock.s{n}=S;
    stock.s1{n}=S1Obj;
    stock.s2{n}=S2Obj;
    % Do collision detection
    %collisionFlag = GJK(S1Obj,S2Obj,iterationsAllowed);
    
    [a,b,c,d,collisionFlag] = GJK_mod(S1Obj,S2Obj,iterationsAllowed);
    tic
    [dist,pts,G,H]=GJK_dist(S1Obj,S2Obj);
    toc;
%     stock.pts{n}=pts;
    try delete(hhh)
    end
    hold on
    hhh(1)=plot3(G(1),G(2),G(3),'Ob','Linewidth',3);
    hold on
    hhh(2)=plot3(H(1),H(2),H(3),'Ob','Linewidth',3);
    hold on
    hhh(3)=line([G(1) H(1)],[G(2) H(2)],[G(3) H(3)]);
%     [a,b, c,dist]=EPA(a,b,c,d);
    stock.pts{n}=pts;
    dist_acc(n)=dist;
%     [q,r]=max(abs(diff(dist_acc)))
%     if max(abs(diff(dist_acc)))>0.1
%         break
%     end
    n=n+1;
    drawnow;
    
    if collisionFlag
        t = text(3,3,3,'Collision!','FontSize',30);
         break;
    end
    
end
hold off
figure
plot(dist_acc);
hold off
dist_acc(end)
% figure
% view(3)
% for i=1:3:length(stock.s)
%     a=stock.s{i};
%     pts=stock.pts{i};
%     pt_x=a(:,1);
%     pt_y=a(:,2);
%     pt_z=a(:,3);
%     T = delaunay(pt_x,pt_y,pt_z);
%     X=[pt_x,pt_y,pt_z];
%     try delete(h1)
%     end
%     try delete(h2)
%     end
%     h1=tetramesh(T,X,'faceColor','b','EdgeColor',[0 0 0],'FaceAlpha',0.1);
%     hold on
%     for j=1:3
%         h2(j)=plot3(pts(1,j),pts(2,j),pts(3,j),'Or','LineWidth',3);
%         hold on
%         d=cross(pts(:,2)-pts(:,1),pts(:,3)-pts(:,1));
%         if d'*pts(:,1)>0
%             d=-d;
%         end
%         line([pts(1,1) pts(1,1)+d(1)],[pts(2,1) pts(2,1)+d(2)],[pts(3,1) pts(3,1)+d(3)])
%         hold on
%     end
%     hold off
%     
%     axis([-1 10 -1 10 -1 10]);
%     drawnow;
%     pause(0.05)
% end
% hold on