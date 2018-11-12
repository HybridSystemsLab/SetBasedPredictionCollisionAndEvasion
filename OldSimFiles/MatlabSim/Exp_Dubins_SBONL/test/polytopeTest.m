xPolytope = [-0.0350   -0.0150   -0.0150   -0.0350   -0.0350   -0.0150   -0.0150   -0.0350;
   -0.0350   -0.0350   -0.0150   -0.0150   -0.0350   -0.0350   -0.0150   -0.0150;
   -0.0137   -0.0137   -0.0137   -0.0137    0.0063    0.0063    0.0063    0.0063];

target = [-0.0200 -0.0200; -0.0200 -0.0200; -0.0200 -0.0200];
%target = [-0.025; 0.000; 0.000];

PolytopeMinDist(xPolytope,target)

figure()
scatter3(xPolytope(1,:), xPolytope(2,:), xPolytope(3,:))
hold on
scatter3(target(1),target(2),target(3))
