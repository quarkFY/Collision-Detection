gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p2.stl');
gm2 = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p22.stl');

% figure(1)
% hold on
% pdegplot(gm);
% hold off
x = gm.Points(:,1);
y = gm.Points(:,2);
z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
figure(2)
hold on
% plot3(x,y,z)
% plot3(x(k(:,1)),y(k(:,2)),z(k(:,3)))
trisurf(k,x,y,z,'FaceColor','cyan')
axis equal
hold off
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
Vertices2 = gm2.Points;
MSH = collisionMesh(Vertices);
MSH2 = collisionMesh(Vertices2);
figure(3)
show(MSH);
hold on 
show(MSH2);
%%