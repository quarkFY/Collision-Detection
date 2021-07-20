function robot = JAKAzu7TreeModel(filepath)
%% Load the model
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
% ConfigZero = [0 pi/2 0 pi/2 0 0]';
% show(robot,currConfig,'visuals','on','collision','off');
%% Create mesh from vertices
for i = 1:robot.NumBodies
    clearCollision(robot.Bodies{i})
end
filepath2 = [filepath,'\SimplifiedMesh\p2.stl'];
gm = stlread(filepath2);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj2 = collisionMesh(Vertices);
% 1.57079632679 3.14159265358 1.57079632679" xyz="0 0 0.005 
% mat = axang2tform([1 0 0 pi/2])*;
mat2 = trvec2tform([0 0 0])* rpy2tr(0, 0, 0);
collisionObj2.Pose = mat2;
addCollision(robot.Bodies{3},collisionObj2);

filepath3 = [filepath,'\SimplifiedMesh\p3.stl'];
gm = stlread(filepath3);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj3 = collisionMesh(Vertices);
% 1.57079632679 3.14159265358 1.57079632679" xyz="0 0 0.005 
% mat = axang2tform([1 0 0 pi/2])*;
mat3 = trvec2tform([0 0 0.005])*rpy2tr(pi/2, pi, pi/2);
collisionObj3.Pose = mat3;
addCollision(robot.Bodies{4},collisionObj3);

filepath4 = [filepath,'\SimplifiedMesh\p4.stl'];
gm = stlread(filepath4);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj4 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat4 = trvec2tform([0.303 0 -0.013])*rpy2tr(-pi/2, 0, -pi/2);
collisionObj4.Pose = mat4;
addCollision(robot.Bodies{5},collisionObj4);

filepath5 = [filepath,'\SimplifiedMesh\p5.stl'];
gm = stlread(filepath5);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj5 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat5 = trvec2tform([0 0 0])*rpy2tr(pi/2, 0, -pi/2);
collisionObj5.Pose = mat5;
addCollision(robot.Bodies{6},collisionObj5);

filepath6 = [filepath,'\SimplifiedMesh\p6.stl'];
gm = stlread(filepath6);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj6 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat6 = trvec2tform([0 0 0])*rpy2tr(pi, 0, -pi/2);
collisionObj6.Pose = mat6;
addCollision(robot.Bodies{7},collisionObj6);
% 
filepath7 = [filepath,'\SimplifiedMesh\p7.stl'];
gm = stlread(filepath7);
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj7 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat7 = trvec2tform([0 0 -0.0335])*rpy2tr(0, -pi/2, 0);
collisionObj7.Pose = mat7;
addCollision(robot.Bodies{8},collisionObj7);

% ConfigZero = [0 pi/2 0 pi/2 0 0]';
% figure(2)
% show(robot,ConfigZero,'Collisions','on','Visuals','on');
% isConfigInCollision = checkCollision(robot,ConfigZero,'Exhaustive','on');
end

