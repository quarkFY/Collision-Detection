%% Load the model
clear all; close all;
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
ConfigZero = [0 pi/2 0 pi/2 0 0]';
%
currConfig = [pi/4 pi/6 -pi*2/5 pi/4 pi/3 0]';
show(robot,currConfig,'visuals','on','collision','off');
%% Create mesh from vertices
cb2 = robot.Bodies{3};
for i = 1:robot.NumBodies
    clearCollision(robot.Bodies{i})
end
gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p2.stl');
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

gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p3.stl');
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

gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p4.stl');
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj4 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat4 = trvec2tform([0.303 0 -0.013])*rpy2tr(-pi/2, 0, -pi/2);
collisionObj4.Pose = mat4;
addCollision(robot.Bodies{5},collisionObj4);

gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p5.stl');
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj5 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat5 = trvec2tform([0 0 0])*rpy2tr(pi/2, 0, -pi/2);
collisionObj5.Pose = mat5;
addCollision(robot.Bodies{6},collisionObj5);

gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p6.stl');
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
gm = stlread('D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7\SimplifiedMesh\p7.stl');
x = gm.Points(:,1); y = gm.Points(:,2); z = gm.Points(:,3);
[k, av] = convhull(x,y,z);
v = [k(:,1);k(:,2);k(:,3)];
Vertices = gm.Points(unique(v),:);
collisionObj7 = collisionMesh(Vertices);
% 0.303 0 -0.013
mat7 = trvec2tform([0 0 -0.0335])*rpy2tr(0, -pi/2, 0);
collisionObj7.Pose = mat7;
addCollision(robot.Bodies{8},collisionObj7);

ConfigZero = [0 pi/2 0 pi/2 0 0]';
figure(2)
show(robot,ConfigZero,'Collisions','on','Visuals','on');
isConfigInCollision = checkCollision(robot,ConfigZero,'Exhaustive','on');
isConfigInCollision
%% 
% deg2ard = pi/180;
% lim_deg = [-270 -85 -175 -85 -270 -270; 270 265 175 265 270 270];
% lim = lim_deg * deg2ard;
% vSafe = 5; aSafe = 2;
% %% real-time collistion detect
% % Define collision-free start & goal configurations
% % startConfig = [0 0 pi/4 0 -pi/2 pi/8]';
% % goalConfig = [0 pi/2 -pi*3/2 0 -pi/2 pi/8]';
% startConfig = [0 pi/3 pi/4 pi/2 pi/2 pi/8]';
% goalConfig = [0 pi/2 -pi*3/2 -pi/2 -pi/2 pi/8]';
% [q,qd,qdd,T] = trapveltraj([startConfig goalConfig],1001,'EndTime',3);
% %%
% vMax = pi; aMax = 2*pi;
% isConfigInCollision = false(1001,1);
% datestr(now,'mmmm dd,yyyy HH:MM:SS.FFF')
% qCurr = zeros(6,1001);
% vCurr = zeros(6,1001);
% qCurr(:,1) = q(:,1); % q sent to JAKA
% vCurr(:,1) = qd(:,1); % v sent to JAKA
% msg = zeros(1,1001);
% isConfigInCollision(1) = checkCollision(robot,qCurr(:,1),'Exhaustive','on');
% msg(1) = 0; % Must start from a collision free configration
% for i = 1:1000
%     step = T(i+1) - T(i);
%     vNext = (q(:,i+1) - qCurr(:,i)) / step;
%     check = max(abs(vNext));
%     if (check >= vMax)
%         msg(i+1) = -1;
%         qCurr(:,i+1) = qCurr(:,i);
%         vCurr(:,i+1) = [0 0 0 0 0 0].';
%     else
%         preConfig = q(:,i+1) + vCurr(:,i)*check / (2*aMax);
%         isConfigInCollision(i+1) = checkCollision(robot,preConfig,'Exhaustive','on');
%         if (isConfigInCollision(i+1))
%             msg(i+1) = 1;
%             qCurr(:,i+1) = qCurr(:,i) + vCurr(:,i)*step - (0.5*aMax*step^2).*sign(vCurr(:,i)); 
%             vCurr(:,i+1) = (qCurr(:,i+1) - qCurr(:,i)) / step; % de-acceleration with aMax
%         else
%             msg(i+1) = 0;
%             qCurr(:,i+1) = q(:,i+1);
%             vCurr(:,i+1) = vNext;
%         end
%     end
% end
% datestr(now,'mmmm dd,yyyy HH:MM:SS.FFF')

%% visualization
% pic=figure('name','Animation');
% pic_num = 1;
% for i=1:length(qCurr)
%         show(robot,qCurr(:,i),'Collisions','on','Visuals','off');
%         if (msg(i)==-1)
%             text(0,0,0,'Out of Range, No Response','FontSize',14);
%         elseif(msg(i)==0)
%             text(0,0,0,'Collision Free','FontSize',14);
%         else
%             text(0,0,0,'Collision Detected, Slow Down','FontSize',14);   
%         end
%         F=getframe(gcf);
%         I=frame2im(F);
%         [I,map]=rgb2ind(I,256);
%         if pic_num == 1
%             imwrite(I,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0.02);
%         else
%             imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.02);
%         end
%         pic_num = pic_num + 1;
%         
% end
