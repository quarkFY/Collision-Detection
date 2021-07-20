%% Load the model
clear all; close all;
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
ConfigZero = [0 pi/2 0 pi/2 0 0]';
%
currConfig = [pi/4 pi/6 -pi*2/5 pi/4 pi/3 0]';
show(robot,currConfig,'visuals','on','collision','off');
%%
for i = 1:robot.NumBodies
    clearCollision(robot.Bodies{i})
end
% show(robot,ConfigZero,'Collisions','on','Visuals','off');
Trans = zeros(4,4,8);
%
r3 = 0.07; h3 = 0.2;
mat = axang2tform([0 0 1 pi/2]); Trans(:,:,3) = eye(4,4) * mat;
collisionObj3 = collisionCylinder(r3,h3); collisionObj3.Pose = Trans(:,:,3);
addCollision(robot.Bodies{3},collisionObj3);
%
x4 = 0.5; y4 = 0.12; z4 = 0.17;
Trans(:,:,4) = [eye(3,3) [x4/2-0.06; 0; -z4/2-0.07]; 0 0 0 1];
collisionObj4 = collisionBox(x4,y4,z4); collisionObj4.Pose = Trans(:,:,4);
addCollision(robot.Bodies{4},collisionObj4);
%
x5 = 0.4; y5 = 0.13; z5 = 0.13;
Trans(:,:,5) = [eye(3,3) [x5/2-0.055; 0; -0.01]; 0 0 0 1];
collisionObj5 = collisionBox(x5,y5,z5); collisionObj5.Pose = Trans(:,:,5);
addCollision(robot.Bodies{5},collisionObj5);
%
r6 = 0.04; h6 = 0.14;
mat = axang2tform([1 0 0 pi/2]); Trans(:,:,6) = [eye(3,3) [0; -h6/2+0.07; 0]; 0 0 0 1]*mat;
collisionObj6 = collisionCylinder(r6,h6); collisionObj6.Pose = Trans(:,:,6);
addCollision(robot.Bodies{6},collisionObj6);
%
r7 = 0.045; h7 = 0.19;
mat = axang2tform([1 0 0 pi/2]); Trans(:,:,7) = [eye(3,3) [0; 0.005; 0]; 0 0 0 1]*mat;
collisionObj7 = collisionCylinder(r7,h7); collisionObj7.Pose = Trans(:,:,7);
addCollision(robot.Bodies{7},collisionObj7);
% Trans(:,:,8) = [eye(3,3) [0; 0; 0]; 0 0 0 1];
% collisionObj8 = collisionCylinder(0.05,0.25);

ConfigZero = [0 pi/2 0 pi/2 0 0]';
show(robot,ConfigZero,'Collisions','on','Visuals','on');
isConfigInCollision = checkCollision(robot,ConfigZero,'Exhaustive','on');
isConfigInCollision
%% 
deg2ard = pi/180;
lim_deg = [-270 -85 -175 -85 -270 -270; 270 265 175 265 270 270];
lim = lim_deg * deg2ard;
vSafe = 5; aSafe = 2;
%% real-time collistion detect
% Define collision-free start & goal configurations
% startConfig = [0 0 pi/4 0 -pi/2 pi/8]';
% goalConfig = [0 pi/2 -pi*3/2 0 -pi/2 pi/8]';
startConfig = [0 pi/3 pi/4 pi/2 pi/2 pi/8]';
goalConfig = [0 pi/2 -pi*3/2 -pi/2 -pi/2 pi/8]';
[q,qd,qdd,T] = trapveltraj([startConfig goalConfig],101,'EndTime',3);
%%
vMax = pi; aMax = 2*pi;
isConfigInCollision = false(101,1);
datestr(now)
qCurr = zeros(6,101);
vCurr = zeros(6,101);
qCurr(:,1) = q(:,1); % q sent to JAKA
vCurr(:,1) = qd(:,1); % v sent to JAKA
msg = zeros(1,101);
isConfigInCollision(1) = checkCollision(robot,qCurr(:,1),'Exhaustive','on');
msg(1) = 0; % Must start from a collision free configration
for i = 1:100
    step = T(i+1) - T(i);
    vNext = (q(:,i+1) - qCurr(:,i)) / step;
    check = max(abs(vNext));
    if (check >= vMax)
        msg(i+1) = -1;
        qCurr(:,i+1) = qCurr(:,i);
        vCurr(:,i+1) = [0 0 0 0 0 0].';
    else
        preConfig = q(:,i+1) + vCurr(:,i)*check / (2*aMax);
        isConfigInCollision(i+1) = checkCollision(robot,preConfig,'Exhaustive','on');
        if (isConfigInCollision(i+1))
            msg(i+1) = 1;
            qCurr(:,i+1) = qCurr(:,i) + vCurr(:,i)*step - (0.5*aMax*step^2).*sign(vCurr(:,i)); 
            vCurr(:,i+1) = (qCurr(:,i+1) - qCurr(:,i)) / step; % de-acceleration with aMax
        else
            msg(i+1) = 0;
            qCurr(:,i+1) = q(:,i+1);
            vCurr(:,i+1) = vNext;
        end
    end
end
datestr(now)

%% visualization
pic=figure('name','Animation');
pic_num = 1;
for i=1:length(qCurr)
        show(robot,qCurr(:,i),'Collisions','on','Visuals','off');
        if (msg(i)==-1)
            text(0,0,0,'Out of Range, No Response','FontSize',14);
        elseif(msg(i)==0)
            text(0,0,0,'Collision Free','FontSize',14);
        else
            text(0,0,0,'Collision Detected, Slow Down','FontSize',14);   
        end
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        if pic_num == 1
            imwrite(I,map,'test.gif','gif', 'Loopcount',inf,'DelayTime',0.02);
        else
            imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.02);
        end
        pic_num = pic_num + 1;
        
end
