%% Load the model
clear all; close all;
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
ConfigZero = [0 pi/2 0 pi/2 0 0]';
%%
% currConfig = [pi/4 pi/6 -pi*2/5 pi/4 pi/3 0]';
show(robot,ConfigZero,'visuals','on','collision','on');
%% Initialization
clearCollision(robot.Bodies{1}); % Base link model cause self-collision
% Joint 1 and Joint 6 will not commit to a collision
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
[q,qd,qdd,T] = trapveltraj([startConfig goalConfig],501,'EndTime',3);
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
            disp(qCurr(:,i+1)-q(:,i+1));
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
