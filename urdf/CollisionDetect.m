%% Load the model
clear all; close all;
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
ConfigZero = [0 pi/2 0 pi/2 0 0]';
%%
show(robot,ConfigZero,'visuals','on','collision','on');
clearCollision(robot.Bodies{1}); % Base link model cause self-collision
%% Collistion detect
% Joint 1 and Joint 6 will not commit to a collision
deg2ard = pi/180;
lim_deg = [-270 -85 -175 -85 -270 -270; 270 265 175 265 270 270];
lim = lim_deg * deg2ard;
step_deg = 30;
step = step_deg * deg2ard;
num_q2 = floor((lim_deg(2,2)-lim_deg(1,2))/step_deg);
num_q3 = floor((lim_deg(2,3)-lim_deg(1,3))/step_deg);
num_q4 = floor((lim_deg(2,4)-lim_deg(1,4))/step_deg);
num_q5 = floor((lim_deg(2,5)-lim_deg(1,5))/step_deg);
isConfigInCollision = false(num_q2,num_q3,num_q4,num_q5);
for q2 = 1 : num_q2
    for q3 = 1 : num_q3
        for q4 = 1 : num_q4
            for q5 = 1 : num_q5
                curr = [0 q2*step+lim(1,2) q3*step+lim(1,3) q4*step+lim(1,4) q5*step+lim(1,5) 0]';
                ConfigCurr = curr + ConfigZero;
                isColliding = checkCollision(robot,ConfigCurr,'Exhaustive','on');
                isConfigInCollision(q2,q3,q4,q5) = isColliding;
            end
        end
    end
end




% for i = 1:length(q)
%     isColliding = checkCollision(robot,q(:,i),'Exhaustive','on');
%     isConfigInCollision(i) = isColliding;
% end

%% visualization
% pic=figure('name','Animation');
% pic_num = 1;
% for i=1:length(q)
%         show(robot,q(:,i),'Collisions','on','Visuals','off');
%         if (isConfigInCollision(i))
%             text(0,0,0,'Collision Detected','FontSize',14);
%         else
%             text(0,0,0,'Collision Free','FontSize',14);
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
%%
% Define collision-free start & goal configurations
% startConfig = [-pi/4 pi 3*pi/2 0 -pi/2 pi/8]';
% goalConfig = [-pi/4 pi 3*pi/4 0 -pi/2 pi/8]';
% q = trapveltraj([startConfig goalConfig],100,'EndTime',3);
save('Collision','isConfigInCollision','');