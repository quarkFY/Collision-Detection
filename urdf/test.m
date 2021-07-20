% filepath = 'D:\00 SJTU Master Course\Spring 2021\Mechatronics\Collision Detect\Zu7';
% global robot
% robot = JAKAzu7TreeModel(filepath);
% %% 
% [outputArg1,outputArg2] = test2(1,2);
rbt = JAKAzu7Tree().SerialRobot;
q_tar = [0 0 0 0 0 0];
%%
figure(1)
view(3)
rbt.plot(q_tar);