function [outputArg1,outputArg2] = test2(inputArg1,inputArg2)
global robot
currConfig = [pi/4 pi/6 -pi*2/5 pi/4 pi/3 0]';
show(robot,currConfig,'visuals','on','collision','on');
outputArg1 = inputArg1;
outputArg2 = inputArg2;
end

