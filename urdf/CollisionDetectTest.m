%% Load the model
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
show(robot,'visuals','on','collision','on');