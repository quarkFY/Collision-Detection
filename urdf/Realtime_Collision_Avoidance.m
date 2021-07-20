%% Load the model
clear all; close all;
robot = importrobot('Zu7.urdf');
robot.DataFormat = 'column';
%% Initialization
% Base link model is attached to Link 1, robot links start from Link 3,
% Collision detect algorithm check links not directly connected only.
% The collision mesh of Link 1 cause a self-collision, so removed.
% Surroundings are added manully, as simple geometris. $ NOT YET $
% Gripper model are added manully as a whole with Link 6. $ NOT YET $
% Revolusion of Joint 1 and Joint 6 will not commit to a collision.
deg2ard = pi/180;
clearCollision(robot.Bodies{1}); 
ConfigZero = [0 pi/2 0 pi/2 0 0]';
lim_deg = [-270 -85 -175 -85 -270 -270; 270 265 175 265 270 270]; 
lim = lim_deg * deg2ard;
%% Open TCP server & client
t_sensor = tcpserver("localhost", 888888, "ConnectionChangedFcn", @connectionFcn);
pause(0.5);
t_zu7 = tcpclient('192.168.1.1', 888888, "Timeout", 20, "ConnectTimeout", 30);
pause(0.5);
%% real-time collistion detect
% receive data from vs
msg_recv_bytes = 24;
cmd_resp_bytes = 24;
isConfigInCollision = 0;
q_last = ConfigZero;
v_last = [0 0 0 0 0 0].';
q_planning = ConfigZero;
while (1)
    while (t_sensor.NumBytesAvailable ~= msg_recv_bytes)
    end
    msg_recv = read(t_sensor,t_sensor.NumBytesAvailable,"double");
    msg_recv_head = msg_recv(1);
    q_target = msg_recv(2:7);
    if (msg_recv_head == 0)
        % send data to zu7 robot
        [isConfigInCollision, q_planning, v_planning] = Collision_Detect(q_last, v_last, q_target);
        cmd_send = q_planning;
        write(t_zu7,cmd_send,"double");
        q_last = q_planning;
        v_last = v_planning;
    elseif (msg_recv_head == 1) 
        % Initialization. 
        % Go to the pre-defined config directly. 
        % Do notcheck for collision.
        cmd_send = q_target;
        write(t_zu7,cmd_send,"double");
        q_last = q_target;
        elseif (msg_recv_head == 2)
            % Preparation.
            % Approximate the config of sensor.
            % Do not check for collison.
            [isPrepareDone, q_planning, v_planning] = Sync_Prepare(q_last, v_last, q_target);
            
    end
end
