classdef JAKAzu7Tree
    properties
        SerialRobot
    end
    methods
        function obj = JAKAzu7Tree()
            d = [-0.1187, 0, 0, -0.1411, -0.1105, -0.1105].';
            a = [0, 0, -0.36, -0.3035, 0, 0].';
            alpha = [pi, -pi/2, pi, pi, -pi/2, pi/2].';
            q_offset = [0, -pi/2, 0, -pi/2, pi, 0].';
            for idx = 1:6
                L(idx) = Link('d',d(idx), 'a', a(idx), 'alpha', alpha(idx),'offset',q_offset(idx),'modified');
            end
            obj.SerialRobot = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6)],'name','JAKAzu7');
        end
    end
end


