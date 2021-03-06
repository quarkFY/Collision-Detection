function [isConfigInCollision, q_plan, v_plan] = Collision_Detect(q_last, v_last, q_tar)
global JAKAzu7Tree
step = 80/1000; % step needs to be decide
v_max = pi; a_max = 2*pi; q_step_max = pi / 18;
v_safe = 5; a_safe = 2;
isConfigInCollision = 0; isMoveTooFast = 0;

% Motion through collision field
deltaq = q_tar - q_last;
deltaq_max = max(abs(deltaq));
if (deltaq_max >= q_step_max)
    q_plan = q_last;
    v_plan = [0 0 0 0 0 0].';
    isConfigInCollision = -1;
    return
end
% Move too fast
if (deltaq_max >= v_max * step)
    deltaq = deltaq * v_max * step / deltaq_max;
    isMoveTooFast = 1;
end
dq_max = max(abs(deltaq)) / step;
q_predict = q_tar + v_next * dq_max / (2*a_max);
isConfigInCollision = checkCollision(JAKAzu7Tree,q_predict,'Exhaustive','on');
if (isConfigInCollision)
    % BUG FIXED!!!!! Be care of the inverse of speed!!!
    v_decrease = - sign(v_last) * step * a_max;
    v_plan = v_last + v_decrease;
    isSpeedReverse = v_plan.* v_last;
    for idx = 1:6
        if isSpeedReverse(idx) <= 0 
            v_plan(idx) = 0;
        end
    end
    q_plan = q_last + (v_last + v_plan) * step / 2; 
elseif isMoveTooFast == 1
    isConfigInCollision = 2;
    q_plan = q_tar;
    v_plan = v_next;
else
    q_plan = q_tar;
    v_plan = v_next;
end
end


