function [isConfigInCollision, q_plan, v_plan] = Collision_Detect(q_last, v_last, q_tar)
step = 80/1000; % step needs to be decide
vMax = pi; aMax = 2*pi;
vSafe = 5; aSafe = 2;
isConfigInCollision = 0;

% datestr(now)
v_next = (q_tar - q_last) / step;
check = max(abs(v_next));
if (check >= vMax)
    q_plan = q_last;
    v_plan = [0 0 0 0 0 0].';
else
    q_predict = q_tar + v_next * check / (2*aMax);
    isConfigInCollision = checkCollision(robot,q_predict,'Exhaustive','on');
    if (isConfigInCollision)
        % BUG FIXED!!!!! Be care of the inverse of speed!!!
        v_decrease = - sign(v_last) * step * aMax;
        v_plan = v_last + v_decrease;
        isSpeedReverse = v_plan.* v_last;
        for idx = 1:6
            if isSpeedReverse(idx) <= 0 
                v_plan(idx) = 0;
            end
        end
        q_plan = q_last + (v_last + v_plan) * step / 2; 
    else
        q_plan = q_tar;
        v_plan = v_next;
    end
end

% datestr(now)
end

