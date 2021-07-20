function [isPrepareDone, q_planning, v_planning] = Sync_Prepare(q_last, v_last, q_target)
step = 80/1000; % step needs to be decide
vMax = pi; aMax = 2*pi;
vSafe = 5; aSafe = 2;
isPrepareDone = 0;

datestr(now)
v_next = (q_target - q_last) / step;
q_planning = q_target;
v_planning = v_next;
check = abs(v_next);
for i = 1:6
    if (check(i) >= vMax)
        q_planning(i) = q_last(i) + vMax * step * sign(v_next(i));
        v_planning(i) = vMax * sign(v_next(i));
    end        
end
datestr(now)
end

