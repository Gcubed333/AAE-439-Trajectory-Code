function [val, terminate, direction] = touchdownEvent(t,y)
    val = y(4);
    terminate = 1;
    direction = -1;
end