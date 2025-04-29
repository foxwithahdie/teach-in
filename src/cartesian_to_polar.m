function [r, theta] = cartesian_to_polar(x, y)
arguments 
    x (:,1) double
    y (:,1) double
end
    r = sqrt(x.^2 + y.^2);
    theta = atan2(y, x);
end