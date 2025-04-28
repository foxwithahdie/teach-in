function [r, theta] = cartesian_to_polar(x, y)
    r = sqrt(x.^2 + y.^2);
    theta = atan2(y, x);
end