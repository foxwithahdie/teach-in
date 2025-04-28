function [x, y] = polar_to_cartesian(r, theta)
    arguments
        r (:, 1) double
        theta (:, 1) double
    end
    x = r .* cos(theta);
    y = r .* sin(theta);
end