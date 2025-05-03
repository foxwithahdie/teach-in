function Grad = approximate_gradient(costfunc, current_neato_pos, goal)
    step_size = 1e-6;
    delta = zeros(size(current_neato_pos));
    Grad = zeros(size(current_neato_pos));
    for n = 1:length(current_neato_pos)
        %set the increment for the current index
        delta(n) = 1;
        %compute the function at different points near x
        f_minus = costfunc(current_neato_pos-(step_size*delta), goal);
        f_plus = costfunc(current_neato_pos+(step_size*delta), goal);
        %compute the ith element of the gradient
        Grad(n) = (f_plus-f_minus)/(2*step_size);
        %reset delta_x
        delta(n) = 0;
    end
end

