function Vopt = run_gradient_descent(fun, current_neato_pos_x, current_neato_pos_y, params, dist_x, dist_y)
    beta = params.beta;
    gamma = params.gamma;
    max_iter = params.max_iter;
    min_gradient = params.min_gradient;
    alpha = 1; 
    neato_pos_x_temp = current_neato_pos_x;
    neato_pos_y_temp = current_neato_pos_y;
    dist_x_temp = dist_x;
    dist_y_temp = dist_y;
    n = 0;
    
    %evaluate gradient and function for first time
    G = approximate_gradient(costfunc,neato_pos_x_temp, neato_pos_y_temp, dist_x_temp, dist_y_temp);
    F = costfunc(neato_pos_x_temp, neato_pos_y_temp);
    %iterate until either gradient is sufficiently small or we hit the max iteration limit
    while n<max_iter && norm(G)>min_gradient
        %compute the scare of the norm of the gradient
        NG2 = norm(G)^2;
        %run line search algorithm to find alpha
        while fun([neato_pos_x_temp, neato_pos_y_temp]-alpha*G)<F-beta*alpha*NG2
            alpha = alpha/gamma;
        end
        
        while fun([neato_pos_x_temp, neato_pos_y_temp]-alpha*G)>F-beta*alpha*NG2
            alpha = alpha*gamma;
        end
        %once alpha has been found, update guess
        [neato_pos_x_temp, neato_pos_y_temp] = [neato_pos_x_temp, neato_pos_y_temp]-alpha*G;
        %evaluate gradient and function at new value of V
        G = approximate_gradient(fun,[neato_pos_x_temp, neato_pos_y_temp]);
        F = fun([neato_pos_x_temp, neato_pos_y_temp]);
        
        n = n+1;
    end
    %return final value of V as our numerical solution
    Vopt = V;
end