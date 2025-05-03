function Vopt = run_gradient_descent(costfunc, current_neato_pos, goal, params)
    beta = params.beta;
    gamma = params.gamma;
    max_iter = params.max_iter;
    min_gradient = params.min_gradient;
    alpha = 1; 
    neato_pos_x_temp = current_neato_pos(1);
    neato_pos_y_temp = current_neato_pos(2);
    dist_x_temp = abs(goal(1)-current_neato_pos(1));
    dist_y_temp = abs(goal(2)-current_neato_pos(2));
    n = 0;
    X = [neato_pos_x_temp, neato_pos_y_temp];
    Z = [dist_x_temp, dist_y_temp];
    %evaluate gradient and function for first time
    G = approximate_gradient(costfunc,X,goal);
    F = costfunc(X, goal);

    %iterate until either gradient is sufficiently small or we hit the max iteration limit
    while n<max_iter && norm(G)>min_gradient
        %compute the scare of the norm of the gradient
        NG2 = norm(G)^2;
        %run line search algorithm to find alpha
        while costfunc(X-alpha*G,goal)<F-beta*alpha*NG2
            alpha = alpha/gamma;
        end
        
        while costfunc(X-alpha*G, goal)>F-beta*alpha*NG2
            alpha = alpha*gamma;
        end
        %once alpha has been found, update guess
        X=X-alpha*G;
        %evaluate gradient and function at new value of V
        G = approximate_gradient(costfunc,X, goal);
        F = costfunc(X, goal);
        
        n = n+1;
    end
    %return final value of V as our numerical solution
    Vopt = X;
end