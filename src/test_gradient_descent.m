goal = [4, 5];
neato_pos = [0.1234, 0.1234];

costfunc_wrap = @(curr_neato_pos, goal_) costfunc(curr_neato_pos, goal_);

params_struct = params(0.5, 0.9, 500, 1e-4);

V_optimal = run_gradient_descent(costfunc_wrap, neato_pos, goal, params_struct);

V_optimal

