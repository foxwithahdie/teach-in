function params_struct = params(beta, gamma, max_iter, min_gradient)
    params_struct = struct();
    params_struct.beta = beta;
    params_struct.gamma = gamma;
    params_struct.max_iter = max_iter;
    params_struct.min_gradient = min_gradient;
end