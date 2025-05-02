function params = params(beta, gamma, max_iter, min_gradient)
    params = struct();
    params.beta = beta;
    params.gamma = gamma;
    params.max_iter = max_iter;
    params.min_gradient = min_gradient;
end