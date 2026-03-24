function results = simulate_multipath_impact(params, env)
% SIMULATE_MULTIPATH_IMPACT  Evaluate impact of multipath on localization
%
% Reproduces Fig. 5:
%   (b) Localization error vs. number of rays (1 to 5)

    p_BS_Tx = env.p_BS_Tx;
    p_BS_Rx = env.p_BS_Rx;
    p_RV_actual = env.p_RV_actual;
    theta_RV_actual = env.theta_RV_actual;

    num_rays_list = params.num_rays_list;
    n_configs = length(num_rays_list);
    max_iter_track = params.max_iter;

    %% Run estimator for each multipath configuration
    error_vs_iter = zeros(n_configs, max_iter_track * 3 + 1);
    final_errors = zeros(1, n_configs);
    total_iters = zeros(1, n_configs);

    for r = 1:n_configs
        num_rays = num_rays_list(r);
        fprintf('  Testing with %d ray(s)...\n', num_rays);

        multipath_cfg.num_rays = num_rays;
        multipath_cfg.scatter_power = params.scatter_sigma * num_rays;

        % Use fixed initial pose for fair comparison
        rng(42);  % Fixed seed for reproducibility
        [p_RV_init, theta_RV_init, ~] = particle_filter_init(...
            p_BS_Tx, p_BS_Rx, p_RV_actual, theta_RV_actual, params);

        % Run estimator
        rng(42 + r);
        [~, ~, est_results] = run_estimator(p_BS_Tx, p_BS_Rx, ...
            p_RV_init, theta_RV_init, p_RV_actual, theta_RV_actual, ...
            params, multipath_cfg);

        n_iters = est_results.total_iterations;
        total_iters(r) = n_iters;
        error_vs_iter(r, 1:n_iters) = est_results.history.error;

        % Pad with final value
        if n_iters < size(error_vs_iter, 2)
            error_vs_iter(r, n_iters+1:end) = est_results.history.error(end);
        end

        final_errors(r) = est_results.final_error;
        fprintf('    %d rays: final error = %.3f m, iters = %d\n', ...
            num_rays, final_errors(r), n_iters);
    end

    %% Store results
    results.num_rays_list = num_rays_list;
    results.error_vs_iter = error_vs_iter;
    results.final_errors = final_errors;
    results.total_iters = total_iters;
    results.max_display_iter = max(total_iters);
end
