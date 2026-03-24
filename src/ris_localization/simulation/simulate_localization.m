function results = simulate_localization(params, env)
% SIMULATE_LOCALIZATION  Full localization simulation
%
% Reproduces Fig. 4:
%   (a) Particle filtering initialization
%   (b) Estimator convergence to actual pose (20 cm accuracy)

    p_BS_Tx = env.p_BS_Tx;
    p_BS_Rx = env.p_BS_Rx;
    p_RV_actual = env.p_RV_actual;
    theta_RV_actual = env.theta_RV_actual;

    %% Step 1: Particle Filtering Initialization (Fig. 4a)
    fprintf('  Running particle filtering initialization...\n');
    [p_RV_init, theta_RV_init, pf_results] = particle_filter_init(...
        p_BS_Tx, p_BS_Rx, p_RV_actual, theta_RV_actual, params);

    %% Step 2: Accurate Localization with Estimator (Fig. 4b)
    fprintf('  Running differential-based estimator...\n');
    multipath_cfg.num_rays = 1;
    multipath_cfg.scatter_power = 0;

    [p_RV_final, theta_RV_final, est_results] = run_estimator(...
        p_BS_Tx, p_BS_Rx, p_RV_init, theta_RV_init, ...
        p_RV_actual, theta_RV_actual, params, multipath_cfg);

    %% Lyapunov Stability Verification
    fprintf('  Verifying Lyapunov stability...\n');
    n_test = 100;
    x1_range = linspace(-pi/2, pi/2, n_test);
    x2_range = linspace(-pi/2, pi/2, n_test);
    V_surface = zeros(n_test, n_test);
    V_dot_surface = zeros(n_test, n_test);
    stability_check = true;

    for i = 1:n_test
        for j = 1:n_test
            [V_val, V_dot_val, is_stable] = verify_lyapunov_stability(...
                x1_range(i), x2_range(j), params.c1, params.c2);
            V_surface(i, j) = V_val;
            V_dot_surface(i, j) = V_dot_val;
            if ~is_stable
                stability_check = false;
            end
        end
    end

    %% Store results
    results.pf = pf_results;
    results.est = est_results;
    results.p_RV_init = p_RV_init;
    results.theta_RV_init = theta_RV_init;
    results.p_RV_final = p_RV_final;
    results.theta_RV_final = theta_RV_final;
    results.lyapunov.V_surface = V_surface;
    results.lyapunov.V_dot_surface = V_dot_surface;
    results.lyapunov.x1_range = x1_range;
    results.lyapunov.x2_range = x2_range;
    results.lyapunov.is_stable = stability_check;

    fprintf('  Final localization error: %.2f m\n', est_results.final_error);
    fprintf('  Heading error: %.2f deg\n', rad2deg(abs(theta_RV_final - theta_RV_actual)));
    fprintf('  Lyapunov stability: %s\n', string(stability_check));
end
