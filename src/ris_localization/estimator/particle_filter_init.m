function [p_RV_init, theta_RV_init, pf_results] = particle_filter_init(...
    p_BS_Tx, p_BS_Rx, p_RV_actual, theta_RV_actual, params)
% PARTICLE_FILTER_INIT  Initialize RV pose using particle filtering
%
% Implements the particle filtering described in Section IV.A:
%   Generate random poses, execute BS-BF and RIS-PS for each,
%   select pose corresponding to maximum signal strength.
%
% Inputs:
%   p_BS_Tx        - [3x1] BSTx position
%   p_BS_Rx        - [3x1] BSRx position
%   p_RV_actual    - [3x1] Actual RV position
%   theta_RV_actual- Actual RV heading
%   params         - System parameters
%
% Outputs:
%   p_RV_init      - [3x1] Initial estimated RV position
%   theta_RV_init  - Initial estimated RV heading
%   pf_results     - Struct with all particle filtering data

    N_particles = params.N_particles;

    % Generate random poses in the search area
    % Center around a rough area (simulating no GPS)
    search_center = p_RV_actual + [5*(rand()-0.5); 5*(rand()-0.5); 0];

    particles = zeros(4, N_particles);  % [x; y; z; theta]
    signal_strengths = zeros(1, N_particles);

    for i = 1:N_particles
        % Random position within search range
        particles(1, i) = p_RV_actual(1) + params.pf_range_xy * (rand() - 0.5);
        particles(2, i) = p_RV_actual(2) + params.pf_range_xy * (rand() - 0.5);
        particles(3, i) = p_RV_actual(3) + params.pf_range_z * (rand() - 0.5);
        particles(4, i) = theta_RV_actual + params.pf_range_theta * (rand() - 0.5);

        p_RV_random = particles(1:3, i);
        theta_RV_random = particles(4, i);

        % Compute received signal for this random pose
        multipath_cfg.num_rays = 1;
        multipath_cfg.scatter_power = 0;

        [~, ~, ~, ss_dBm] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_random, theta_RV_random, p_RV_actual, theta_RV_actual, ...
            params, multipath_cfg);

        signal_strengths(i) = ss_dBm;
    end

    % Select particle with maximum signal strength
    [max_ss, best_idx] = max(signal_strengths);
    p_RV_init = particles(1:3, best_idx);
    theta_RV_init = particles(4, best_idx);

    % Store results for plotting
    pf_results.particles = particles;
    pf_results.signal_strengths = signal_strengths;
    pf_results.best_idx = best_idx;
    pf_results.max_signal_strength = max_ss;
    pf_results.init_error = norm(p_RV_init - p_RV_actual);

    fprintf('  Particle filtering: init error = %.2f m, best SS = %.1f dBm\n', ...
        pf_results.init_error, max_ss);
end
