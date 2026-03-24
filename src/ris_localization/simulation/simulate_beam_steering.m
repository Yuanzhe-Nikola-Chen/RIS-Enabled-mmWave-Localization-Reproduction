function results = simulate_beam_steering(params, env)
% SIMULATE_BEAM_STEERING  Simulate beam steering and signal strength
%
% Reproduces the signal strength comparison from Section V:
%   Without beam steering: -120.7 dBm
%   With beam steering:    -62.5 dBm

    p_BS_Tx = env.p_BS_Tx;
    p_BS_Rx = env.p_BS_Rx;
    p_RV = env.p_RV_actual;
    theta_RV = env.theta_RV_actual;

    multipath_cfg.num_rays = 1;
    multipath_cfg.scatter_power = 0;

    %% Case 1: With accurate beam steering (perfect CSI)
    [y_Rx_BF, g_BF, SNR_BF, ss_BF] = compute_received_signal(...
        p_BS_Tx, p_BS_Rx, p_RV, theta_RV, p_RV, theta_RV, params, multipath_cfg);

    %% Case 2: Without beam steering (random/no beamforming)
    % Simulate with large position error (no directional steering)
    p_RV_random = p_RV + [50; 50; 10];  % Far-off estimate
    theta_RV_random = theta_RV + pi;

    [y_Rx_noBF, g_noBF, SNR_noBF, ss_noBF] = compute_received_signal(...
        p_BS_Tx, p_BS_Rx, p_RV_random, theta_RV_random, p_RV, theta_RV, params, multipath_cfg);

    %% Signal strength for varying angular mismatches
    N_points = 100;
    phi_offsets = linspace(-pi/4, pi/4, N_points);
    zeta_offsets = linspace(-pi/6, pi/6, N_points);

    ss_vs_phi = zeros(1, N_points);
    ss_vs_zeta = zeros(1, N_points);

    for i = 1:N_points
        % Vary azimuth
        [phi_inc, zeta_inc] = compute_angles(p_BS_Tx, p_RV);
        d_inc = norm(p_RV - p_BS_Tx);
        p_RV_off = p_BS_Tx + d_inc * [cos(zeta_inc)*cos(phi_inc + phi_offsets(i));
                                       cos(zeta_inc)*sin(phi_inc + phi_offsets(i));
                                       sin(zeta_inc)];
        [~, ~, ~, ss_vs_phi(i)] = compute_received_signal(...
            p_BS_Tx, p_BS_Rx, p_RV, theta_RV, p_RV_off, theta_RV, params, multipath_cfg);

        % Vary elevation
        p_RV_off_z = p_BS_Tx + d_inc * [cos(zeta_inc + zeta_offsets(i))*cos(phi_inc);
                                         cos(zeta_inc + zeta_offsets(i))*sin(phi_inc);
                                         sin(zeta_inc + zeta_offsets(i))];
        [~, ~, ~, ss_vs_zeta(i)] = compute_received_signal(...
            p_BS_Tx, p_BS_Rx, p_RV, theta_RV, p_RV_off_z, theta_RV, params, multipath_cfg);
    end

    %% Store results
    results.ss_with_BF = ss_BF;
    results.ss_without_BF = ss_noBF;
    results.SNR_with_BF = SNR_BF;
    results.SNR_without_BF = SNR_noBF;
    results.phi_offsets = rad2deg(phi_offsets);
    results.zeta_offsets = rad2deg(zeta_offsets);
    results.ss_vs_phi = ss_vs_phi;
    results.ss_vs_zeta = ss_vs_zeta;

    fprintf('  Signal strength with beam steering: %.1f dBm\n', ss_BF);
    fprintf('  Signal strength without beam steering: %.1f dBm\n', ss_noBF);
    fprintf('  Improvement: %.1f dB\n', ss_BF - ss_noBF);
end
