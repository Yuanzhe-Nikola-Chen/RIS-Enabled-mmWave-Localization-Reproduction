function [y_Rx, g_cascaded, SNR_dB, signal_strength_dBm] = ...
    compute_received_signal(p_BS_Tx, p_BS_Rx, p_RV_est, theta_RV_est, ...
    p_RV_actual, theta_RV_actual, params, multipath_config)
% COMPUTE_RECEIVED_SIGNAL  Compute the received signal at BSRx
%
% Implements Eq. (1)-(2): Cascaded BSTx -> RIS -> BSRx channel
%
% Inputs:
%   p_BS_Tx        - [3x1] BSTx position
%   p_BS_Rx        - [3x1] BSRx position
%   p_RV_est       - [3x1] Estimated RV position
%   theta_RV_est   - Estimated RV heading (rad)
%   p_RV_actual    - [3x1] Actual RV position
%   theta_RV_actual- Actual RV heading (rad)
%   params         - System parameters
%   multipath_config - (optional) struct with multipath settings
%
% Outputs:
%   y_Rx               - Complex received signal
%   g_cascaded         - Cascaded channel gain
%   SNR_dB             - Signal-to-noise ratio in dB
%   signal_strength_dBm- Received signal strength in dBm

    if nargin < 8
        multipath_config.num_rays = 1;  % LoS only
        multipath_config.scatter_power = 0;
    end

    lambda = params.lambda;

    %% Compute Angles
    % Estimated angles (for beam steering)
    [phi_BS_T_RV, zeta_BS_T_RV] = compute_angles(p_BS_Tx, p_RV_est);
    [phi_RV_BS_R, zeta_RV_BS_R] = compute_angles(p_RV_est, p_BS_Rx);

    % Actual angles (determine true propagation)
    [phi_BS_T_RV_hat, zeta_BS_T_RV_hat] = compute_angles(p_BS_Tx, p_RV_actual);
    [phi_RV_BS_R_hat, zeta_RV_BS_R_hat] = compute_angles(p_RV_actual, p_BS_Rx);

    %% Compute Array Coordinates
    % BSTx antenna coordinates
    bs_tx_coords = compute_antenna_coords(p_BS_Tx, params.N_h, params.N_v, ...
        params.d_h, params.d_v, params.d_s, 'BS', 0);
    % BSRx antenna coordinates
    bs_rx_coords = compute_antenna_coords(p_BS_Rx, params.N_h, params.N_v, ...
        params.d_h, params.d_v, params.d_s, 'BS', 0);
    % RIS element coordinates (actual position)
    ris_coords_actual = compute_antenna_coords(p_RV_actual, params.M_l, params.M_w, ...
        params.d_l, params.d_w, params.d_spacing, 'RIS', theta_RV_actual);
    % RIS element coordinates (estimated, for beam steering design)
    ris_coords_est = compute_antenna_coords(p_RV_est, params.M_l, params.M_w, ...
        params.d_l, params.d_w, params.d_spacing, 'RIS', theta_RV_est);

    %% BSTx Beam Steering Vector: Eq. (3)
    % Beam steered toward estimated RV position
    a_BS_Tx = compute_steering_vector(phi_BS_T_RV, zeta_BS_T_RV, ...
        p_BS_Tx, bs_tx_coords, lambda, 'tx');

    %% BSRx Array Response Vector
    % Response based on actual arrival angle
    a_BS_Rx = compute_steering_vector(phi_RV_BS_R_hat, zeta_RV_BS_R_hat, ...
        p_BS_Rx, bs_rx_coords, lambda, 'rx');

    %% RIS Phase Shift Design: Eq. (17)
    % Designed based on estimated angles
    [Psi_RIS, ~] = design_ris_phase_shift(phi_BS_T_RV, zeta_BS_T_RV, ...
        phi_RV_BS_R, zeta_RV_BS_R, theta_RV_est, p_RV_est, ...
        ris_coords_est, lambda, params, 'directional');

    %% RIS Array Response to Incident Wave: Eq. (6)
    % Based on actual incident angle at actual RIS position
    a_RIS_inc = compute_steering_vector(phi_BS_T_RV_hat, zeta_BS_T_RV_hat, ...
        p_RV_actual, ris_coords_actual, lambda, 'tx');

    %% RIS Array Response to Reflected Wave
    a_RIS_ref = compute_steering_vector(phi_RV_BS_R_hat, zeta_RV_BS_R_hat, ...
        p_RV_actual, ris_coords_actual, lambda, 'tx');

    %% Path Loss Gains
    % BSTx -> RIS path loss: Eq. (9)
    [g_BS_T_RV, ~, Delta_BS_T_RV] = compute_path_loss(p_BS_Tx, p_RV_actual, ...
        phi_BS_T_RV, zeta_BS_T_RV, phi_BS_T_RV_hat, zeta_BS_T_RV_hat, ...
        lambda, params.G_BS_T, 1, 'incident');

    % RIS -> BSRx path loss: Eq. (13)
    [g_RV_BS_R, ~, Delta_RV_BS_R] = compute_path_loss(p_RV_actual, p_BS_Rx, ...
        phi_RV_BS_R, zeta_RV_BS_R, phi_RV_BS_R_hat, zeta_RV_BS_R_hat, ...
        lambda, params.G_BS_R, params.G_RIS, 'reflected');

    %% Cascaded Channel Gain: Eq. (2)
    % MIMO beamforming gain
    bf_gain = (a_BS_Rx' * a_BS_Rx) * (a_BS_Tx' * a_BS_Tx);  % Simplified for same direction
    % More accurate: use actual angle response
    bf_gain_mimo = abs(a_BS_Rx' * ones(params.N, 1)) * abs(a_BS_Tx' * ones(params.N, 1)) / params.N^2;

    % RIS array response gain
    ris_gain = a_RIS_inc' * Psi_RIS * a_RIS_ref;

    % Full cascaded gain: Eq. (2)
    g_cascaded = g_BS_T_RV * g_RV_BS_R * params.N * abs(ris_gain) / params.M;

    %% Add Additional Losses
    total_loss_dB = params.weather_loss_dB + params.reflection_loss_dB + ...
                    params.polarization_loss_dB;
    g_cascaded = g_cascaded * 10^(-total_loss_dB / 20);

    %% Multipath Components
    multipath_signal = 0;
    if multipath_config.num_rays > 1
        for ray = 2:multipath_config.num_rays
            % Random scattered path with additional loss
            scatter_loss = 10 + 5 * ray;  % Increasing loss per ray (dB)
            random_phase = exp(1j * 2 * pi * rand());
            scatter_amp = abs(g_cascaded) * 10^(-scatter_loss / 20);
            multipath_signal = multipath_signal + scatter_amp * random_phase;
        end
    end

    %% Received Signal: Eq. (1)
    s = 1;  % Pilot signal with ||s||_2 = 1
    noise = sqrt(params.sigma2 / 2) * (randn() + 1j * randn());

    y_Rx = sqrt(params.P_Tx) * g_cascaded * s + multipath_signal + noise;

    %% Signal Metrics
    signal_power = abs(sqrt(params.P_Tx) * g_cascaded)^2;
    SNR_dB = 10 * log10(signal_power / params.sigma2);
    signal_strength_dBm = 10 * log10(signal_power) + 30;
end
