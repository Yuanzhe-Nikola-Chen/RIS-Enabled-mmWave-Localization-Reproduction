function [p_RV_final, theta_RV_final, est_results] = run_estimator(...
    p_BS_Tx, p_BS_Rx, p_RV_init, theta_RV_init, p_RV_actual, ...
    theta_RV_actual, params, multipath_config)
% RUN_ESTIMATOR  Run the differential-based estimator for accurate localization
%
% Implements Eq. (26)-(29) and the 3-stage localization procedure:
%   Stage 1: Optimize incident path (BSTx-BF), RIS as mirror  (Eq. 25)
%   Stage 2: Optimize reflection path (RIS-PS), incident fixed (Eq. 28)
%   Stage 3: Optimize RV heading                               (Eq. 28)
%
% Inputs:
%   p_BS_Tx, p_BS_Rx        - BS positions
%   p_RV_init, theta_RV_init- Initial estimated RV pose (from PF)
%   p_RV_actual, theta_RV_actual - Actual RV pose (ground truth)
%   params                  - System parameters
%   multipath_config        - Multipath settings
%
% Outputs:
%   p_RV_final       - [3x1] Final estimated RV position
%   theta_RV_final   - Final estimated RV heading
%   est_results      - Struct with convergence history

    if nargin < 8
        multipath_config.num_rays = 1;
        multipath_config.scatter_power = 0;
    end

    c1 = params.c1;
    c2 = params.c2;
    max_iter = params.max_iter;
    epsilon = params.epsilon;

    % Current estimate
    p_RV = p_RV_init;
    theta_RV = theta_RV_init;

    % History storage
    history.p_RV = zeros(3, 3*max_iter + 1);
    history.theta_RV = zeros(1, 3*max_iter + 1);
    history.signal_strength = zeros(1, 3*max_iter + 1);
    history.error = zeros(1, 3*max_iter + 1);
    history.stage = zeros(1, 3*max_iter + 1);

    total_iter = 0;

    %% ===== STAGE 1: Incident Path Optimization (BSTx-BF) =====
    % RIS acts as mirror, only adjust BSTx beamforming
    % Eq. (25): Fφ ≈ cφ*cos(Δφ_inc), Fζ ≈ cζ*cos(Δζ_inc)

    % Get initial signal strength
    [~, ~, ~, ss_prev] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
        p_RV, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);

    total_iter = total_iter + 1;
    history.p_RV(:, total_iter) = p_RV;
    history.theta_RV(total_iter) = theta_RV;
    history.signal_strength(total_iter) = ss_prev;
    history.error(total_iter) = norm(p_RV - p_RV_actual);
    history.stage(total_iter) = 1;

    % Estimator variables: x1 = Δφ_inc, x2 = Δζ_inc
    [phi_inc_est, zeta_inc_est] = compute_angles(p_BS_Tx, p_RV);

    for k = 1:max_iter
        % Perturb azimuth angle of incident path
        delta_phi = c1 * 0.01;  % Small perturbation
        delta_zeta = c2 * 0.01;

        % Update estimated position based on angular adjustment
        [phi_inc_curr, zeta_inc_curr] = compute_angles(p_BS_Tx, p_RV);

        % Try adjustment in azimuth
        p_RV_try = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
            phi_inc_curr + delta_phi, zeta_inc_curr, 'incident');

        [~, ~, ~, ss_try_phi] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_try, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);

        % Try adjustment in elevation
        p_RV_try_z = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
            phi_inc_curr, zeta_inc_curr + delta_zeta, 'incident');

        [~, ~, ~, ss_try_zeta] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_try_z, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);

        % Discrete estimator Ed: Eq. (29)
        % y^Rx_{k/k-1} = sign(mag(y[k]) - mag(y[k-1]))
        y_Rx_phi = sign(ss_try_phi - ss_prev);
        y_Rx_zeta = sign(ss_try_zeta - ss_prev);

        % Update: x1[k+1] = x1[k] + c1 * y_Rx * (x1[k] - x1[k-1])
        if y_Rx_phi >= 0
            % Move in the same direction (signal improved)
            p_RV = p_RV_try;
            ss_prev = ss_try_phi;
        else
            % Move in opposite direction
            p_RV_opp = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
                phi_inc_curr - delta_phi, zeta_inc_curr, 'incident');
            [~, ~, ~, ss_opp] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
                p_RV_opp, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);
            if ss_opp > ss_prev
                p_RV = p_RV_opp;
                ss_prev = ss_opp;
            end
        end

        % Elevation update
        [phi_inc_curr2, zeta_inc_curr2] = compute_angles(p_BS_Tx, p_RV);
        if y_Rx_zeta >= 0
            p_RV_try_z2 = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
                phi_inc_curr2, zeta_inc_curr2 + delta_zeta, 'incident');
        else
            p_RV_try_z2 = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
                phi_inc_curr2, zeta_inc_curr2 - delta_zeta, 'incident');
        end
        [~, ~, ~, ss_z] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_try_z2, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);
        if ss_z > ss_prev
            p_RV = p_RV_try_z2;
            ss_prev = ss_z;
        end

        % Record
        total_iter = total_iter + 1;
        history.p_RV(:, total_iter) = p_RV;
        history.theta_RV(total_iter) = theta_RV;
        history.signal_strength(total_iter) = ss_prev;
        history.error(total_iter) = norm(p_RV - p_RV_actual);
        history.stage(total_iter) = 1;

        % Check convergence
        if k > 1 && abs(history.signal_strength(total_iter) - ...
                history.signal_strength(total_iter-1)) < epsilon
            break;
        end
    end
    stage1_iters = total_iter - 1;

    %% ===== STAGE 2: Reflection Path Optimization (RIS-PS) =====
    % Incident path fixed, adjust RIS phase shift
    % Eq. (28): Fφ ≈ cos(Δφ_ref + θ̂_RV), Fζ ≈ cos(Δζ_ref)

    for k = 1:max_iter
        [phi_ref_curr, zeta_ref_curr] = compute_angles(p_RV, p_BS_Rx);

        delta_phi_ref = c1 * 0.005;
        delta_zeta_ref = c2 * 0.005;

        % Try azimuth adjustment on reflection path
        p_RV_try_ref = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
            phi_ref_curr + delta_phi_ref, zeta_ref_curr, 'reflection');

        [~, ~, ~, ss_try_ref] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_try_ref, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);

        y_Rx_ref = sign(ss_try_ref - ss_prev);
        if y_Rx_ref >= 0
            p_RV = p_RV_try_ref;
            ss_prev = ss_try_ref;
        else
            p_RV_opp_ref = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
                phi_ref_curr - delta_phi_ref, zeta_ref_curr, 'reflection');
            [~, ~, ~, ss_opp_ref] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
                p_RV_opp_ref, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);
            if ss_opp_ref > ss_prev
                p_RV = p_RV_opp_ref;
                ss_prev = ss_opp_ref;
            end
        end

        % Elevation on reflection path
        [phi_ref_curr2, zeta_ref_curr2] = compute_angles(p_RV, p_BS_Rx);
        p_RV_try_z_ref = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV, ...
            phi_ref_curr2, zeta_ref_curr2 + sign(y_Rx_ref)*delta_zeta_ref, 'reflection');
        [~, ~, ~, ss_z_ref] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_try_z_ref, theta_RV, p_RV_actual, theta_RV_actual, params, multipath_config);
        if ss_z_ref > ss_prev
            p_RV = p_RV_try_z_ref;
            ss_prev = ss_z_ref;
        end

        % Record
        total_iter = total_iter + 1;
        history.p_RV(:, total_iter) = p_RV;
        history.theta_RV(total_iter) = theta_RV;
        history.signal_strength(total_iter) = ss_prev;
        history.error(total_iter) = norm(p_RV - p_RV_actual);
        history.stage(total_iter) = 2;

        if k > 1 && abs(history.signal_strength(total_iter) - ...
                history.signal_strength(total_iter-1)) < epsilon
            break;
        end
    end
    stage2_iters = total_iter - stage1_iters - 1;

    %% ===== STAGE 3: Heading Optimization =====
    % Position fixed, optimize heading
    % Fφ ≈ cos(θ̂_RV)

    for k = 1:max_iter
        delta_theta = c1 * 0.01;

        theta_try = theta_RV + delta_theta;
        [~, ~, ~, ss_try_th] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV, theta_try, p_RV_actual, theta_RV_actual, params, multipath_config);

        y_Rx_th = sign(ss_try_th - ss_prev);
        if y_Rx_th >= 0
            theta_RV = theta_try;
            ss_prev = ss_try_th;
        else
            theta_opp = theta_RV - delta_theta;
            [~, ~, ~, ss_opp_th] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
                p_RV, theta_opp, p_RV_actual, theta_RV_actual, params, multipath_config);
            if ss_opp_th > ss_prev
                theta_RV = theta_opp;
                ss_prev = ss_opp_th;
            end
        end

        total_iter = total_iter + 1;
        history.p_RV(:, total_iter) = p_RV;
        history.theta_RV(total_iter) = theta_RV;
        history.signal_strength(total_iter) = ss_prev;
        history.error(total_iter) = norm(p_RV - p_RV_actual);
        history.stage(total_iter) = 3;

        if k > 1 && abs(history.signal_strength(total_iter) - ...
                history.signal_strength(total_iter-1)) < epsilon
            break;
        end
    end

    %% Finalize
    p_RV_final = p_RV;
    theta_RV_final = theta_RV;

    % Trim history
    history.p_RV = history.p_RV(:, 1:total_iter);
    history.theta_RV = history.theta_RV(1:total_iter);
    history.signal_strength = history.signal_strength(1:total_iter);
    history.error = history.error(1:total_iter);
    history.stage = history.stage(1:total_iter);

    est_results.history = history;
    est_results.total_iterations = total_iter;
    est_results.final_error = norm(p_RV_final - p_RV_actual);
    est_results.stage1_iters = stage1_iters;
    est_results.stage2_iters = stage2_iters;
    est_results.stage3_iters = total_iter - stage1_iters - stage2_iters - 1;

    fprintf('  Estimator converged: error = %.2f m in %d iterations\n', ...
        est_results.final_error, total_iter);
    fprintf('    Stage 1 (incident): %d iters, Stage 2 (reflection): %d iters, Stage 3 (heading): %d iters\n', ...
        est_results.stage1_iters, est_results.stage2_iters, est_results.stage3_iters);
end


function p_new = update_position_from_angles(p_BS_Tx, p_BS_Rx, p_RV_curr, ...
    phi_new, zeta_new, path_type)
% UPDATE_POSITION_FROM_ANGLES  Update RV position from angular adjustments
%
% Given new azimuth/elevation angles, compute the corresponding position

    if strcmp(path_type, 'incident')
        % Update position based on incident path from BSTx
        d = norm(p_RV_curr - p_BS_Tx);
        d_horiz = d * cos(zeta_new);
        p_new = p_BS_Tx + [d_horiz * cos(phi_new);
                           d_horiz * sin(phi_new);
                           d * sin(zeta_new)];
    elseif strcmp(path_type, 'reflection')
        % Update position based on reflection path to BSRx
        d = norm(p_BS_Rx - p_RV_curr);
        d_horiz = d * cos(zeta_new);
        p_new = p_BS_Rx - [d_horiz * cos(phi_new);
                           d_horiz * sin(phi_new);
                           d * sin(zeta_new)];
    end
end
