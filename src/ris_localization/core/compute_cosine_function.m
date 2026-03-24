function [F, F_phi, F_zeta] = compute_cosine_function(p_BS_Tx, p_BS_Rx, ...
    p_RV_est, theta_RV_est, p_RV_actual, theta_RV_actual, ris_ps_mode)
% COMPUTE_COSINE_FUNCTION  Compute the cosine-shape function F(φ,ζ)
%
% Implements Eq. (19)-(24):
%   F(φ,ζ) = Fφ(φ,θ) × Fζ(ζ)
% where Fφ and Fζ model the channel gain variation due to angular mismatch
%
% Inputs:
%   p_BS_Tx        - [3x1] BSTx position
%   p_BS_Rx        - [3x1] BSRx position
%   p_RV_est       - [3x1] Estimated RV position
%   theta_RV_est   - Estimated RV heading (rad)
%   p_RV_actual    - [3x1] Actual RV position
%   theta_RV_actual- Actual RV heading (rad)
%   ris_ps_mode    - 'full' (Eq. 22-23), 'incident_only' (Eq. 25),
%                    'reflection_only' (Eq. 28), 'heading_only'
%
% Outputs:
%   F       - Total cosine function value
%   F_phi   - Azimuth component
%   F_zeta  - Elevation component

    if nargin < 7
        ris_ps_mode = 'full';
    end

    % Estimated angles
    [phi_inc_est, zeta_inc_est] = compute_angles(p_BS_Tx, p_RV_est);
    [phi_ref_est, zeta_ref_est] = compute_angles(p_RV_est, p_BS_Rx);

    % Actual angles
    [phi_inc_act, zeta_inc_act] = compute_angles(p_BS_Tx, p_RV_actual);
    [phi_ref_act, zeta_ref_act] = compute_angles(p_RV_actual, p_BS_Rx);

    % Angular mismatches
    Delta_phi_inc = phi_inc_act - phi_inc_est;
    Delta_zeta_inc = zeta_inc_act - zeta_inc_est;
    Delta_phi_ref = phi_ref_act - phi_ref_est;
    Delta_zeta_ref = zeta_ref_act - zeta_ref_est;
    Delta_theta = theta_RV_actual - theta_RV_est;

    switch ris_ps_mode
        case 'full'
            % Eq. (22)-(23): Full cosine function
            F_phi = cos(Delta_phi_inc) * ...
                    cos(Delta_phi_inc + Delta_phi_ref + Delta_theta);
            F_zeta = cos(Delta_zeta_inc) * ...
                     cos(Delta_zeta_inc + Delta_zeta_ref);

        case 'incident_only'
            % Eq. (25): RIS acts as mirror, only incident path matters
            % cφ and cζ are unknown constants
            c_phi = cos(phi_ref_act - (phi_inc_act + theta_RV_actual));
            c_zeta = cos(zeta_ref_act - zeta_inc_act);
            F_phi = c_phi * cos(Delta_phi_inc);
            F_zeta = c_zeta * cos(Delta_zeta_inc);

        case 'reflection_only'
            % Eq. (28): Incident path fixed, adjust RIS-PS for reflection
            % After incident optimization: Δφ_inc ≈ 0, Δζ_inc ≈ 0
            F_phi = cos(Delta_phi_ref + Delta_theta);
            F_zeta = cos(Delta_zeta_ref);

        case 'heading_only'
            % Only heading estimation remains
            F_phi = cos(Delta_theta);
            F_zeta = 1;

        otherwise
            error('Unknown RIS-PS mode: %s', ris_ps_mode);
    end

    F = F_phi * F_zeta;
end
