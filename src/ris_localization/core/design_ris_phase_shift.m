function [Psi_RIS, psi_diag] = design_ris_phase_shift(phi_inc, zeta_inc, ...
    phi_ref, zeta_ref, theta_RV, p_RV, ris_coords, lambda, params, mode)
% DESIGN_RIS_PHASE_SHIFT  Design the RIS phase shift matrix
%
% Implements Eq. (17)-(18):
%   ΨRIS = a_inc(φ, ζ) * a_ref†(φ, ζ)  (directional beam steering)
%
% Or mirror mode (reflection law) for estimator stages
%
% Inputs:
%   phi_inc   - Azimuth angle of incident path (estimated)
%   zeta_inc  - Elevation angle of incident path (estimated)
%   phi_ref   - Azimuth angle of reflection path (estimated)
%   zeta_ref  - Elevation angle of reflection path (estimated)
%   theta_RV  - Estimated RV heading
%   p_RV      - [3x1] Estimated RV position (RIS center)
%   ris_coords- [3xM] RIS element coordinates
%   lambda    - Carrier wavelength
%   params    - System parameters
%   mode      - 'directional' (Eq. 17) or 'mirror' (reflection law)
%
% Outputs:
%   Psi_RIS   - [MxM] RIS phase shift matrix (diagonal)
%   psi_diag  - [Mx1] Diagonal entries of phase shift matrix

    M = size(ris_coords, 2);

    if strcmp(mode, 'directional')
        % Eq. (17): Directional RIS-PS for beam steering
        % Incident array response
        a_inc = compute_steering_vector(phi_inc, zeta_inc, p_RV, ...
            ris_coords, lambda, 'tx');
        % Reflected array response
        a_ref = compute_steering_vector(phi_ref, zeta_ref, p_RV, ...
            ris_coords, lambda, 'tx');

        % Phase shift: compensate incident and direct toward reflection
        % Eq. (18): azimuth and elevation phase components
        psi_diag = conj(a_inc) .* a_ref;

    elseif strcmp(mode, 'mirror')
        % Mirror mode: RIS follows reflection law (no additional steering)
        % Phase shift is zero (identity)
        psi_diag = ones(M, 1);

    else
        error('Unknown RIS-PS mode: %s', mode);
    end

    % Normalize to unit magnitude
    psi_diag = psi_diag ./ abs(psi_diag);

    % Construct diagonal matrix
    Psi_RIS = diag(psi_diag);
end
