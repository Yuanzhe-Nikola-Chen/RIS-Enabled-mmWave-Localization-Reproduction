function [g_path, F_rad, Delta] = compute_path_loss(p_from, p_to, ...
    phi_est, zeta_est, phi_actual, zeta_actual, lambda, G_ant, G_ris, path_type)
% COMPUTE_PATH_LOSS  Compute path loss gain and radiation pattern
%
% Implements Eq. (9)-(16):
%   Path loss: g = lambda * sqrt(G * F(φ,ζ)) / (4π) * exp(-jφ) / Δ
%   Radiation pattern: F(φ,ζ) = cos(Δζ) * cos(Δφ) (within main lobe)
%
% Inputs:
%   p_from       - [3x1] Source position
%   p_to         - [3x1] Target position
%   phi_est      - Estimated azimuth angle (for beam direction)
%   zeta_est     - Estimated elevation angle (for beam direction)
%   phi_actual   - Actual azimuth angle
%   zeta_actual  - Actual elevation angle
%   lambda       - Carrier wavelength
%   G_ant        - Antenna gain (linear)
%   G_ris        - RIS reflection gain (linear), set to 1 if not applicable
%   path_type    - 'incident' (BSTx->RIS) or 'reflected' (RIS->BSRx)
%
% Outputs:
%   g_path       - Complex path loss gain
%   F_rad        - Radiation pattern value [0, 1]
%   Delta        - Euclidean distance between points

    % Distance: Eq. (11), (15)
    Delta = norm(p_to - p_from);

    % Phase shift due to propagation: Eq. (10), (14)
    W = compute_wave_vector(phi_actual, zeta_actual);
    phi_propagation = (2 * pi / lambda) * (W' * (p_to - p_from));

    % Radiation pattern: Eq. (12) or (16)
    delta_phi = phi_actual - phi_est;
    delta_zeta = zeta_actual - zeta_est;

    % Wrap angle differences to [-pi, pi]
    delta_phi = wrapToPi(delta_phi);
    delta_zeta = wrapToPi(delta_zeta);

    if strcmp(path_type, 'incident')
        % Eq. (12): BSTx radiation pattern
        if abs(delta_zeta) <= pi/2 && abs(delta_phi) <= pi/2
            F_rad = cos(delta_zeta) * cos(delta_phi);
        else
            F_rad = 0;
        end
    elseif strcmp(path_type, 'reflected')
        % Eq. (16): RIS reflected radiation pattern
        if delta_zeta >= -pi && delta_zeta <= pi
            F_rad = cos(delta_zeta) * cos(delta_phi);
            F_rad = max(F_rad, 0);
        else
            F_rad = 0;
        end
    end

    % Path loss gain: Eq. (9) or (13)
    if Delta > 0
        total_gain = G_ant * G_ris;
        g_path = (lambda * sqrt(total_gain * abs(F_rad)) / (4 * pi)) * ...
                 exp(-1j * phi_propagation) / Delta;
    else
        g_path = 0;
    end
end

function angle = wrapToPi(angle)
% Wrap angle to [-pi, pi]
    angle = mod(angle + pi, 2*pi) - pi;
end
