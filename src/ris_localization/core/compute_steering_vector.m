function a = compute_steering_vector(phi, zeta, p_center, coords, lambda, mode)
% COMPUTE_STEERING_VECTOR  Compute array steering/response vector
%
% Implements Eq. (3)-(4) for BS and Eq. (6)-(7) for RIS
%
% Inputs:
%   phi      - Azimuth angle (radians)
%   zeta     - Elevation angle (radians)
%   p_center - [3x1] Center position of the array
%   coords   - [3xN] Coordinates of each element
%   lambda   - Carrier wavelength (m)
%   mode     - 'tx' (transmit/incident), 'rx' (receive/reflect)
%
% Output:
%   a        - [Nx1] Complex steering/response vector

    N = size(coords, 2);
    W = compute_wave_vector(phi, zeta);

    % Phase shift for each element: Eq. (4) or (7)
    phase_shifts = zeros(N, 1);
    for n = 1:N
        delta_p = coords(:, n) - p_center;
        phase_shifts(n) = (2 * pi / lambda) * (W' * delta_p);
    end

    % Steering vector: Eq. (3) or (6)
    if strcmp(mode, 'rx')
        % Conjugate for receive (AoA response)
        a = exp(1j * phase_shifts);
    else
        % Transmit steering vector
        a = exp(-1j * phase_shifts);
    end
end
