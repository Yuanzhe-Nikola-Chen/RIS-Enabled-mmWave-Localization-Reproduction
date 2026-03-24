function W = compute_wave_vector(phi, zeta)
% COMPUTE_WAVE_VECTOR  Compute normalized wave vector
%
% Implements Eq. (5):
%   W(phi, zeta) = [cos(zeta)*cos(phi); cos(zeta)*sin(phi); sin(zeta)]
%
% Inputs:
%   phi  - Azimuth angle (radians)
%   zeta - Elevation angle (radians)
%
% Output:
%   W    - [3x1] Normalized wave vector

    W = [cos(zeta) * cos(phi);
         cos(zeta) * sin(phi);
         sin(zeta)];
end
