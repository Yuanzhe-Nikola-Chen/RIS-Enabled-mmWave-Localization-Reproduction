function [phi, zeta] = compute_angles(p_from, p_to)
% COMPUTE_ANGLES  Compute azimuth and elevation angles between two points
%
% Implements Appendix B: Eq. (B-1) to (B-4)
%
% For BSTx -> RV path: p_from = p_BS_Tx, p_to = p_RV
% For RV -> BSRx path: p_from = p_RV, p_to = p_BS_Rx
%
% Inputs:
%   p_from - [3x1] Source position
%   p_to   - [3x1] Target position
%
% Outputs:
%   phi    - Azimuth angle (radians), measured from x-axis in XY plane
%   zeta   - Elevation angle (radians), from horizontal plane

    dx = p_to(1) - p_from(1);
    dy = p_to(2) - p_from(2);
    dz = p_to(3) - p_from(3);

    % Horizontal distance
    d_xy = sqrt(dx^2 + dy^2);

    % Azimuth angle: Eq. (B-1) and (B-3)
    % atan2 gives the correct quadrant
    phi = atan2(dy, dx);

    % Elevation angle: Eq. (B-2) and (B-4)
    if d_xy > 1e-10
        zeta = atan2(dz, d_xy);
    else
        zeta = sign(dz) * pi/2;
    end
end
