function coords = compute_antenna_coords(p_center, N_h, N_v, d_h, d_v, d_s, type, theta_RV)
% COMPUTE_ANTENNA_COORDS  Compute coordinates of BS antennas or RIS elements
%
% Implements Appendix A: Eq. (A-1) for BS and Eq. (A-2) for RIS
%
% Inputs:
%   p_center  - [3x1] Center position of the array
%   N_h       - Number of horizontal elements
%   N_v       - Number of vertical elements
%   d_h       - Horizontal element size
%   d_v       - Vertical element size
%   d_s       - Element spacing
%   type      - 'BS' or 'RIS'
%   theta_RV  - RV heading angle (only used for type='RIS')
%
% Output:
%   coords    - [3 x N] matrix, each column is position of an element

    N_total = N_h * N_v;
    coords = zeros(3, N_total);

    if strcmp(type, 'BS')
        % Eq. (A-1): BS antenna coordinates
        for n = 1:N_total
            mod_val = mod(n, N_h);
            if mod_val == 0
                mod_val = N_h;
            end
            ceil_val = ceil(n / N_h);

            offset = [0;
                      d_h * (mod_val - 0.5) + d_s * (mod_val - 1);
                      d_v * (ceil_val - 0.5) + d_s * (ceil_val - 1)];
            coords(:, n) = p_center + offset;
        end

    elseif strcmp(type, 'RIS')
        % Eq. (A-2): RIS element coordinates
        % Rotation matrix: global -> local (inverse = local -> global)
        Gamma = [cos(theta_RV), -sin(theta_RV), 0;
                 sin(theta_RV),  cos(theta_RV), 0;
                 0,              0,              1];
        Gamma_inv = Gamma';  % = Gamma^{-1} for rotation matrix

        M_l = N_h;  % Using N_h as M_l for RIS
        M_w = N_v;  % Using N_v as M_w for RIS
        d_l = d_h;
        d_w = d_v;

        for m = 1:N_total
            % l(m) and w(m) from Appendix A
            mod_val = mod(m - 1, M_l);
            l_m = d_l * (mod_val - (M_l - 1) / 2);
            w_m = d_w * ((M_w + 1) / 2 - ceil(m / M_l));

            local_offset = [l_m; w_m; 0];
            coords(:, m) = p_center + Gamma_inv * local_offset;
        end
    end
end
