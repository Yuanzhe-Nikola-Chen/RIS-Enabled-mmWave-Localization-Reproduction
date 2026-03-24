function [V, V_dot, is_stable] = verify_lyapunov_stability(x1, x2, c1, c2)
% VERIFY_LYAPUNOV_STABILITY  Verify stability of estimator E
%
% Implements Eq. (27):
%   V = 2 - cos²(x1) - cos²(x2)  ≥ 0
%   V̇ = -(c1*sin²(x1)*cos(x1) + c2*sin²(x2)*cos(x2))  ≤ 0
%
% Inputs:
%   x1, x2 - Estimator states (Δφ and Δζ)
%   c1, c2 - Positive constants
%
% Outputs:
%   V          - Lyapunov function value
%   V_dot      - Time derivative of V
%   is_stable  - Boolean: true if V≥0 and V̇≤0

    % Lyapunov function: Eq. (27)
    V = 2 - cos(x1)^2 - cos(x2)^2;

    % Time derivative: V̇
    V_dot = -(c1 * sin(x1)^2 * cos(x1) + c2 * sin(x2)^2 * cos(x2));

    % Stability check
    is_stable = (V >= -1e-10) && (V_dot <= 1e-10);
end
