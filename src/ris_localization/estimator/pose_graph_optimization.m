function [X_RV_opt, slam_results] = pose_graph_optimization(...
    trajectory, p_BS_Tx, p_BS_Rx, p_RV_actual_traj, theta_RV_actual_traj, params)
% POSE_GRAPH_OPTIMIZATION  Virtual loop closure for pose graph SLAM
%
% Implements Eq. (30)-(34):
%   - Weighting coefficient Ωi based on estimator convergence
%   - Least-squares optimization for drift removal
%   - Odometry-based pose propagation: Eq. (32)-(33)
%
% Inputs:
%   trajectory            - Struct with trajectory data
%   p_BS_Tx, p_BS_Rx      - BS positions
%   p_RV_actual_traj      - [3xI] Actual RV positions along trajectory
%   theta_RV_actual_traj  - [1xI] Actual RV headings along trajectory
%   params                - System parameters
%
% Outputs:
%   X_RV_opt     - [4xI] Optimized poses [x; y; z; theta]
%   slam_results - Struct with optimization details

    I = size(p_RV_actual_traj, 2);  % Number of poses
    gamma = params.gamma;

    %% Generate odometry with drift
    % Eq. (33): Vehicle kinematic model
    X_odom = zeros(4, I);           % Odometry-based poses
    X_actual = zeros(4, I);         % Ground truth
    odometry_noise_std = [0.05; 0.05; 0.01; deg2rad(0.5)];  % x,y,z,theta noise

    X_actual(:, 1) = [p_RV_actual_traj(:, 1); theta_RV_actual_traj(1)];
    X_odom(:, 1) = X_actual(:, 1);  % Start with known initial pose

    for i = 2:I
        % Actual displacement
        U_actual = [p_RV_actual_traj(:, i) - p_RV_actual_traj(:, i-1);
                    theta_RV_actual_traj(i) - theta_RV_actual_traj(i-1)];

        % Odometry with drift (accumulating noise)
        noise = odometry_noise_std .* randn(4, 1);
        U_odom = U_actual + noise;

        X_actual(:, i) = [p_RV_actual_traj(:, i); theta_RV_actual_traj(i)];
        X_odom(:, i) = X_odom(:, i-1) + U_odom;
    end

    %% Run Estimator at Each Pose and Compute Weights
    % Eq. (30): Ωi = 1 / (1 + γ * |y^Rx_{k/k-1}|)
    Omega = zeros(1, I);
    estimator_residuals = zeros(1, I);
    converged_poses = false(1, I);

    multipath_cfg.num_rays = 1;
    multipath_cfg.scatter_power = 0;

    for i = 1:I
        p_RV_est_i = X_odom(1:3, i);
        theta_RV_est_i = X_odom(4, i);

        % Quick estimator convergence check (simplified)
        [~, ~, ~, ss1] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_RV_est_i, theta_RV_est_i, ...
            p_RV_actual_traj(:, i), theta_RV_actual_traj(i), ...
            params, multipath_cfg);

        % Perturbed estimate
        p_perturbed = p_RV_est_i + 0.01 * randn(3, 1);
        [~, ~, ~, ss2] = compute_received_signal(p_BS_Tx, p_BS_Rx, ...
            p_perturbed, theta_RV_est_i, ...
            p_RV_actual_traj(:, i), theta_RV_actual_traj(i), ...
            params, multipath_cfg);

        residual = abs(ss1 - ss2) / max(abs(ss1), 1e-10);
        estimator_residuals(i) = residual;

        % Weight: Eq. (30)
        Omega(i) = 1 / (1 + gamma * residual);

        % Check if converged within time constant
        if residual < params.epsilon || i == 1 || i == I
            converged_poses(i) = true;
            Omega(i) = 1.0;  % Full weight for converged poses
        end
    end

    % Ensure first and last poses have accurate localization
    converged_poses(1) = true;
    converged_poses(end) = true;
    Omega(1) = 1.0;
    Omega(end) = 1.0;

    % For converged poses, use accurate localization
    X_est = X_odom;
    X_est(:, 1) = X_actual(:, 1);    % Start pose is accurate
    X_est(:, end) = X_actual(:, end); % End pose is accurate (loop closure)

    %% Pose Graph Optimization: Eq. (34)
    % argmin Σ Ωi * ||X_{i-1} + U_{i-1} - X_i||^2
    % Using weighted least squares

    X_opt = X_est;

    % Iterative optimization (Gauss-Newton style)
    n_opt_iters = 50;
    for iter = 1:n_opt_iters
        X_prev = X_opt;

        for i = 2:I-1  % Don't modify first and last (loop closure anchors)
            % Compute odometry constraint from previous pose
            U_prev = X_odom(:, i) - X_odom(:, i-1);

            % Predicted pose from previous
            X_pred_from_prev = X_opt(:, i-1) + U_prev;

            % Predicted pose from next (backward)
            U_next = X_odom(:, i+1) - X_odom(:, i);
            X_pred_from_next = X_opt(:, i+1) - U_next;

            % Weighted average: Eq. (34)
            w_prev = Omega(i-1);
            w_curr = Omega(i);
            w_next = Omega(i+1);
            w_total = w_prev + w_curr + w_next;

            X_opt(:, i) = (w_prev * X_pred_from_prev + ...
                          w_curr * X_est(:, i) + ...
                          w_next * X_pred_from_next) / w_total;
        end

        % Check convergence
        delta = norm(X_opt - X_prev, 'fro');
        if delta < 1e-6
            break;
        end
    end

    X_RV_opt = X_opt;

    %% Compute Results
    slam_results.X_actual = X_actual;
    slam_results.X_odom = X_odom;
    slam_results.X_optimized = X_opt;
    slam_results.Omega = Omega;
    slam_results.converged_poses = converged_poses;
    slam_results.estimator_residuals = estimator_residuals;

    % Error metrics
    odom_error = zeros(1, I);
    opt_error = zeros(1, I);
    for i = 1:I
        odom_error(i) = norm(X_odom(1:3, i) - X_actual(1:3, i));
        opt_error(i) = norm(X_opt(1:3, i) - X_actual(1:3, i));
    end
    slam_results.odom_error = odom_error;
    slam_results.opt_error = opt_error;
    slam_results.mean_odom_error = mean(odom_error);
    slam_results.mean_opt_error = mean(opt_error);

    fprintf('  Pose graph SLAM: mean odom error = %.3f m, optimized = %.3f m\n', ...
        slam_results.mean_odom_error, slam_results.mean_opt_error);
end
