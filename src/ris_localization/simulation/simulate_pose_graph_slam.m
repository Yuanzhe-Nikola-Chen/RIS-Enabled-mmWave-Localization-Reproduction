function results = simulate_pose_graph_slam(params, env)
% SIMULATE_POSE_GRAPH_SLAM  Run pose graph SLAM with virtual loop closure
%
% Uses the vehicle trajectory and RIS-enabled localization to:
%   1. Generate odometry with drift
%   2. Perform RIS-enabled localization at stationary points
%   3. Use virtual loop closures to correct drift
%   4. Optimize pose graph

    trajectory = env.trajectory;
    p_BS_Tx = env.p_BS_Tx;
    p_BS_Rx = env.p_BS_Rx;

    %% Subsample trajectory for pose graph nodes
    % Select every Nth point as a pose graph node
    subsample_rate = 10;
    indices = 1:subsample_rate:trajectory.N_poses;
    I = length(indices);

    p_RV_actual_traj = zeros(3, I);
    theta_RV_actual_traj = zeros(1, I);

    for i = 1:I
        idx = indices(i);
        p_RV_actual_traj(:, i) = [trajectory.x(idx); trajectory.y(idx); trajectory.z(idx)];
        theta_RV_actual_traj(i) = trajectory.theta(idx);
    end

    %% Run Pose Graph Optimization
    [X_RV_opt, slam_results] = pose_graph_optimization(...
        trajectory, p_BS_Tx, p_BS_Rx, p_RV_actual_traj, ...
        theta_RV_actual_traj, params);

    %% Add trajectory info
    results = slam_results;
    results.indices = indices;
    results.trajectory = trajectory;
    results.I = I;

    fprintf('  Pose graph SLAM complete:\n');
    fprintf('    %d poses, mean odom error: %.3f m, optimized: %.3f m\n', ...
        I, results.mean_odom_error, results.mean_opt_error);
end
