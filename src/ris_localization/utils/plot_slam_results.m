function plot_slam_results(results)
% PLOT_SLAM_RESULTS  Plot pose graph SLAM with virtual loop closure results

    figure('Name', 'Pose Graph SLAM', 'Position', [100 100 1200 500]);

    %% Subplot 1: Trajectory comparison (top view)
    subplot(1, 2, 1);

    % Ground truth
    plot(results.X_actual(1, :), results.X_actual(2, :), 'b-o', ...
        'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
    hold on;

    % Odometry (with drift)
    plot(results.X_odom(1, :), results.X_odom(2, :), 'r--s', ...
        'LineWidth', 1.5, 'MarkerSize', 4, 'MarkerFaceColor', 'r');

    % Optimized
    plot(results.X_optimized(1, :), results.X_optimized(2, :), 'g-^', ...
        'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'g');

    % Mark converged (loop closure) poses
    conv_idx = find(results.converged_poses);
    plot(results.X_actual(1, conv_idx), results.X_actual(2, conv_idx), ...
        'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y', 'LineWidth', 2);

    xlabel('X (meters)');
    ylabel('Y (meters)');
    title('Pose Graph: Trajectory Comparison');
    legend('Ground Truth', 'Odometry (with drift)', 'Optimized (RIS-SLAM)', ...
        'Virtual Loop Closures', 'Location', 'best');
    grid on;
    axis equal;

    %% Subplot 2: Error comparison
    subplot(1, 2, 2);

    pose_indices = 1:length(results.odom_error);
    plot(pose_indices, results.odom_error, 'r-', 'LineWidth', 1.5);
    hold on;
    plot(pose_indices, results.opt_error, 'g-', 'LineWidth', 1.5);

    % Mark converged poses
    plot(conv_idx, results.opt_error(conv_idx), 'kp', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 2);

    xlabel('Pose Index');
    ylabel('Position Error (m)');
    title('Localization Error: Odometry vs. RIS-optimized');
    legend('Odometry Error', 'Optimized Error', 'Loop Closures', ...
        'Location', 'best');
    grid on;

    sgtitle(sprintf('Pose Graph SLAM: Odometry RMSE = %.3f m, Optimized RMSE = %.3f m', ...
        results.mean_odom_error, results.mean_opt_error));

    saveas(gcf, 'fig_pose_graph_slam.png');
    fprintf('  Saved: fig_pose_graph_slam.png\n');

    %% Weights visualization
    figure('Name', 'Estimator Weights', 'Position', [100 100 600 400]);

    bar(pose_indices, results.Omega, 0.6, 'FaceColor', [0.3 0.6 0.9]);
    hold on;
    plot(conv_idx, results.Omega(conv_idx), 'rp', 'MarkerSize', 12, ...
        'MarkerFaceColor', 'r', 'LineWidth', 2);
    xlabel('Pose Index');
    ylabel('\Omega_i');
    title('Estimator Convergence Weights (Eq. 30)');
    legend('Weight \Omega_i', 'Converged Poses', 'Location', 'best');
    grid on;

    saveas(gcf, 'fig_weights.png');
    fprintf('  Saved: fig_weights.png\n');
end
