function plot_localization_results(results, params, env)
% PLOT_LOCALIZATION_RESULTS  Plot particle filtering and estimator convergence
%
% Reproduces Fig. 4:
%   (a) Particle filtering initialization
%   (b) Convergence of estimated pose to actual pose

    p_RV_actual = env.p_RV_actual;

    %% ===== Fig. 4(a): Particle Filtering Initialization =====
    figure('Name', 'Fig. 4(a) - Particle Filtering', 'Position', [100 100 700 600]);

    % Plot ground
    fill([env.area_x(1) env.area_x(2) env.area_x(2) env.area_x(1)], ...
         [env.area_y(1) env.area_y(1) env.area_y(2) env.area_y(2)], ...
         [0.9 0.9 0.85], 'EdgeColor', 'none');
    hold on;

    % Plot buildings (top view)
    for b = 1:size(env.buildings, 1)
        bld = env.buildings(b, :);
        cx = bld(1); cy = bld(2); w = bld(3); d = bld(4);
        rectangle('Position', [cx-w/2, cy-d/2, w, d], ...
            'FaceColor', [0.7 0.75 0.8], 'EdgeColor', [0.4 0.4 0.4]);
    end

    % Plot BSs
    plot(env.p_BS_Tx(1), env.p_BS_Tx(2), 'k^', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'k', 'LineWidth', 2);
    plot(env.p_BS_Rx(1), env.p_BS_Rx(2), 'k^', 'MarkerSize', 14, ...
        'MarkerFaceColor', [0.4 0.4 0.4], 'LineWidth', 2);

    % Plot random poses (particles)
    particles = results.pf.particles;
    plot(particles(1, :), particles(2, :), 'r.', 'MarkerSize', 8);

    % Plot actual RV pose
    plot(p_RV_actual(1), p_RV_actual(2), 'gd', 'MarkerSize', 14, ...
        'MarkerFaceColor', 'g', 'LineWidth', 2);

    % Plot estimated (best particle)
    plot(results.p_RV_init(1), results.p_RV_init(2), 'ms', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'm', 'LineWidth', 2);

    xlabel('X (meters)');
    ylabel('Y (meters)');
    title(sprintf('(a) Particle Filtering Initialization (error = %.2f m)', results.pf.init_error));
    legend('Ground', '', 'mMIMO BS_{Tx}', 'mMIMO BS_{Rx}', ...
        'RV Random Pose', 'RV Actual Pose', 'RV Estimated Pose', ...
        'Location', 'southwest');
    grid on;
    axis equal;
    xlim(env.area_x);
    ylim(env.area_y);

    saveas(gcf, 'fig4a_particle_filtering.png');
    fprintf('  Saved: fig4a_particle_filtering.png\n');

    %% ===== Fig. 4(b): Estimator Convergence =====
    figure('Name', 'Fig. 4(b) - Estimator Convergence', 'Position', [100 100 1200 500]);

    % Top subplot: XY convergence
    subplot(1, 2, 1);
    history = results.est.history;

    % Actual pose
    plot(p_RV_actual(1), p_RV_actual(2), 'bs', 'MarkerSize', 12, ...
        'MarkerFaceColor', 'b', 'LineWidth', 2);
    hold on;

    % Color by stage
    colors_stage = {'r', [0 0.7 0], [0 0.5 1]};
    stage_names = {'Estimator_{inc}', 'Estimator_{ref}', 'Estimator_{\theta}'};
    markers = {'o', 's', 'd'};

    for s = 1:3
        idx = find(history.stage == s);
        if ~isempty(idx)
            plot(history.p_RV(1, idx), history.p_RV(2, idx), ...
                [markers{s}], 'Color', colors_stage{s}, 'MarkerSize', 8, ...
                'MarkerFaceColor', colors_stage{s}, 'LineWidth', 1.5);
        end
    end

    % Initial estimate
    plot(results.p_RV_init(1), results.p_RV_init(2), 'mp', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'm', 'LineWidth', 2);

    % Final estimate
    plot(results.p_RV_final(1), results.p_RV_final(2), 'c^', ...
        'MarkerSize', 14, 'MarkerFaceColor', 'c', 'LineWidth', 2);

    xlabel('X (meters)');
    ylabel('Y (meters)');
    title(sprintf('Convergence: XY Plane (final error = %.2f m)', results.est.final_error));
    legend('RV Pose Actual', stage_names{:}, ...
        'RV Pose Estimated_{init}', 'RV Pose Estimated_{final}', ...
        'Location', 'best');
    grid on;
    axis equal;

    % Bottom subplot: Signal strength vs iteration
    subplot(1, 2, 2);
    n_total = results.est.total_iterations;
    iters = 0:(n_total - 1);

    plot(iters, history.signal_strength, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
    hold on;

    % Mark stage boundaries
    stage1_end = results.est.stage1_iters;
    stage2_end = stage1_end + results.est.stage2_iters;

    if stage1_end > 0 && stage1_end <= n_total
        xline(stage1_end, '--r', 'Stage 1 \rightarrow 2', 'LineWidth', 1.5, 'FontSize', 10);
    end
    if stage2_end > 0 && stage2_end <= n_total
        xline(stage2_end, '--g', 'Stage 2 \rightarrow 3', 'LineWidth', 1.5, 'FontSize', 10);
    end

    xlabel('Localization Iteration');
    ylabel('Signal Strength (dBm)');
    title('Signal Strength Convergence');
    grid on;

    sgtitle('Fig. 4(b): Convergence of Estimated Pose to Actual Pose');

    saveas(gcf, 'fig4b_estimator_convergence.png');
    fprintf('  Saved: fig4b_estimator_convergence.png\n');

    %% ===== Localization Error vs Iteration =====
    figure('Name', 'Localization Error', 'Position', [100 100 600 400]);

    plot(iters, history.error, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
    hold on;
    yline(0.2, '--r', '0.2 m target', 'LineWidth', 1.5);
    xlabel('Localization Iteration');
    ylabel('Localization Error (m)');
    title('Position Error Convergence');
    grid on;

    saveas(gcf, 'fig4_error_convergence.png');
    fprintf('  Saved: fig4_error_convergence.png\n');

    %% ===== Lyapunov Surface =====
    figure('Name', 'Lyapunov Stability', 'Position', [100 100 1000 400]);

    subplot(1, 2, 1);
    surf(results.lyapunov.x1_range, results.lyapunov.x2_range, ...
        results.lyapunov.V_surface', 'EdgeColor', 'none');
    xlabel('x_1 = \Delta\phi'); ylabel('x_2 = \Delta\zeta');
    zlabel('V(x_1, x_2)');
    title('Lyapunov Function V \geq 0');
    colorbar;
    view([-35 30]);

    subplot(1, 2, 2);
    surf(results.lyapunov.x1_range, results.lyapunov.x2_range, ...
        results.lyapunov.V_dot_surface', 'EdgeColor', 'none');
    xlabel('x_1 = \Delta\phi'); ylabel('x_2 = \Delta\zeta');
    zlabel('dV/dt');
    title('Lyapunov Derivative \dot{V} \leq 0');
    colorbar;
    view([-35 30]);

    sgtitle(sprintf('Lyapunov Stability Analysis (Stable: %s)', ...
        string(results.lyapunov.is_stable)));

    saveas(gcf, 'fig_lyapunov_stability.png');
    fprintf('  Saved: fig_lyapunov_stability.png\n');
end
