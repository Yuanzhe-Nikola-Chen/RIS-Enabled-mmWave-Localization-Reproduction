function plot_beam_steering_results(results, params, env)
% PLOT_BEAM_STEERING_RESULTS  Plot beam steering and signal strength
%
% Reproduces aspects of Fig. 3(d): geometry-based beam steering

    figure('Name', 'Beam Steering Results', 'Position', [100 100 1200 500]);

    %% Subplot 1: Signal strength vs azimuth offset
    subplot(1, 2, 1);
    plot(results.phi_offsets, results.ss_vs_phi, 'b-', 'LineWidth', 1.5);
    hold on;
    [max_ss, max_idx] = max(results.ss_vs_phi);
    plot(results.phi_offsets(max_idx), max_ss, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('Azimuth Offset (degrees)');
    ylabel('Signal Strength (dBm)');
    title('Signal Strength vs. Azimuth Mismatch');
    grid on;
    legend('Signal Strength', 'Maximum', 'Location', 'best');

    %% Subplot 2: Signal strength vs elevation offset
    subplot(1, 2, 2);
    plot(results.zeta_offsets, results.ss_vs_zeta, 'r-', 'LineWidth', 1.5);
    hold on;
    [max_ss_z, max_idx_z] = max(results.ss_vs_zeta);
    plot(results.zeta_offsets(max_idx_z), max_ss_z, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    xlabel('Elevation Offset (degrees)');
    ylabel('Signal Strength (dBm)');
    title('Signal Strength vs. Elevation Mismatch');
    grid on;
    legend('Signal Strength', 'Maximum', 'Location', 'best');

    sgtitle(sprintf('Beam Steering: With BF = %.1f dBm, Without BF = %.1f dBm (Gain: %.1f dB)', ...
        results.ss_with_BF, results.ss_without_BF, results.ss_with_BF - results.ss_without_BF));

    saveas(gcf, 'fig3d_beam_steering.png');
    fprintf('  Saved: fig3d_beam_steering.png\n');

    %% Figure: 3D Urban Scene (Fig. 3 overview)
    figure('Name', 'Urban Environment', 'Position', [100 100 900 700]);

    % Plot buildings
    for b = 1:size(env.buildings, 1)
        bld = env.buildings(b, :);
        cx = bld(1); cy = bld(2); w = bld(3); d = bld(4); h = bld(5);
        draw_box(cx - w/2, cy - d/2, 0, w, d, h, [0.6 0.7 0.8], 0.4);
    end
    hold on;

    % Plot ground plane
    fill3([env.area_x(1) env.area_x(2) env.area_x(2) env.area_x(1)], ...
          [env.area_y(1) env.area_y(1) env.area_y(2) env.area_y(2)], ...
          [0 0 0 0], [0.85 0.85 0.8], 'FaceAlpha', 0.3);

    % Plot BSs
    plot3(env.p_BS_Tx(1), env.p_BS_Tx(2), env.p_BS_Tx(3), ...
        'k^', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'LineWidth', 2);
    text(env.p_BS_Tx(1)+2, env.p_BS_Tx(2), env.p_BS_Tx(3)+3, 'BS_{Tx}', 'FontSize', 12, 'FontWeight', 'bold');

    plot3(env.p_BS_Rx(1), env.p_BS_Rx(2), env.p_BS_Rx(3), ...
        'k^', 'MarkerSize', 15, 'MarkerFaceColor', [0.3 0.3 0.3], 'LineWidth', 2);
    text(env.p_BS_Rx(1)+2, env.p_BS_Rx(2), env.p_BS_Rx(3)+3, 'BS_{Rx}', 'FontSize', 12, 'FontWeight', 'bold');

    % Plot RV
    plot3(env.p_RV_actual(1), env.p_RV_actual(2), env.p_RV_actual(3), ...
        'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    text(env.p_RV_actual(1)+2, env.p_RV_actual(2), env.p_RV_actual(3)+2, 'RV', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');

    % Plot LoS paths
    % BSTx -> RV (blue)
    plot3([env.p_BS_Tx(1) env.p_RV_actual(1)], ...
          [env.p_BS_Tx(2) env.p_RV_actual(2)], ...
          [env.p_BS_Tx(3) env.p_RV_actual(3)], ...
          'b-', 'LineWidth', 2);
    % RV -> BSRx (green)
    plot3([env.p_RV_actual(1) env.p_BS_Rx(1)], ...
          [env.p_RV_actual(2) env.p_BS_Rx(2)], ...
          [env.p_RV_actual(3) env.p_BS_Rx(3)], ...
          'g-', 'LineWidth', 2);

    xlabel('X (meters)'); ylabel('Y (meters)'); zlabel('Z (meters)');
    title('Dense Urban Environment with Cascaded BS_{Tx} - RV - BS_{Rx} Path');
    legend('', '', 'mMIMO BS_{Tx}', '', 'mMIMO BS_{Rx}', '', 'RV (RIS)', ...
        'Incident Path', 'Reflected Path', 'Location', 'best');
    view([-35 25]);
    grid on;
    axis equal;

    saveas(gcf, 'fig3_urban_environment.png');
    fprintf('  Saved: fig3_urban_environment.png\n');
end


function draw_box(x, y, z, w, d, h, color, alpha)
% Draw a 3D box (building)
    vertices = [
        x,   y,   z;
        x+w, y,   z;
        x+w, y+d, z;
        x,   y+d, z;
        x,   y,   z+h;
        x+w, y,   z+h;
        x+w, y+d, z+h;
        x,   y+d, z+h;
    ];
    faces = [
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8;
        1 2 3 4;
        5 6 7 8;
    ];
    patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', color, 'FaceAlpha', alpha, 'EdgeColor', [0.3 0.3 0.3]);
end
