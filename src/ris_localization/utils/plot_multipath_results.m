function plot_multipath_results(results)
% PLOT_MULTIPATH_RESULTS  Plot multipath impact on localization
%
% Reproduces Fig. 5(b): localization error with increasing number of rays

    figure('Name', 'Fig. 5(b) - Multipath Impact', 'Position', [100 100 700 500]);

    markers = {'-^', '-o', '-s', '-*', '-d'};
    colors = {[1 0 0], [0 0 1], [0 0.6 0], [0.8 0.5 0], [0.5 0 0.8]};
    line_styles = {'--', '-', '-.', ':', '-'};

    max_iter_display = min(25, results.max_display_iter + 5);
    legend_entries = cell(1, length(results.num_rays_list));

    for r = 1:length(results.num_rays_list)
        n_iters = results.total_iters(r);
        n_plot = min(n_iters, max_iter_display);
        iters = 0:(n_plot - 1);

        error_data = results.error_vs_iter(r, 1:n_plot);

        plot(iters, error_data, markers{r}, ...
            'Color', colors{r}, 'LineWidth', 1.5, 'MarkerSize', 7, ...
            'MarkerFaceColor', 'none');
        hold on;

        legend_entries{r} = sprintf('%d rays', results.num_rays_list(r));
    end

    xlabel('Iter');
    ylabel('Localization error (m)');
    title('Localization Performance Under Imperfect Conditions');
    legend(legend_entries, 'Location', 'northeast');
    grid on;
    xlim([0 max_iter_display]);
    ylim([0 max(results.error_vs_iter(:, 1)) * 1.1]);

    saveas(gcf, 'fig5b_multipath_impact.png');
    fprintf('  Saved: fig5b_multipath_impact.png\n');

    %% Summary bar chart
    figure('Name', 'Multipath Final Errors', 'Position', [100 100 500 400]);
    bar(results.num_rays_list, results.final_errors, 0.5, 'FaceColor', [0.3 0.5 0.8]);
    xlabel('Number of Rays');
    ylabel('Final Localization Error (m)');
    title('Final Error vs. Number of Multipath Rays');
    grid on;

    saveas(gcf, 'fig5_multipath_bar.png');
    fprintf('  Saved: fig5_multipath_bar.png\n');
end
