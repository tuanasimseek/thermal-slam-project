function plot_traj(poses, optimizedNodes, frameIdx, options)
% PLOT_TRAJ
% Ham trajectory ve optimize edilmiş graph sonucunu çizer.
% PNG olarak düzgün isimle kaydedebilir.

if nargin < 4
    options = struct();
end

if ~isfield(options, 'title')
    options.title = 'Thermal SLAM Trajectory';
end

if ~isfield(options, 'saveDir')
    options.saveDir = '';
end

if ~isfield(options, 'fileName')
    options.fileName = sprintf('traj_%05d.png', frameIdx);
end

hasOptimized = ~isempty(optimizedNodes);

figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 500]);

if hasOptimized
    subplot(1, 2, 1);
end

plot(poses(:,1), poses(:,2), 'b-', 'LineWidth', 1.5);
hold on;

plot(poses(1,1), poses(1,2), 'go', ...
    'MarkerSize', 9, 'MarkerFaceColor', 'g');

plot(poses(end,1), poses(end,2), 'rs', ...
    'MarkerSize', 9, 'MarkerFaceColor', 'r');

xlabel('X');
ylabel('Y');
title('Ham Trajectory');
legend('Ham yol', 'Başlangıç', 'Bitiş', 'Location', 'best');
grid on;
axis equal;

if hasOptimized
    subplot(1, 2, 2);

    plot(poses(:,1), poses(:,2), '--', ...
        'LineWidth', 1.0, 'Color', [0.6 0.6 1.0]);
    hold on;

    plot(optimizedNodes(:,1), optimizedNodes(:,2), 'r-', ...
        'LineWidth', 2.0);

    scatter(optimizedNodes(:,1), optimizedNodes(:,2), ...
        35, 'r', 'filled');

    xlabel('X');
    ylabel('Y');
    title('Optimize Edilmiş Trajectory');
    legend('Ham yol', 'Optimize yol', 'Keyframe', 'Location', 'best');
    grid on;
    axis equal;
end

sgtitle(options.title, 'FontWeight', 'bold');

if ~isempty(options.saveDir)

    if ~isfolder(options.saveDir)
        mkdir(options.saveDir);
    end

    savePath = fullfile(options.saveDir, options.fileName);

    exportgraphics(gcf, savePath, 'Resolution', 300);

    fprintf("PNG kaydedildi: %s\n", savePath);
end

close(gcf);

end