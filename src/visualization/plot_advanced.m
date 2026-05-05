function plot_advanced(trajFile, graphFile, metricsFile)
% PLOT_ADVANCED  Gelismis SLAM gorsellestiricisi
%
% Kullanim:
%   plot_advanced('results/set00_V000_traj.mat', 'results/set00_V000_graph.mat')
%   plot_advanced('results/set00_V000_traj.mat', 'results/set00_V000_graph.mat', 'results/set00_metrics.mat')
%
% Uretilen paneller:
%   Panel 1 — Ziyaret yogunluk haritasi (heatmap)
%   Panel 2 — Trajectory + guven bandi
%   Panel 3 — Loop closure kenarlari
%   Panel 4 — Metrik ozeti (opsiyonel, metricsFile verilirse)
%
% Kaydedilen dosyalar:
%   figures/adv_heatmap.png
%   figures/adv_confidence.png
%   figures/adv_loop_closure.png
%   figures/adv_summary.png  (metricsFile verilirse)

% ---- giris kontrolu ----
if nargin < 2
    error('En az 2 arguman gerekli: trajFile ve graphFile');
end
hasMetics = (nargin == 3);

% ---- veri yukle ----
fprintf('Yukleniyor: %s\n', trajFile);
T = load(trajFile);
trajectory   = T.trajectory;       % [N x 2] x,y

G = load(graphFile);
graph        = G.graph;             % PoseGraph nesnesi
optimized    = G.optimizedNodes;    % [M x 2]
kfFrameIds   = G.keyframeFrameIds;  % [1 x M]

if hasMetics
    M = load(metricsFile);
    metrics = M.metrics;
end

% ---- cikti klasoru ----
[outDir, ~, ~] = fileparts(trajFile);
figDir = fullfile(outDir, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

% ---- isim etiketi (dosya adından cek) ----
[~, baseName, ~] = fileparts(trajFile);
label = strrep(baseName, '_traj', '');

% ==========================================================
% PANEL 1: Ziyaret yogunluk haritasi (Heatmap)
% ==========================================================
fig1 = figure('Name', [label ' — Heatmap'], 'Color', [0.12 0.12 0.15], ...
    'Position', [100 100 700 600]);

ax1 = axes('Parent', fig1);
hold(ax1, 'on');

% histogram2 ile 2D yogunluk
nBins = 60;
h = histogram2(ax1, trajectory(:,1), trajectory(:,2), nBins, ...
    'DisplayStyle', 'tile', 'ShowEmptyBins', 'on', ...
    'EdgeColor', 'none');

% renk haritasi: koyu -> sicak
colormap(ax1, hot);
cb = colorbar(ax1);
cb.Label.String = 'Gecis sayisi';
cb.Color = [0.85 0.85 0.85];
cb.Label.Color = [0.85 0.85 0.85];

% keyframe noktalari uzerine ciz
scatter(ax1, optimized(:,1), optimized(:,2), 50, ...
    'cyan', 'filled', 'MarkerEdgeColor', 'white', 'LineWidth', 0.5);

% baslangic / bitis noktalari
plot(ax1, trajectory(1,1),   trajectory(1,2),   'gs', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'green',  'LineWidth', 1.5);
plot(ax1, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'red',    'LineWidth', 1.5);

legend(ax1, {'Keyframe', 'Baslangic', 'Bitis'}, ...
    'TextColor', 'white', 'Color', [0.2 0.2 0.2], 'Location', 'best');

set(ax1, 'Color', [0.08 0.08 0.10], ...
    'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7], ...
    'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.3);
grid(ax1, 'on');
xlabel(ax1, 'X (m)', 'Color', [0.85 0.85 0.85]);
ylabel(ax1, 'Y (m)', 'Color', [0.85 0.85 0.85]);
title(ax1, [label ' — Ziyaret Yogunlugu'], 'Color', 'white', 'FontSize', 13);
axis(ax1, 'equal');

saveas(fig1, fullfile(figDir, [label '_adv_heatmap.png']));
fprintf('Kaydedildi: %s\n', fullfile(figDir, [label '_adv_heatmap.png']));

% ==========================================================
% PANEL 2: Trajectory + Guven Bandi
% ==========================================================
fig2 = figure('Name', [label ' — Guven Bandi'], 'Color', [0.12 0.12 0.15], ...
    'Position', [150 100 700 600]);

ax2 = axes('Parent', fig2);
hold(ax2, 'on');

% --- guven bandi hesapla ---
% Her frame icin onceki N frame'in standart sapmasını kullan (kayan pencere)
winSize  = 30;
N        = size(trajectory, 1);
sigma_x  = zeros(N, 1);
sigma_y  = zeros(N, 1);

for i = 1:N
    idx_start = max(1, i - winSize);
    window_x  = trajectory(idx_start:i, 1);
    window_y  = idx_start:i;
    if length(window_x) > 1
        sigma_x(i) = std(trajectory(idx_start:i, 1));
        sigma_y(i) = std(trajectory(idx_start:i, 2));
    end
end

% Ort. sigma (x ve y'nin RMS'i)
sigma = sqrt(sigma_x.^2 + sigma_y.^2);

% Bant: trajectory yonune dik ofset (basitleştirilmis: +-sigma y ekseninde)
upper_x = trajectory(:,1);
upper_y = trajectory(:,2) + sigma;
lower_x = trajectory(:,1);
lower_y = trajectory(:,2) - sigma;

% Guven bandi (fill)
fillX = [upper_x; flipud(lower_x)];
fillY = [upper_y; flipud(lower_y)];
fill(ax2, fillX, fillY, [0.3 0.6 1.0], 'FaceAlpha', 0.18, ...
    'EdgeColor', 'none', 'DisplayName', '\pm\sigma guven bandi');

% Ana trajectory
plot(ax2, trajectory(:,1), trajectory(:,2), '-', ...
    'Color', [0.4 0.8 1.0], 'LineWidth', 1.2, 'DisplayName', 'Trajectory');

% Optimize edilmis keyframe dugumler
plot(ax2, optimized(:,1), optimized(:,2), 'o', ...
    'Color', 'white', 'MarkerFaceColor', [0.9 0.7 0.2], ...
    'MarkerSize', 5, 'LineWidth', 0.8, 'DisplayName', 'Keyframe');

% Baslangic / bitis
plot(ax2, trajectory(1,1),   trajectory(1,2),   'gs', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'green', 'LineWidth', 1.5, ...
    'DisplayName', 'Baslangic');
plot(ax2, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'red',   'LineWidth', 1.5, ...
    'DisplayName', 'Bitis');

legend(ax2, 'TextColor', 'white', 'Color', [0.2 0.2 0.2], 'Location', 'best');
set(ax2, 'Color', [0.08 0.08 0.10], ...
    'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
grid(ax2, 'on');
set(ax2, 'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.3);
xlabel(ax2, 'X (m)', 'Color', [0.85 0.85 0.85]);
ylabel(ax2, 'Y (m)', 'Color', [0.85 0.85 0.85]);
title(ax2, [label ' — Trajectory ve Guven Bandi'], 'Color', 'white', 'FontSize', 13);
axis(ax2, 'equal');

saveas(fig2, fullfile(figDir, [label '_adv_confidence.png']));
fprintf('Kaydedildi: %s\n', fullfile(figDir, [label '_adv_confidence.png']));

% ==========================================================
% PANEL 3: Loop Closure Kenarlari
% ==========================================================
fig3 = figure('Name', [label ' — Loop Closure'], 'Color', [0.12 0.12 0.15], ...
    'Position', [200 100 700 600]);

ax3 = axes('Parent', fig3);
hold(ax3, 'on');

% Ana trajectory (soluk)
plot(ax3, trajectory(:,1), trajectory(:,2), '-', ...
    'Color', [0.5 0.5 0.6], 'LineWidth', 0.8, 'DisplayName', 'Trajectory');

% Optimize edilmis dugumler
scatter(ax3, optimized(:,1), optimized(:,2), 30, ...
    'white', 'filled', 'MarkerEdgeColor', [0.6 0.6 0.6], ...
    'LineWidth', 0.5, 'DisplayName', 'Keyframe');

% --- Loop closure kenarlarini cizdirme ---
% graph.edges: [i, j, dx, dy, weight]
% Sadece ardisik olmayan kenarlari goster (loop kenarlari)
edges = graph.edges;

loopCount = 0;
for e = 1:size(edges, 1)
    ni = edges(e, 1);   % kaynak node id
    nj = edges(e, 2);   % hedef node id

    % Ardisik olmayan (fark > 1) kenarlar loop closure'dir
    if abs(nj - ni) > 1
        % Node id'leri indekse cevir
        idx_i = find(kfFrameIds == ni, 1);
        idx_j = find(kfFrameIds == nj, 1);

        if ~isempty(idx_i) && ~isempty(idx_j)
            x_pts = [optimized(idx_i, 1), optimized(idx_j, 1)];
            y_pts = [optimized(idx_i, 2), optimized(idx_j, 2)];

            if loopCount == 0
                plot(ax3, x_pts, y_pts, '--', ...
                    'Color', [1.0 0.4 0.2], 'LineWidth', 1.5, ...
                    'DisplayName', 'Loop closure kenari');
            else
                plot(ax3, x_pts, y_pts, '--', ...
                    'Color', [1.0 0.4 0.2], 'LineWidth', 1.5, ...
                    'HandleVisibility', 'off');
            end

            % Baglanan noktalari vurgula
            plot(ax3, optimized(idx_i, 1), optimized(idx_i, 2), 'o', ...
                'Color', [1.0 0.6 0.2], 'MarkerFaceColor', [1.0 0.6 0.2], ...
                'MarkerSize', 7, 'HandleVisibility', 'off');
            plot(ax3, optimized(idx_j, 1), optimized(idx_j, 2), 'o', ...
                'Color', [1.0 0.6 0.2], 'MarkerFaceColor', [1.0 0.6 0.2], ...
                'MarkerSize', 7, 'HandleVisibility', 'off');

            loopCount = loopCount + 1;
        end
    end
end

% Eger hic loop kenari yoksa not yaz
if loopCount == 0
    text(ax3, mean(xlim(ax3)), mean(ylim(ax3)), ...
        'Bu videoda loop closure kenari yok', ...
        'Color', [0.8 0.8 0.4], 'FontSize', 11, ...
        'HorizontalAlignment', 'center');
    fprintf('Not: %s icin loop closure kenari bulunamadi.\n', label);
end

% Baslangic / bitis
plot(ax3, trajectory(1,1),   trajectory(1,2),   'gs', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'green', 'LineWidth', 1.5, ...
    'DisplayName', 'Baslangic');
plot(ax3, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'red',   'LineWidth', 1.5, ...
    'DisplayName', 'Bitis');

legend(ax3, 'TextColor', 'white', 'Color', [0.2 0.2 0.2], 'Location', 'best');
set(ax3, 'Color', [0.08 0.08 0.10], ...
    'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
grid(ax3, 'on');
set(ax3, 'GridColor', [0.3 0.3 0.3], 'GridAlpha', 0.3);
xlabel(ax3, 'X (m)', 'Color', [0.85 0.85 0.85]);
ylabel(ax3, 'Y (m)', 'Color', [0.85 0.85 0.85]);
title(ax3, sprintf('%s — Loop Closure Kenarlari (%d adet)', label, loopCount), ...
    'Color', 'white', 'FontSize', 13);
axis(ax3, 'equal');

saveas(fig3, fullfile(figDir, [label '_adv_loop_closure.png']));
fprintf('Kaydedildi: %s\n', fullfile(figDir, [label '_adv_loop_closure.png']));

% ==========================================================
% PANEL 4: Metrik Ozeti (opsiyonel)
% ==========================================================
if hasMetics
    fig4 = figure('Name', [label ' — Metrik Ozeti'], 'Color', [0.12 0.12 0.15], ...
        'Position', [250 100 800 500]);
    ax4 = axes('Parent', fig4);

    % metrics struct array'inden tablo olustur
    videoNames   = {metrics.videoName};
    nFrames      = [metrics.nFrames];
    pathLengths  = [metrics.pathLength];
    meanConf     = [metrics.meanConfidence];
    nEdges       = [metrics.nEdges];

    % 4 alt grafik: path length, meanConfidence, nEdges, nFrames
    nVid = numel(metrics);
    xTicks = 1:nVid;
    shortNames = cellfun(@(n) strrep(n, 'set00_', ''), videoNames, 'UniformOutput', false);

    delete(ax4);

    % -- Path Length --
    ax4a = subplot(2, 2, 1, 'Parent', fig4);
    bar(ax4a, xTicks, pathLengths, 'FaceColor', [0.3 0.7 1.0], 'EdgeColor', 'none');
    set(ax4a, 'XTick', xTicks, 'XTickLabel', shortNames, ...
        'XTickLabelRotation', 45, 'Color', [0.08 0.08 0.10], ...
        'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    grid(ax4a, 'on'); grid(ax4a, 'minor');
    ylabel(ax4a, 'Metre', 'Color', [0.85 0.85 0.85]);
    title(ax4a, 'Yol Uzunlugu', 'Color', 'white');

    % -- Mean Confidence --
    ax4b = subplot(2, 2, 2, 'Parent', fig4);
    bar(ax4b, xTicks, meanConf, 'FaceColor', [0.3 1.0 0.5], 'EdgeColor', 'none');
    yline(ax4b, mean(meanConf), '--w', 'LineWidth', 1, 'Label', 'Ortalama');
    set(ax4b, 'XTick', xTicks, 'XTickLabel', shortNames, ...
        'XTickLabelRotation', 45, 'Color', [0.08 0.08 0.10], ...
        'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    grid(ax4b, 'on');
    ylabel(ax4b, 'Guven', 'Color', [0.85 0.85 0.85]);
    title(ax4b, 'Ortalama Guven Skoru', 'Color', 'white');

    % -- nEdges --
    ax4c = subplot(2, 2, 3, 'Parent', fig4);
    bar(ax4c, xTicks, nEdges, 'FaceColor', [1.0 0.5 0.2], 'EdgeColor', 'none');
    set(ax4c, 'XTick', xTicks, 'XTickLabel', shortNames, ...
        'XTickLabelRotation', 45, 'Color', [0.08 0.08 0.10], ...
        'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    grid(ax4c, 'on');
    ylabel(ax4c, 'Kenar Sayisi', 'Color', [0.85 0.85 0.85]);
    title(ax4c, 'Graf Kenar Sayisi', 'Color', 'white');

    % -- nFrames (used) --
    ax4d = subplot(2, 2, 4, 'Parent', fig4);
    usedCount  = [metrics.usedCount];
    invalidCnt = [metrics.invalidCount];
    bar(ax4d, xTicks, [usedCount; invalidCnt]', 'stacked', 'EdgeColor', 'none');
    colororder(ax4d, [[0.3 0.7 1.0]; [0.9 0.3 0.3]]);
    legend(ax4d, {'Gecerli frame', 'Gecersiz frame'}, ...
        'TextColor', 'white', 'Color', [0.2 0.2 0.2]);
    set(ax4d, 'XTick', xTicks, 'XTickLabel', shortNames, ...
        'XTickLabelRotation', 45, 'Color', [0.08 0.08 0.10], ...
        'XColor', [0.7 0.7 0.7], 'YColor', [0.7 0.7 0.7]);
    grid(ax4d, 'on');
    ylabel(ax4d, 'Frame Sayisi', 'Color', [0.85 0.85 0.85]);
    title(ax4d, 'Gecerli / Gecersiz Frame', 'Color', 'white');

    sgtitle(fig4, [label ' — Metrik Ozeti'], 'Color', 'white', 'FontSize', 14);

    set(fig4, 'Color', [0.12 0.12 0.15]);

    saveas(fig4, fullfile(figDir, [label '_adv_metrics.png']));
    fprintf('Kaydedildi: %s\n', fullfile(figDir, [label '_adv_metrics.png']));
end

% ---- konsol ozet ----
fprintf('\n========== %s OZET ==========\n', label);
fprintf('Trajectory uzunlugu : %d frame\n', size(trajectory, 1));
fprintf('Keyframe sayisi     : %d\n', size(optimized, 1));
fprintf('Graf kenar sayisi   : %d\n', size(edges, 1));
fprintf('Loop closure kenari : %d\n', loopCount);
if hasMetics && isfield(metrics(1), 'meanConfidence')
    allConf = [metrics.meanConfidence];
    fprintf('Ort. guven skoru    : %.4f\n', mean(allConf));
end
fprintf('================================\n\n');

end