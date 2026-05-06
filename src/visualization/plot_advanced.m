function plot_advanced(trajFile, graphFile, metricsFile, saveDir, methodName)
% PLOT_ADVANCED
% Heatmap, guven bandi, loop closure ve opsiyonel metrik grafikleri uretir.
%
% Kullanim:
%   plot_advanced(trajPath, graphPath, [], saveDir, 'cnn')
%   plot_advanced(trajPath, graphPath, metricPath, saveDir, 'ssd')

if nargin < 2
    error('En az 2 arguman gerekli: trajFile ve graphFile');
end

if nargin < 3
    metricsFile = [];
end

if nargin < 4 || isempty(saveDir)
    saveDir = fullfile(pwd, 'results_final', 'figures', 'advanced');
end

if nargin < 5 || isempty(methodName)
    methodName = 'default';
end

hasMetrics = ~isempty(metricsFile) && exist(metricsFile, 'file');

%% VERI YUKLE
fprintf('Yukleniyor: %s\n', trajFile);

T = load(trajFile);
trajectory = T.trajectory;

G = load(graphFile);
graph = G.graph;
optimized = G.optimizedNodes;

if isfield(G, 'keyframeFrameIds')
    kfFrameIds = G.keyframeFrameIds;
else
    kfFrameIds = 1:size(optimized,1);
end

if hasMetrics
    M = load(metricsFile);
    metrics = M.metrics;
end

%% CIKTI KLASORLERI
heatmapDir    = fullfile(saveDir, 'heatmap');
confidenceDir = fullfile(saveDir, 'confidence');
loopDir       = fullfile(saveDir, 'loop_closure');
metricsDir    = fullfile(saveDir, 'metrics');

dirs = {heatmapDir, confidenceDir, loopDir, metricsDir};

for i = 1:numel(dirs)
    if ~exist(dirs{i}, 'dir')
        mkdir(dirs{i});
    end
end

[~, baseName, ~] = fileparts(trajFile);
label = strrep(baseName, '_traj', '');

if ~contains(label, methodName)
    label = sprintf('%s_%s', label, methodName);
end

%% ==========================================================
% PANEL 1: HEATMAP
% ==========================================================
fig1 = figure('Visible','off','Name',[label ' Heatmap'], ...
    'Color',[0.12 0.12 0.15], 'Position',[100 100 800 650]);

ax1 = axes('Parent', fig1);
hold(ax1, 'on');

histogram2(ax1, trajectory(:,1), trajectory(:,2), 60, ...
    'DisplayStyle','tile', 'ShowEmptyBins','on', 'EdgeColor','none');

colormap(ax1, hot);

cb = colorbar(ax1);
cb.Label.String = 'Gecis sayisi';
cb.Color = [0.85 0.85 0.85];
cb.Label.Color = [0.85 0.85 0.85];

scatter(ax1, optimized(:,1), optimized(:,2), 45, ...
    'cyan', 'filled', 'MarkerEdgeColor','white', 'LineWidth',0.5);

plot(ax1, trajectory(1,1), trajectory(1,2), 'gs', ...
    'MarkerSize',10, 'MarkerFaceColor','green', 'LineWidth',1.5);

plot(ax1, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize',10, 'MarkerFaceColor','red', 'LineWidth',1.5);

legend(ax1, {'Keyframe','Baslangic','Bitis'}, ...
    'TextColor','white', 'Color',[0.2 0.2 0.2], 'Location','best');

styleAx(ax1);

xlabel(ax1,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax1,'Y (m)','Color',[0.85 0.85 0.85]);
title(ax1,[label ' — Ziyaret Yogunlugu'], 'Color','white', 'FontSize',13);
axis(ax1,'equal');

out1 = fullfile(heatmapDir, [label '_adv_heatmap.png']);
exportgraphics(fig1, out1, 'Resolution', 300);
fprintf('Kaydedildi: %s\n', out1);
close(fig1);

%% ==========================================================
% PANEL 2: GUVEN BANDI
% ==========================================================
fig2 = figure('Visible','off','Name',[label ' Guven Bandi'], ...
    'Color',[0.12 0.12 0.15], 'Position',[100 100 800 650]);

ax2 = axes('Parent', fig2);
hold(ax2, 'on');

winSize = 30;
N = size(trajectory, 1);
sigma = zeros(N, 1);

for i = 1:N
    i0 = max(1, i - winSize);
    w = trajectory(i0:i, :);

    if size(w,1) > 1
        sigma(i) = sqrt(var(w(:,1)) + var(w(:,2)));
    end
end

fillX = [trajectory(:,1); flipud(trajectory(:,1))];
fillY = [trajectory(:,2) + sigma; flipud(trajectory(:,2) - sigma)];

fill(ax2, fillX, fillY, [0.3 0.6 1.0], ...
    'FaceAlpha',0.18, 'EdgeColor','none', ...
    'DisplayName','Guven bandi');

plot(ax2, trajectory(:,1), trajectory(:,2), '-', ...
    'Color',[0.4 0.8 1.0], 'LineWidth',1.2, ...
    'DisplayName','Trajectory');

plot(ax2, optimized(:,1), optimized(:,2), 'o', ...
    'Color','white', 'MarkerFaceColor',[0.9 0.7 0.2], ...
    'MarkerSize',5, 'LineWidth',0.8, ...
    'DisplayName','Keyframe');

plot(ax2, trajectory(1,1), trajectory(1,2), 'gs', ...
    'MarkerSize',10, 'MarkerFaceColor','green', ...
    'LineWidth',1.5, 'DisplayName','Baslangic');

plot(ax2, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize',10, 'MarkerFaceColor','red', ...
    'LineWidth',1.5, 'DisplayName','Bitis');

legend(ax2, 'TextColor','white', 'Color',[0.2 0.2 0.2], 'Location','best');

styleAx(ax2);

xlabel(ax2,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax2,'Y (m)','Color',[0.85 0.85 0.85]);
title(ax2,[label ' — Trajectory ve Guven Bandi'], 'Color','white', 'FontSize',13);
axis(ax2,'equal');

out2 = fullfile(confidenceDir, [label '_adv_confidence.png']);
exportgraphics(fig2, out2, 'Resolution', 300);
fprintf('Kaydedildi: %s\n', out2);
close(fig2);

%% ==========================================================
% PANEL 3: LOOP CLOSURE
% ==========================================================
fig3 = figure('Visible','off','Name',[label ' Loop Closure'], ...
    'Color',[0.12 0.12 0.15], 'Position',[100 100 800 650]);

ax3 = axes('Parent', fig3);
hold(ax3, 'on');

plot(ax3, trajectory(:,1), trajectory(:,2), '-', ...
    'Color',[0.5 0.5 0.6], 'LineWidth',0.8, ...
    'DisplayName','Trajectory');

scatter(ax3, optimized(:,1), optimized(:,2), 30, ...
    'white', 'filled', 'MarkerEdgeColor',[0.6 0.6 0.6], ...
    'LineWidth',0.5, 'DisplayName','Keyframe');

edges = graph.edges;
loopCount = 0;

for e = 1:size(edges, 1)

    ni = edges(e, 1);
    nj = edges(e, 2);

    if abs(nj - ni) > 1 && ni <= size(optimized,1) && nj <= size(optimized,1)

        x_pts = [optimized(ni,1), optimized(nj,1)];
        y_pts = [optimized(ni,2), optimized(nj,2)];

        lbl = 'Loop closure';
        if loopCount > 0
            lbl = '';
        end

        plot(ax3, x_pts, y_pts, '--', ...
            'Color',[1.0 0.4 0.2], 'LineWidth',1.8, ...
            'DisplayName',lbl);

        loopCount = loopCount + 1;
    end
end

M_nodes = size(optimized, 1);
minGap = 5;
distThresh = 0.5;
loopPairs = [];

for i = 1:M_nodes
    for j = i+minGap:M_nodes
        d = norm(optimized(i,:) - optimized(j,:));

        if d < distThresh
            loopPairs(end+1, :) = [i, j, d]; %#ok<AGROW>
        end
    end
end

potCount = size(loopPairs, 1);

for lp = 1:potCount

    i = loopPairs(lp, 1);
    j = loopPairs(lp, 2);

    x_pts = [optimized(i,1), optimized(j,1)];
    y_pts = [optimized(i,2), optimized(j,2)];

    lbl = sprintf('Potansiyel loop < %.1fm', distThresh);
    if lp > 1
        lbl = '';
    end

    plot(ax3, x_pts, y_pts, ':', ...
        'Color',[0.3 1.0 0.6], 'LineWidth',1.5, ...
        'DisplayName',lbl);
end

if loopCount == 0 && potCount == 0
    xlims = xlim(ax3);
    ylims = ylim(ax3);

    text(ax3, mean(xlims), mean(ylims), ...
        'Loop closure yok', ...
        'Color',[0.8 0.8 0.4], ...
        'FontSize',11, ...
        'HorizontalAlignment','center');
end

plot(ax3, trajectory(1,1), trajectory(1,2), 'gs', ...
    'MarkerSize',10, 'MarkerFaceColor','green', ...
    'LineWidth',1.5, 'DisplayName','Baslangic');

plot(ax3, trajectory(end,1), trajectory(end,2), 'r^', ...
    'MarkerSize',10, 'MarkerFaceColor','red', ...
    'LineWidth',1.5, 'DisplayName','Bitis');

legend(ax3, 'TextColor','white', 'Color',[0.2 0.2 0.2], 'Location','best');

styleAx(ax3);

xlabel(ax3,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax3,'Y (m)','Color',[0.85 0.85 0.85]);

title(ax3, sprintf('%s — Loop Closure graph:%d potansiyel:%d', ...
    label, loopCount, potCount), ...
    'Color','white', 'FontSize',13);

axis(ax3,'equal');

out3 = fullfile(loopDir, [label '_adv_loop_closure.png']);
exportgraphics(fig3, out3, 'Resolution', 300);
fprintf('Kaydedildi: %s\n', out3);
close(fig3);

%% ==========================================================
% PANEL 4: METRIK
% ==========================================================
if hasMetrics

    fig4 = figure('Visible','off','Name',[label ' Metrik'], ...
        'Color',[0.12 0.12 0.15], 'Position',[100 100 1000 600]);

    nVid = numel(metrics);
    xTicks = 1:nVid;

    videoNames = {metrics.videoName};

    shortNames = cellfun(@(n) strrep(strrep(n,'set00_',''),'set01_',''), ...
        videoNames, 'UniformOutput', false);

    pathLengths = [metrics.pathLength];
    meanConf    = [metrics.meanConfidence];
    nEdgesM     = [metrics.nEdges];
    usedCount   = [metrics.usedCount];
    invalidCnt  = [metrics.invalidCount];

    subplot(2,2,1);
    bar(xTicks, pathLengths, 'FaceColor',[0.3 0.7 1.0], 'EdgeColor','none');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot;
    ylabel('Metre','Color',[0.85 0.85 0.85]);
    title('Yol Uzunlugu');

    subplot(2,2,2);
    bar(xTicks, meanConf, 'FaceColor',[0.3 1.0 0.5], 'EdgeColor','none');
    yline(mean(meanConf),'--w','LineWidth',1,'Label','Ort.');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot;
    ylabel('Guven','Color',[0.85 0.85 0.85]);
    title('Ort. Guven Skoru');

    subplot(2,2,3);
    bar(xTicks, nEdgesM, 'FaceColor',[1.0 0.5 0.2], 'EdgeColor','none');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot;
    ylabel('Kenar','Color',[0.85 0.85 0.85]);
    title('Graf Kenar Sayisi');

    subplot(2,2,4);
    bar(xTicks, [usedCount; invalidCnt]', 'stacked', 'EdgeColor','none');
    legend({'Gecerli','Gecersiz'}, ...
        'TextColor','white', 'Color',[0.2 0.2 0.2]);

    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot;
    ylabel('Frame','Color',[0.85 0.85 0.85]);
    title('Gecerli / Gecersiz');

    sgtitle([label ' — Metrik Ozeti'], 'Color','white', 'FontSize',14);

    out4 = fullfile(metricsDir, [label '_adv_metrics.png']);
    exportgraphics(fig4, out4, 'Resolution', 300);
    fprintf('Kaydedildi: %s\n', out4);
    close(fig4);
end

%% OZET
fprintf('\n========== %s OZET ==========\n', label);
fprintf('Trajectory uzunlugu : %d frame\n', size(trajectory,1));
fprintf('Keyframe sayisi     : %d\n', size(optimized,1));
fprintf('Graf kenar sayisi   : %d\n', size(edges,1));
fprintf('Loop graph          : %d\n', loopCount);
fprintf('Loop potansiyel     : %d\n', potCount);

if hasMetrics && isfield(metrics(1),'meanConfidence')
    fprintf('Ort. guven skoru    : %.4f\n', mean([metrics.meanConfidence]));
end

fprintf('================================\n\n');

end

%% YARDIMCI FONKSIYONLAR
function styleAx(ax)

set(ax, ...
    'Color',[0.08 0.08 0.10], ...
    'XColor',[0.7 0.7 0.7], ...
    'YColor',[0.7 0.7 0.7], ...
    'GridColor',[0.3 0.3 0.3], ...
    'GridAlpha',0.3);

grid(ax,'on');

end

function styleSubplot

set(gca, ...
    'Color',[0.08 0.08 0.10], ...
    'XColor',[0.7 0.7 0.7], ...
    'YColor',[0.7 0.7 0.7], ...
    'GridColor',[0.3 0.3 0.3], ...
    'GridAlpha',0.3);

grid on;

t = get(gca,'Title');
set(t,'Color','white');

end