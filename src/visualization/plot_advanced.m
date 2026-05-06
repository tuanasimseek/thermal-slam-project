function plot_advanced(trajFile, graphFile, metricsFile, saveDir, methodName, confFile)
% PLOT_ADVANCED
% Heatmap, guven bandi, loop closure ve opsiyonel metrik grafikleri uretir.

if nargin < 2, error('En az 2 arguman gerekli'); end
if nargin < 3, metricsFile = []; end
if nargin < 4 || isempty(saveDir)
    saveDir = fullfile(pwd, 'results_final', 'figures', 'advanced');
end
if nargin < 5 || isempty(methodName), methodName = 'default'; end
if nargin < 6, confFile = ''; end

hasMetrics = ~isempty(metricsFile) && exist(metricsFile, 'file');

% DÜZELTME: conf.mat varsa oku, yoksa NaN
hasConf  = ~isempty(confFile) && exist(confFile, 'file');
meanConf = NaN;
if hasConf
    C = load(confFile);
    if isfield(C, 'meanConf')
        meanConf = C.meanConf;
    end
end

%% VERİ YÜKLE
fprintf('Yukleniyor: %s\n', trajFile);

T          = load(trajFile);
trajectory = T.trajectory;

G         = load(graphFile);
graph     = G.graph;
optimized = G.optimizedNodes;

if hasMetrics
    M       = load(metricsFile);
    metrics = M.metrics;
end

%% ÇIKTI KLASÖRLERİ
heatmapDir    = fullfile(saveDir, 'heatmap');
confidenceDir = fullfile(saveDir, 'confidence');
loopDir       = fullfile(saveDir, 'loop_closure');
metricsDir    = fullfile(saveDir, 'metrics');

for d = {heatmapDir, confidenceDir, loopDir, metricsDir}
    if ~exist(d{1}, 'dir'), mkdir(d{1}); end
end

[~, baseName, ~] = fileparts(trajFile);
label = strrep(baseName, '_traj', '');
if ~contains(label, methodName)
    label = sprintf('%s_%s', label, methodName);
end

%% ==========================================================
% PANEL 1: HEATMAP
% ==========================================================
fig1 = figure('Visible','off','Color',[0.12 0.12 0.15],'Position',[100 100 800 650]);
ax1  = axes('Parent', fig1);
hold(ax1, 'on');

histogram2(ax1, trajectory(:,1), trajectory(:,2), 60, ...
    'DisplayStyle','tile','ShowEmptyBins','on','EdgeColor','none');
colormap(ax1, hot);

cb = colorbar(ax1);
cb.Label.String = 'Gecis sayisi';
cb.Color        = [0.85 0.85 0.85];
cb.Label.Color  = [0.85 0.85 0.85];

scatter(ax1, optimized(:,1), optimized(:,2), 45, 'cyan', 'filled', ...
    'MarkerEdgeColor','white','LineWidth',0.5);
plot(ax1, trajectory(1,1),   trajectory(1,2),   'gs','MarkerSize',10,'MarkerFaceColor','green','LineWidth',1.5);
plot(ax1, trajectory(end,1), trajectory(end,2), 'r^','MarkerSize',10,'MarkerFaceColor','red',  'LineWidth',1.5);

legend(ax1, {'Keyframe','Baslangic','Bitis'}, ...
    'TextColor','white','Color',[0.2 0.2 0.2],'Location','best');
styleAx(ax1);
xlabel(ax1,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax1,'Y (m)','Color',[0.85 0.85 0.85]);
title(ax1, [label ' — Ziyaret Yogunlugu'],'Color','white','FontSize',13);
axis(ax1,'equal');

out1 = fullfile(heatmapDir, [label '_adv_heatmap.png']);
exportgraphics(fig1, out1, 'Resolution',300);
fprintf('Kaydedildi: %s\n', out1);
close(fig1);

%% ==========================================================
% PANEL 2: GÜVEN BANDI
% DÜZELTME: Başlığa gerçek conf değeri eklendi
% ==========================================================
fig2 = figure('Visible','off','Color',[0.12 0.12 0.15],'Position',[100 100 800 650]);
ax2  = axes('Parent', fig2);
hold(ax2, 'on');

winSize = 30;
N       = size(trajectory,1);
sigma   = zeros(N,1);

for i = 1:N
    i0 = max(1, i-winSize);
    w  = trajectory(i0:i, :);
    if size(w,1) > 1
        sigma(i) = sqrt(var(w(:,1)) + var(w(:,2)));
    end
end

fillX = [trajectory(:,1); flipud(trajectory(:,1))];
fillY = [trajectory(:,2)+sigma; flipud(trajectory(:,2)-sigma)];

fill(ax2, fillX, fillY, [0.3 0.6 1.0], 'FaceAlpha',0.18,'EdgeColor','none','DisplayName','Guven bandi');
plot(ax2, trajectory(:,1), trajectory(:,2), '-','Color',[0.4 0.8 1.0],'LineWidth',1.2,'DisplayName','Trajectory');
plot(ax2, optimized(:,1),  optimized(:,2),  'o','Color','white','MarkerFaceColor',[0.9 0.7 0.2],'MarkerSize',5,'LineWidth',0.8,'DisplayName','Keyframe');
plot(ax2, trajectory(1,1),   trajectory(1,2),   'gs','MarkerSize',10,'MarkerFaceColor','green','LineWidth',1.5,'DisplayName','Baslangic');
plot(ax2, trajectory(end,1), trajectory(end,2), 'r^','MarkerSize',10,'MarkerFaceColor','red',  'LineWidth',1.5,'DisplayName','Bitis');

legend(ax2,'TextColor','white','Color',[0.2 0.2 0.2],'Location','best');
styleAx(ax2);
xlabel(ax2,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax2,'Y (m)','Color',[0.85 0.85 0.85]);

% DÜZELTME: Başlığa güven skoru eklendi
if isnan(meanConf)
    confStr = '';
else
    confStr = sprintf('  |  Ort. Guven: %.4f', meanConf);
end
title(ax2, [label ' — Trajectory ve Guven Bandi' confStr], 'Color','white','FontSize',13);
axis(ax2,'equal');

out2 = fullfile(confidenceDir, [label '_adv_confidence.png']);
exportgraphics(fig2, out2, 'Resolution',300);
fprintf('Kaydedildi: %s\n', out2);
close(fig2);

%% ==========================================================
% PANEL 3: LOOP CLOSURE (legend uyarısı düzeltmesi korundu)
% ==========================================================
fig3 = figure('Visible','off','Color',[0.12 0.12 0.15],'Position',[100 100 800 650]);
ax3  = axes('Parent', fig3);
hold(ax3, 'on');

hTraj = plot(ax3, trajectory(:,1), trajectory(:,2), '-','Color',[0.5 0.5 0.6],'LineWidth',0.8);
hKF   = scatter(ax3, optimized(:,1), optimized(:,2), 30, 'white','filled','MarkerEdgeColor',[0.6 0.6 0.6],'LineWidth',0.5);

edges     = graph.edges;
loopCount = 0;
hLoop     = [];

for e = 1:size(edges,1)
    ni = edges(e,1);
    nj = edges(e,2);
    if abs(nj-ni) > 1 && ni <= size(optimized,1) && nj <= size(optimized,1)
        h = plot(ax3, [optimized(ni,1) optimized(nj,1)], ...
                      [optimized(ni,2) optimized(nj,2)], '--', ...
                 'Color',[1.0 0.4 0.2],'LineWidth',1.8,'HandleVisibility','off');
        if loopCount == 0
            hLoop = h;
            set(hLoop,'HandleVisibility','on');
        end
        loopCount = loopCount + 1;
    end
end

M_nodes   = size(optimized,1);
distThresh = 0.5;
loopPairs  = [];

for i = 1:M_nodes
    for j = i+5:M_nodes
        if norm(optimized(i,:)-optimized(j,:)) < distThresh
            loopPairs(end+1,:) = [i j]; %#ok<AGROW>
        end
    end
end

potCount = size(loopPairs,1);
hPot     = [];

for lp = 1:potCount
    i = loopPairs(lp,1);
    j = loopPairs(lp,2);
    h = plot(ax3, [optimized(i,1) optimized(j,1)], ...
                  [optimized(i,2) optimized(j,2)], ':', ...
             'Color',[0.3 1.0 0.6],'LineWidth',1.5,'HandleVisibility','off');
    if lp == 1
        hPot = h;
        set(hPot,'HandleVisibility','on');
    end
end

if loopCount == 0 && potCount == 0
    xl = xlim(ax3); yl = ylim(ax3);
    text(ax3, mean(xl), mean(yl), 'Loop closure yok', ...
        'Color',[0.8 0.8 0.4],'FontSize',11,'HorizontalAlignment','center');
end

hStart = plot(ax3, trajectory(1,1),   trajectory(1,2),   'gs','MarkerSize',10,'MarkerFaceColor','green','LineWidth',1.5);
hEnd   = plot(ax3, trajectory(end,1), trajectory(end,2), 'r^','MarkerSize',10,'MarkerFaceColor','red',  'LineWidth',1.5);

lHandles = [hTraj hKF hStart hEnd];
lLabels  = {'Trajectory','Keyframe','Baslangic','Bitis'};
if ~isempty(hLoop), lHandles(end+1) = hLoop; lLabels{end+1} = 'Loop closure'; end
if ~isempty(hPot),  lHandles(end+1) = hPot;  lLabels{end+1} = sprintf('Potansiyel (<%.1fm)', distThresh); end

legend(ax3, lHandles, lLabels,'TextColor','white','Color',[0.2 0.2 0.2],'Location','best');
styleAx(ax3);
xlabel(ax3,'X (m)','Color',[0.85 0.85 0.85]);
ylabel(ax3,'Y (m)','Color',[0.85 0.85 0.85]);
title(ax3, sprintf('%s — Loop Closure  graph:%d  potansiyel:%d', label, loopCount, potCount), ...
    'Color','white','FontSize',13);
axis(ax3,'equal');

out3 = fullfile(loopDir, [label '_adv_loop_closure.png']);
exportgraphics(fig3, out3, 'Resolution',300);
fprintf('Kaydedildi: %s\n', out3);
close(fig3);

%% ==========================================================
% PANEL 4: METRİK (varsa)
% ==========================================================
if hasMetrics
    fig4   = figure('Visible','off','Color',[0.12 0.12 0.15],'Position',[100 100 1000 600]);
    nVid   = numel(metrics);
    xTicks = 1:nVid;

    videoNames = {metrics.videoName};
    shortNames = cellfun(@(n) strrep(strrep(n,'set00_',''),'set01_',''), videoNames,'UniformOutput',false);

    subplot(2,2,1);
    bar(xTicks,[metrics.pathLength],'FaceColor',[0.3 0.7 1.0],'EdgeColor','none');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot; ylabel('Metre','Color',[0.85 0.85 0.85]); title('Yol Uzunlugu');

    subplot(2,2,2);
    bar(xTicks,[metrics.meanConfidence],'FaceColor',[0.3 1.0 0.5],'EdgeColor','none');
    yline(mean([metrics.meanConfidence]),'--w','LineWidth',1,'Label','Ort.');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot; ylabel('Guven','Color',[0.85 0.85 0.85]); title('Ort. Guven Skoru');

    subplot(2,2,3);
    bar(xTicks,[metrics.nEdges],'FaceColor',[1.0 0.5 0.2],'EdgeColor','none');
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot; ylabel('Kenar','Color',[0.85 0.85 0.85]); title('Graf Kenar Sayisi');

    subplot(2,2,4);
    bar(xTicks,[metrics.usedCount; metrics.invalidCount]','stacked','EdgeColor','none');
    legend({'Gecerli','Gecersiz'},'TextColor','white','Color',[0.2 0.2 0.2]);
    set(gca,'XTick',xTicks,'XTickLabel',shortNames,'XTickLabelRotation',45);
    styleSubplot; ylabel('Frame','Color',[0.85 0.85 0.85]); title('Gecerli / Gecersiz');

    sgtitle([label ' — Metrik Ozeti'],'Color','white','FontSize',14);

    out4 = fullfile(metricsDir,[label '_adv_metrics.png']);
    exportgraphics(fig4, out4,'Resolution',300);
    fprintf('Kaydedildi: %s\n', out4);
    close(fig4);
end

%% ÖZET
fprintf('\n========== %s OZET ==========\n', label);
fprintf('Trajectory uzunlugu : %d frame\n', size(trajectory,1));
fprintf('Keyframe sayisi     : %d\n',        size(optimized,1));
fprintf('Graf kenar sayisi   : %d\n',        size(edges,1));
fprintf('Loop graph          : %d\n',        loopCount);
fprintf('Loop potansiyel     : %d\n',        potCount);
if ~isnan(meanConf)
    fprintf('Ort. guven skoru    : %.4f\n',  meanConf);
else
    fprintf('Ort. guven skoru    : (conf.mat bulunamadi)\n');
end
fprintf('================================\n\n');

end

%% YARDIMCI FONKSİYONLAR
function styleAx(ax)
set(ax,'Color',[0.08 0.08 0.10],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7], ...
    'GridColor',[0.3 0.3 0.3],'GridAlpha',0.3);
grid(ax,'on');
end

function styleSubplot
set(gca,'Color',[0.08 0.08 0.10],'XColor',[0.7 0.7 0.7],'YColor',[0.7 0.7 0.7], ...
    'GridColor',[0.3 0.3 0.3],'GridAlpha',0.3);
grid on;
t = get(gca,'Title'); set(t,'Color','white');
end