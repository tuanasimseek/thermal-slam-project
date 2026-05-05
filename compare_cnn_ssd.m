% COMPARE_CNN_SSD
% CNN ve SSD sonuclarini karsilastirir, gelismis gorsel uretir.
%
% Kullanim:
%   compare_cnn_ssd           % set00 icin
%   compare_cnn_ssd('set01')  % baska set icin

function compare_cnn_ssd(setName)

if nargin < 1, setName = 'set00'; end

clc;
addpath(genpath('src'));

resultsSSD = fullfile(pwd, 'results_ssd');
resultsCNN = fullfile(pwd, 'results_cnn');

fprintf('CNN vs SSD Karsilastirma — %s\n', setName);
fprintf('==========================================\n');

% Veri klasorunden video listesi
dataDir   = fullfile(pwd, 'data', setName);
videoDirs = dir(dataDir);
videoDirs = videoDirs([videoDirs.isdir]);
videoDirs = videoDirs(~ismember({videoDirs.name},{'.','..'}));

results = struct([]);
ri = 0;

for vi = 1:length(videoDirs)
    vName = videoDirs(vi).name;

    cnnPath = fullfile(resultsCNN, sprintf('%s_%s_graph.mat', setName, vName));
    ssdPath = fullfile(resultsSSD, sprintf('%s_%s_graph.mat', setName, vName));

    if ~exist(cnnPath,'file')
        fprintf('SKIP %s — CNN graph yok\n', vName); continue;
    end
    if ~exist(ssdPath,'file')
        fprintf('SKIP %s — SSD graph yok\n', vName); continue;
    end

    cnnG = load(cnnPath, 'optimizedNodes');
    ssdG = load(ssdPath, 'optimizedNodes');

    trajCNN = cnnG.optimizedNodes(:,1:2);
    trajSSD = ssdG.optimizedNodes(:,1:2);

    if size(trajCNN,1) < 2 || size(trajSSD,1) < 2
        fprintf('SKIP %s — node yetersiz\n', vName); continue;
    end

    pathCNN   = sum(sqrt(sum(diff(trajCNN,1,1).^2,2)));
    pathSSD   = sum(sqrt(sum(diff(trajSSD,1,1).^2,2)));
    driftCNN  = norm(trajCNN(end,:)-trajCNN(1,:));
    driftSSD  = norm(trajSSD(end,:)-trajSSD(1,:));
    smoothCNN = computeSmoothness(trajCNN);
    smoothSSD = computeSmoothness(trajSSD);

    ri = ri+1;
    results(ri).video     = vName;
    results(ri).pathCNN   = pathCNN;
    results(ri).pathSSD   = pathSSD;
    results(ri).driftCNN  = driftCNN;
    results(ri).driftSSD  = driftSSD;
    results(ri).smoothCNN = smoothCNN;
    results(ri).smoothSSD = smoothSSD;

    fprintf('\n--- %s ---\n', vName);
    fprintf('  Path   CNN: %8.4f  |  SSD: %8.4f\n', pathCNN,   pathSSD);
    fprintf('  Drift  CNN: %8.4f  |  SSD: %8.4f\n', driftCNN,  driftSSD);
    fprintf('  Smooth CNN: %8.4f  |  SSD: %8.4f\n', smoothCNN, smoothSSD);
end

if isempty(results)
    fprintf('Hic sonuc yok — hem CNN hem SSD graph dosyalari olmali.\n');
    return;
end

% ---- ozet tablo ----
fprintf('\n==========================================\n');
fprintf('%-6s | %8s %8s | %8s %8s | %8s %8s\n',...
    'Video','PathCNN','PathSSD','DriftCNN','DriftSSD','SmthCNN','SmthSSD');
fprintf('%s\n', repmat('-',1,70));
for vi = 1:length(results)
    fprintf('%-6s | %8.4f %8.4f | %8.4f %8.4f | %8.4f %8.4f\n',...
        results(vi).video,...
        results(vi).pathCNN,  results(vi).pathSSD,...
        results(vi).driftCNN, results(vi).driftSSD,...
        results(vi).smoothCNN,results(vi).smoothSSD);
end
fprintf('%s\n', repmat('-',1,70));
fprintf('%-6s | %8.4f %8.4f | %8.4f %8.4f | %8.4f %8.4f\n','MEAN',...
    mean([results.pathCNN]),  mean([results.pathSSD]),...
    mean([results.driftCNN]), mean([results.driftSSD]),...
    mean([results.smoothCNN]),mean([results.smoothSSD]));

% ---- gorsel ----
finalDir = fullfile(pwd, 'results_final');
figDir   = fullfile(finalDir, 'figures', 'compare');

if ~exist(figDir,'dir'), mkdir(figDir); end

x      = 1:length(results);
videos = {results.video};
bg     = [0.12 0.12 0.15];
axbg   = [0.08 0.08 0.10];
tc     = [0.85 0.85 0.85];
gc     = [0.3  0.3  0.3 ];
cCNN   = [0.3  0.75 1.0 ];   % mavi — CNN
cSSD   = [1.0  0.55 0.2 ];   % turuncu — SSD

fig = figure('Name','CNN vs SSD','Color',bg,'Position',[80 80 1300 800]);

metrics_list = {
    'Path Length (m)',   'pathCNN',   'pathSSD',   'Uzun = daha fazla hareket';
    'Drift (m)',         'driftCNN',  'driftSSD',  'Kucuk = daha az surukleme';
    'Smoothness (rad)',  'smoothCNN', 'smoothSSD', 'Kucuk = daha yumusak';
};

for mi = 1:3
    mLabel  = metrics_list{mi,1};
    fCNN    = metrics_list{mi,2};
    fSSD    = metrics_list{mi,3};
    mNote   = metrics_list{mi,4};

    valCNN = [results.(fCNN)];
    valSSD = [results.(fSSD)];

    % --- Alt grafik 1: gruplu bar ---
    ax = subplot(3,2,(mi-1)*2+1);
    hold(ax,'on');
    bw = 0.35;
    bar(ax, x - bw/2, valCNN, bw, 'FaceColor', cCNN, 'EdgeColor','none');
    bar(ax, x + bw/2, valSSD, bw, 'FaceColor', cSSD, 'EdgeColor','none');
    legend(ax, {'CNN','SSD'}, 'TextColor','white','Color',[0.2 0.2 0.2], ...
        'Location','best','FontSize',8);
    set(ax,'XTick',x,'XTickLabel',videos,'XTickLabelRotation',30,...
        'Color',axbg,'XColor',tc,'YColor',tc,'GridColor',gc,'GridAlpha',0.3,...
        'FontSize',9);
    grid(ax,'on');
    ylabel(ax, mLabel,'Color',tc,'FontSize',9);
    title(ax, [mLabel ' — Video Bazli'],'Color','white','FontSize',10);

    % Fark isaretleri
    for vi2 = 1:length(results)
        diff_val = valCNN(vi2) - valSSD(vi2);
        if abs(diff_val) > 0.001
            clr = [0.4 1.0 0.5];   % CNN daha iyi
            if diff_val > 0, clr = [1.0 0.4 0.4]; end  % SSD daha iyi (drift/smooth icin)
            maxV = max(valCNN(vi2), valSSD(vi2));
            text(ax, x(vi2), maxV * 1.05, sprintf('%.2f', diff_val), ...
                'Color', clr, 'FontSize', 7, 'HorizontalAlignment','center');
        end
    end

    % --- Alt grafik 2: ortalama karsilastirma (yatay bar) ---
    ax2 = subplot(3,2,(mi-1)*2+2);
    hold(ax2,'on');
    meanVals = [mean(valCNN), mean(valSSD)];
    bh = barh(ax2, [1 2], meanVals, 0.5, 'EdgeColor','none');
    bh.FaceColor = 'flat';
    bh.CData = [cCNN; cSSD];
    set(ax2,'YTick',[1 2],'YTickLabel',{'CNN','SSD'},...
        'Color',axbg,'XColor',tc,'YColor',tc,'GridColor',gc,'GridAlpha',0.3,...
        'FontSize',10);
    grid(ax2,'on');
    xlabel(ax2, mLabel,'Color',tc,'FontSize',9);
    title(ax2,[mLabel ' — Ortalama'],'Color','white','FontSize',10);

    % Deger etiketi
    for bi = 1:2
        text(ax2, meanVals(bi)*0.5, bi, sprintf('%.4f', meanVals(bi)), ...
            'Color','white','FontSize',9,'HorizontalAlignment','center',...
            'FontWeight','bold');
    end

    % Not
    text(ax2, max(meanVals)*1.02, 1.5, mNote, ...
        'Color',[0.6 0.6 0.6],'FontSize',7,'HorizontalAlignment','left');
end

sgtitle(sprintf('CNN vs SSD Karsilastirma — %s', setName), ...
    'Color','white','FontSize',14,'FontWeight','bold');
set(fig,'Color',bg);

outFile = fullfile(figDir, sprintf('compare_cnn_ssd_%s.png', setName));
saveas(fig, outFile);
fprintf('\nGorsel kaydedildi: %s\n', outFile);

% Mat kaydet
save(fullfile(finalDir,'compare_results.mat'),'results');
fprintf('compare_results.mat kaydedildi: %s\n', fullfile(finalDir,'compare_results.mat'));
fprintf('KARSILASTIRMA BITTI\n');

end

% ---- yardimci ----
function s = computeSmoothness(traj)
    if size(traj,1) < 3, s = 0; return; end
    angles = zeros(size(traj,1)-2, 1);
    for i = 2:size(traj,1)-1
        v1 = traj(i,:)   - traj(i-1,:);
        v2 = traj(i+1,:) - traj(i,:);
        n1 = norm(v1); n2 = norm(v2);
        if n1 < 1e-9 || n2 < 1e-9
            angles(i-1) = 0;
        else
            cosA = max(-1, min(1, dot(v1,v2)/(n1*n2)));
            angles(i-1) = acos(cosA);
        end
    end
    s = mean(angles);
end