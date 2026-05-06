% COMPARE_CNN_SSD
% CNN ve SSD sonuclarini karsilastirir.
% Ciktilari results_final/figures/compare ve results_final/mat/compare içine kaydeder.
%
% Kullanim:
%   compare_cnn_ssd
%   compare_cnn_ssd('set00')

function compare_cnn_ssd(setName)

if nargin < 1
    setName = 'set00';
end

clc;
addpath(genpath('src'));

resultsSSD = fullfile(pwd, 'results_ssd');
resultsCNN = fullfile(pwd, 'results_cnn');

finalDir = fullfile(pwd, 'results_final');
figDir   = fullfile(finalDir, 'figures', 'compare');
matDir   = fullfile(finalDir, 'mat', 'compare');

dirs = {finalDir, figDir, matDir};

for i = 1:numel(dirs)
    if ~exist(dirs{i}, 'dir')
        mkdir(dirs{i});
    end
end

fprintf('CNN vs SSD Karsilastirma — %s\n', setName);
fprintf('==========================================\n');

dataDir = fullfile(pwd, 'data', setName);

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
        fprintf('SKIP %s — CNN graph yok\n', vName);
        continue;
    end

    if ~exist(ssdPath,'file')
        fprintf('SKIP %s — SSD graph yok\n', vName);
        continue;
    end

    cnnG = load(cnnPath, 'optimizedNodes');
    ssdG = load(ssdPath, 'optimizedNodes');

    trajCNN = cnnG.optimizedNodes(:,1:2);
    trajSSD = ssdG.optimizedNodes(:,1:2);

    if size(trajCNN,1) < 2 || size(trajSSD,1) < 2
        fprintf('SKIP %s — node yetersiz\n', vName);
        continue;
    end

    pathCNN   = computePathLength(trajCNN);
    pathSSD   = computePathLength(trajSSD);

    driftCNN  = norm(trajCNN(end,:) - trajCNN(1,:));
    driftSSD  = norm(trajSSD(end,:) - trajSSD(1,:));

    smoothCNN = computeSmoothness(trajCNN);
    smoothSSD = computeSmoothness(trajSSD);

    ri = ri + 1;

    results(ri).video     = vName;
    results(ri).pathCNN   = pathCNN;
    results(ri).pathSSD   = pathSSD;
    results(ri).driftCNN  = driftCNN;
    results(ri).driftSSD  = driftSSD;
    results(ri).smoothCNN = smoothCNN;
    results(ri).smoothSSD = smoothSSD;

    fprintf('\n--- %s ---\n', vName);
    fprintf('Path   CNN: %.4f | SSD: %.4f\n', pathCNN, pathSSD);
    fprintf('Drift  CNN: %.4f | SSD: %.4f\n', driftCNN, driftSSD);
    fprintf('Smooth CNN: %.4f | SSD: %.4f\n', smoothCNN, smoothSSD);
end

if isempty(results)
    fprintf('Hic sonuc yok. Once main_cnn ve main_ssd calistirilmali.\n');
    return;
end

%% KONSOL TABLOSU
fprintf('\n==========================================\n');
fprintf('%-6s | %8s %8s | %8s %8s | %8s %8s\n', ...
    'Video','PathCNN','PathSSD','DriftCNN','DriftSSD','SmthCNN','SmthSSD');

fprintf('%s\n', repmat('-',1,70));

for vi = 1:length(results)
    fprintf('%-6s | %8.4f %8.4f | %8.4f %8.4f | %8.4f %8.4f\n', ...
        results(vi).video, ...
        results(vi).pathCNN, results(vi).pathSSD, ...
        results(vi).driftCNN, results(vi).driftSSD, ...
        results(vi).smoothCNN, results(vi).smoothSSD);
end

fprintf('%s\n', repmat('-',1,70));

fprintf('%-6s | %8.4f %8.4f | %8.4f %8.4f | %8.4f %8.4f\n', ...
    'MEAN', ...
    mean([results.pathCNN]), mean([results.pathSSD]), ...
    mean([results.driftCNN]), mean([results.driftSSD]), ...
    mean([results.smoothCNN]), mean([results.smoothSSD]));

%% GORSEL
x = 1:length(results);
videos = {results.video};

bg   = [0.12 0.12 0.15];
axbg = [0.08 0.08 0.10];
tc   = [0.85 0.85 0.85];
gc   = [0.3 0.3 0.3];

cCNN = [0.3 0.75 1.0];
cSSD = [1.0 0.55 0.2];

fig = figure('Visible','off', ...
    'Name','CNN vs SSD', ...
    'Color',bg, ...
    'Position',[80 80 1400 850]);

metricsList = {
    'Path Length (m)',  'pathCNN',   'pathSSD',   'Yol uzunlugu';
    'Drift (m)',        'driftCNN',  'driftSSD',  'Kucuk deger daha iyi';
    'Smoothness (rad)', 'smoothCNN', 'smoothSSD', 'Kucuk deger daha yumusak';
};

for mi = 1:3

    mLabel = metricsList{mi,1};
    fCNN   = metricsList{mi,2};
    fSSD   = metricsList{mi,3};
    mNote  = metricsList{mi,4};

    valCNN = [results.(fCNN)];
    valSSD = [results.(fSSD)];

    %% Video bazli bar
    ax = subplot(3,2,(mi-1)*2+1);
    hold(ax,'on');

    bw = 0.35;

    bar(ax, x - bw/2, valCNN, bw, ...
        'FaceColor', cCNN, ...
        'EdgeColor','none');

    bar(ax, x + bw/2, valSSD, bw, ...
        'FaceColor', cSSD, ...
        'EdgeColor','none');

    legend(ax, {'CNN','SSD'}, ...
        'TextColor','white', ...
        'Color',[0.2 0.2 0.2], ...
        'Location','best', ...
        'FontSize',8);

    set(ax, ...
        'XTick',x, ...
        'XTickLabel',videos, ...
        'XTickLabelRotation',30, ...
        'Color',axbg, ...
        'XColor',tc, ...
        'YColor',tc, ...
        'GridColor',gc, ...
        'GridAlpha',0.3, ...
        'FontSize',9);

    grid(ax,'on');

    ylabel(ax, mLabel, 'Color',tc, 'FontSize',9);
    title(ax, [mLabel ' — Video Bazli'], 'Color','white', 'FontSize',10);

    %% Ortalama bar
    ax2 = subplot(3,2,(mi-1)*2+2);
    hold(ax2,'on');

    meanVals = [mean(valCNN), mean(valSSD)];

    bh = barh(ax2, [1 2], meanVals, 0.5, 'EdgeColor','none');
    bh.FaceColor = 'flat';
    bh.CData = [cCNN; cSSD];

    set(ax2, ...
        'YTick',[1 2], ...
        'YTickLabel',{'CNN','SSD'}, ...
        'Color',axbg, ...
        'XColor',tc, ...
        'YColor',tc, ...
        'GridColor',gc, ...
        'GridAlpha',0.3, ...
        'FontSize',10);

    grid(ax2,'on');

    xlabel(ax2, mLabel, 'Color',tc, 'FontSize',9);
    title(ax2, [mLabel ' — Ortalama'], 'Color','white', 'FontSize',10);

    for bi = 1:2
        text(ax2, meanVals(bi)*0.5, bi, sprintf('%.4f', meanVals(bi)), ...
            'Color','white', ...
            'FontSize',9, ...
            'HorizontalAlignment','center', ...
            'FontWeight','bold');
    end

    text(ax2, max(meanVals)*1.02, 1.5, mNote, ...
        'Color',[0.6 0.6 0.6], ...
        'FontSize',8, ...
        'HorizontalAlignment','left');
end

sgtitle(sprintf('CNN vs SSD Karsilastirma — %s', setName), ...
    'Color','white', ...
    'FontSize',15, ...
    'FontWeight','bold');

outPng = fullfile(figDir, sprintf('compare_cnn_ssd_%s.png', setName));
exportgraphics(fig, outPng, 'Resolution', 300);
close(fig);

fprintf('\nGorsel kaydedildi: %s\n', outPng);

%% MAT SAVE
outMat = fullfile(matDir, sprintf('compare_cnn_ssd_%s.mat', setName));
save(outMat, 'results');

fprintf('MAT kaydedildi: %s\n', outMat);
fprintf('KARSILASTIRMA BITTI\n');

end

%% YARDIMCI FONKSIYONLAR

function p = computePathLength(traj)

if size(traj,1) < 2
    p = 0;
    return;
end

p = sum(sqrt(sum(diff(traj,1,1).^2,2)));

end

function s = computeSmoothness(traj)

if size(traj,1) < 3
    s = 0;
    return;
end

angles = zeros(size(traj,1)-2, 1);

for i = 2:size(traj,1)-1

    v1 = traj(i,:)   - traj(i-1,:);
    v2 = traj(i+1,:) - traj(i,:);

    n1 = norm(v1);
    n2 = norm(v2);

    if n1 < 1e-9 || n2 < 1e-9
        angles(i-1) = 0;
    else
        cosA = max(-1, min(1, dot(v1,v2)/(n1*n2)));
        angles(i-1) = acos(cosA);
    end
end

s = mean(angles);

end