clc; clear; close all;
addpath(genpath('src'));

setName    = 'set00';
resultsSSD = fullfile(pwd, 'results_ssd');
resultsCNN = fullfile(pwd, 'results_cnn');

fprintf('CNN vs SSD Karşılaştırma — %s\n', setName);

videoDirs = dir(fullfile(pwd,'data',setName));
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

    pathCNN  = sum(sqrt(sum(diff(trajCNN,1,1).^2,2)));
    pathSSD  = sum(sqrt(sum(diff(trajSSD,1,1).^2,2)));
    driftCNN = norm(trajCNN(end,:)-trajCNN(1,:));
    driftSSD = norm(trajSSD(end,:)-trajSSD(1,:));
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
    fprintf('  Path   — CNN: %.4f | SSD: %.4f\n', pathCNN, pathSSD);
    fprintf('  Drift  — CNN: %.4f | SSD: %.4f\n', driftCNN, driftSSD);
    fprintf('  Smooth — CNN: %.4f | SSD: %.4f\n', smoothCNN, smoothSSD);
end

if isempty(results)
    fprintf('Hiç sonuç yok.\n'); return;
end

fprintf('\n========== ÖZET ==========\n');
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

%% Grafik
figDir = fullfile(resultsCNN,'figures');
if ~exist(figDir,'dir'), mkdir(figDir); end

x = 1:length(results);
videos = {results.video};

figure('Name','CNN vs SSD','Position',[100 100 1200 400]);

subplot(1,3,1);
bar(x,[[results.pathCNN]' [results.pathSSD]']);
set(gca,'XTick',x,'XTickLabel',videos);
legend('CNN','SSD'); title('Path Length'); ylabel('m'); grid on;

subplot(1,3,2);
bar(x,[[results.driftCNN]' [results.driftSSD]']);
set(gca,'XTick',x,'XTickLabel',videos);
legend('CNN','SSD'); title('Drift'); ylabel('m'); grid on;

subplot(1,3,3);
bar(x,[[results.smoothCNN]' [results.smoothSSD]']);
set(gca,'XTick',x,'XTickLabel',videos);
legend('CNN','SSD'); title('Smoothness (düşük=iyi)'); ylabel('rad'); grid on;

sgtitle(sprintf('CNN vs SSD — %s', setName));
saveas(gcf, fullfile(figDir,'compare_cnn_ssd.png'));
fprintf('\nGrafik kaydedildi.\n');

save(fullfile(resultsCNN,'compare_results.mat'),'results');
fprintf('KARŞILAŞTIRMA BİTTİ\n');

function s = computeSmoothness(traj)
    if size(traj,1) < 3, s=0; return; end
    angles = zeros(size(traj,1)-2,1);
    for i = 2:size(traj,1)-1
        v1 = traj(i,:)-traj(i-1,:);
        v2 = traj(i+1,:)-traj(i,:);
        n1=norm(v1); n2=norm(v2);
        if n1<1e-9||n2<1e-9, angles(i-1)=0;
        else
            cosA=max(-1,min(1,dot(v1,v2)/(n1*n2)));
            angles(i-1)=acos(cosA);
        end
    end
    s=mean(angles);
end