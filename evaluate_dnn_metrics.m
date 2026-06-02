function evaluate_dnn_metrics()
% EVALUATE_DNN_METRICS
% DNN graph thermal SLAM sonuclarini degerlendirir.
%
% Ground truth pose dosyasi bulunursa hizalanmis ATE/RTE hesaplar.
% Ground truth yoksa evaluate_dnn_proxy_metrics ile ayni mantikta proxy
% metriklere duser. Boylece proje, gercek pose verisi eklendiginde klasik
% hata metriklerini de destekler.

clc; close all;
addpath(genpath('src'));

dataRoot = fullfile(pwd, 'data');
matDir = fullfile(pwd, 'results_final', 'mat', 'dnn');
figDir = fullfile(pwd, 'results_final', 'figures', 'metrics');

if ~isfolder(matDir)
    error('DNN sonuc klasoru yok: %s\nOnce main_dnn_slam calistir.', matDir);
end
if ~isfolder(figDir)
    mkdir(figDir);
end

trajFiles = dir(fullfile(matDir, '*_dnn_traj.mat'));
if isempty(trajFiles)
    error('DNN trajectory sonucu bulunamadi: %s', matDir);
end

metrics = struct([]);
mi = 0;

for i = 1:numel(trajFiles)
    trajPath = fullfile(trajFiles(i).folder, trajFiles(i).name);
    prefix = erase(trajFiles(i).name, '_traj.mat');
    graphPath = fullfile(matDir, [prefix '_graph.mat']);
    confPath = fullfile(matDir, [prefix '_conf.mat']);

    if ~isfile(graphPath)
        fprintf('SKIP graph yok: %s\n', prefix);
        continue;
    end

    T = load(trajPath, 'trajectory');
    G = load(graphPath, 'graph', 'optimizedNodes');
    trajectory = T.trajectory;
    optimizedNodes = G.optimizedNodes;
    graph = G.graph;

    if size(optimizedNodes,1) < 2
        fprintf('SKIP yetersiz node: %s\n', prefix);
        continue;
    end

    [setName, videoName] = parseDnnPrefix(prefix);
    [gtPoses, gtPath, gtInfo] = load_ground_truth_pose(setName, videoName, dataRoot);

    meanConf = NaN;
    if isfile(confPath)
        C = load(confPath);
        if isfield(C, 'meanConf')
            meanConf = C.meanConf;
        end
    end

    edges = graph.edges;
    loopCount = 0;
    if ~isempty(edges) && size(edges,2) >= 7
        loopCount = sum(edges(:,7) == 1);
    end

    mi = mi + 1;
    metrics(mi).name = prefix; %#ok<AGROW>
    metrics(mi).setName = setName;
    metrics(mi).videoName = videoName;
    metrics(mi).rawPathLength = pathLength2D(trajectory(:,1:2));
    metrics(mi).optimizedPathLength = pathLength2D(optimizedNodes(:,1:2));
    metrics(mi).loopClosureCount = loopCount;
    metrics(mi).meanConfidence = meanConf;
    metrics(mi).nodeCount = size(optimizedNodes,1);
    metrics(mi).edgeCount = size(edges,1);
    metrics(mi).groundTruthPath = gtPath;
    metrics(mi).groundTruthFormat = gtInfo.format;

    if ~isempty(gtPoses)
        gtAligned = resamplePoses(gtPoses, size(optimizedNodes,1));
        predAligned = similarityAlign2D(optimizedNodes(:,1:2), gtAligned(:,1:2));

        ate = sqrt(mean(sum((predAligned - gtAligned(:,1:2)).^2, 2)));
        rte = relativeTrajectoryError(predAligned, gtAligned(:,1:2));

        metrics(mi).metricType = 'ground_truth_aligned';
        metrics(mi).ATE = ate;
        metrics(mi).RTE = rte;
        metrics(mi).endpointDisplacement = norm(predAligned(end,:) - predAligned(1,:));
        metrics(mi).endpointRatio = metrics(mi).endpointDisplacement / ...
            max(pathLength2D(predAligned), eps);
    else
        endpoint = norm(optimizedNodes(end,1:2) - optimizedNodes(1,1:2));
        metrics(mi).metricType = 'proxy_no_ground_truth';
        metrics(mi).ATE = NaN;
        metrics(mi).RTE = NaN;
        metrics(mi).endpointDisplacement = endpoint;
        metrics(mi).endpointRatio = endpoint / max(metrics(mi).optimizedPathLength, eps);
    end
end

if isempty(metrics)
    error('Degerlendirilecek DNN metrik bulunamadi.');
end

outMat = fullfile(matDir, 'dnn_metrics.mat');
save(outMat, 'metrics');

fprintf('\n%-22s %-22s %9s %9s %9s %6s %7s\n', ...
    'Sequence', 'MetricType', 'ATE', 'RTE', 'EndRatio', 'Loops', 'Conf');
fprintf('%s\n', repmat('-', 1, 92));
for i = 1:numel(metrics)
    fprintf('%-22s %-22s %9.4f %9.4f %9.4f %6d %7.4f\n', ...
        metrics(i).name, metrics(i).metricType, metrics(i).ATE, metrics(i).RTE, ...
        metrics(i).endpointRatio, metrics(i).loopClosureCount, metrics(i).meanConfidence);
end

makeMetricFigure(metrics, figDir);

fprintf('\nMAT kaydedildi: %s\n', outMat);
fprintf('Ground truth yoksa ATE/RTE NaN kalir ve EndRatio proxy metrik olarak yorumlanir.\n');
end

function [setName, videoName] = parseDnnPrefix(prefix)
base = erase(prefix, '_dnn');
parts = strsplit(base, '_');
setName = parts{1};
if numel(parts) > 1
    videoName = strjoin(parts(2:end), '_');
else
    videoName = '';
end
end

function sampled = resamplePoses(poses, targetN)
if size(poses,1) == targetN
    sampled = poses;
    return;
end

xOld = linspace(1, targetN, size(poses,1));
xNew = 1:targetN;
sampled = zeros(targetN, size(poses,2));
for c = 1:size(poses,2)
    sampled(:,c) = interp1(xOld, poses(:,c), xNew, 'linear', 'extrap');
end
end

function aligned = similarityAlign2D(pred, gt)
pred0 = pred - mean(pred, 1);
gt0 = gt - mean(gt, 1);

C = pred0' * gt0 / size(pred0, 1);
[U, ~, V] = svd(C);
R = V * U';
if det(R) < 0
    V(:,end) = -V(:,end);
    R = V * U';
end

scale = trace((pred0 * R)' * gt0) / max(sum(pred0(:).^2), eps);
aligned = scale * pred0 * R + mean(gt, 1);
end

function rte = relativeTrajectoryError(pred, gt)
n = size(pred,1);
if n < 4
    rte = NaN;
    return;
end

windowSize = max(2, round(n / 10));
errs = [];
for i = 1:(n - windowSize)
    dp = pred(i+windowSize,:) - pred(i,:);
    dg = gt(i+windowSize,:) - gt(i,:);
    errs(end+1) = norm(dp - dg); %#ok<AGROW>
end
rte = mean(errs);
end

function p = pathLength2D(traj)
if size(traj,1) < 2
    p = 0;
else
    p = sum(sqrt(sum(diff(traj,1,1).^2, 2)));
end
end

function makeMetricFigure(metrics, figDir)
names = {metrics.name};
x = 1:numel(metrics);
ateVals = [metrics.ATE];
rteVals = [metrics.RTE];
endpointRatios = [metrics.endpointRatio];
loopCounts = [metrics.loopClosureCount];
confVals = [metrics.meanConfidence];

fig = figure('Visible','off','Color','w','Position',[100 100 1300 760]);

subplot(2,2,1);
bar(x, ateVals);
title('Ground Truth ATE (varsa)'); ylabel('m'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);
if all(isnan(ateVals))
    text(mean(x), 0.5, 'Ground truth yok - ATE hesaplanmadi', ...
        'HorizontalAlignment','center', 'FontWeight','bold');
end

subplot(2,2,2);
bar(x, rteVals);
title('Ground Truth RTE (varsa)'); ylabel('m'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);
if all(isnan(rteVals))
    text(mean(x), 0.5, 'Ground truth yok - RTE hesaplanmadi', ...
        'HorizontalAlignment','center', 'FontWeight','bold');
end

subplot(2,2,3);
bar(x, endpointRatios);
title('Endpoint / Path Ratio'); ylabel('proxy oran'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

subplot(2,2,4);
yyaxis left;
bar(x, loopCounts);
ylabel('Loop closure');
yyaxis right;
plot(x, confVals, 'o-', 'LineWidth', 1.5);
ylabel('Mean confidence');
title('Loop Closure ve Guven');
grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

sgtitle('DNN Graph Thermal SLAM Metrics');
outPng = fullfile(figDir, 'dnn_metrics.png');
exportgraphics(fig, outPng, 'Resolution', 300);
close(fig);

fprintf('PNG kaydedildi: %s\n', outPng);
end
