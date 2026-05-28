function evaluate_dnn_proxy_metrics()
% EVALUATE_DNN_PROXY_METRICS
% Ground truth pose olmadiginda DNN-SLAM sonuclarini proxy metriklerle
% degerlendirir. Bu dosya ATE/RTE iddiasi uretmez.
%
% Metrikler:
%   - Path Length
%   - Endpoint Displacement
%   - Endpoint Ratio = Endpoint Displacement / Path Length
%   - Loop Closure Count
%   - Mean Confidence

clc; close all;
addpath(genpath('src'));

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
    confPath  = fullfile(matDir, [prefix '_conf.mat']);

    if ~isfile(graphPath)
        fprintf('SKIP graph yok: %s\n', prefix);
        continue;
    end

    T = load(trajPath, 'trajectory');
    G = load(graphPath, 'graph', 'optimizedNodes');

    trajectory = T.trajectory;
    optimizedNodes = G.optimizedNodes;
    graph = G.graph;

    if size(optimizedNodes,2) < 2 || size(optimizedNodes,1) < 2
        fprintf('SKIP yetersiz node: %s\n', prefix);
        continue;
    end

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
    metrics(mi).rawPathLength = pathLength2D(trajectory(:,1:2));
    metrics(mi).optimizedPathLength = pathLength2D(optimizedNodes(:,1:2));
    metrics(mi).finalDrift = norm(optimizedNodes(end,1:2) - optimizedNodes(1,1:2));
    % Ground truth veya bilinen kapali rota yoksa bu deger hata degil,
    % trajectory'nin ne kadar kapali/ileri gittigini gosteren proxy orandir.
    metrics(mi).normalizedDrift = metrics(mi).finalDrift / max(metrics(mi).optimizedPathLength, eps);
    metrics(mi).loopClosureCount = loopCount;
    metrics(mi).meanConfidence = meanConf;
    metrics(mi).nodeCount = size(optimizedNodes,1);
    metrics(mi).edgeCount = size(edges,1);
end

if isempty(metrics)
    error('Degerlendirilecek DNN metrik bulunamadi.');
end

outMat = fullfile(matDir, 'dnn_proxy_metrics.mat');
save(outMat, 'metrics');

fprintf('\n%-20s %10s %10s %10s %8s %8s\n', ...
    'Sequence', 'Path', 'Endpoint', 'EndRatio', 'Loops', 'Conf');
fprintf('%s\n', repmat('-', 1, 74));
for i = 1:numel(metrics)
    fprintf('%-20s %10.4f %10.4f %10.4f %8d %8.4f\n', ...
        metrics(i).name, metrics(i).optimizedPathLength, ...
        metrics(i).finalDrift, metrics(i).normalizedDrift, ...
        metrics(i).loopClosureCount, metrics(i).meanConfidence);
end

names = {metrics.name};
x = 1:numel(metrics);

fig = figure('Visible','off','Color','w','Position',[100 100 1300 760]);

subplot(2,2,1);
bar(x, [metrics.optimizedPathLength]);
title('DNN-SLAM Path Length'); ylabel('m'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

subplot(2,2,2);
bar(x, [metrics.finalDrift]);
title('Endpoint Displacement'); ylabel('m'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

subplot(2,2,3);
bar(x, [metrics.normalizedDrift]);
title('Endpoint / Path Ratio'); ylabel('endpoint / path'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

subplot(2,2,4);
yyaxis left;
bar(x, [metrics.loopClosureCount]);
ylabel('Loop closure');
yyaxis right;
plot(x, [metrics.meanConfidence], 'o-', 'LineWidth', 1.5);
ylabel('Mean confidence');
title('Loop Closure ve Guven');
grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

sgtitle('DNN Graph Thermal SLAM Proxy Metrics');
outPng = fullfile(figDir, 'dnn_proxy_metrics.png');
exportgraphics(fig, outPng, 'Resolution', 300);
close(fig);

fprintf('\nMAT kaydedildi: %s\n', outMat);
fprintf('PNG kaydedildi: %s\n', outPng);
end

function p = pathLength2D(traj)
if size(traj,1) < 2
    p = 0;
else
    p = sum(sqrt(sum(diff(traj,1,1).^2, 2)));
end
end
