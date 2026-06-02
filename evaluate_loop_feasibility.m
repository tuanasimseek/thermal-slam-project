function evaluate_loop_feasibility()
% EVALUATE_LOOP_FEASIBILITY
% DNN pose graph sonuclarinda loop closure icin veri uygunlugunu inceler.
%
% Graph'a eklenen gercek loop edge sayisi ile optimize edilmis trajectory
% uzerinden hesaplanan yakin-donus potansiyel loop sayisini birlikte
% raporlar. Bu analiz, loop closure'in veri seti ve rota geometrisi
% nedeniyle neden sinirli kalabildigini savunmada gostermek icindir.

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

graphFiles = dir(fullfile(matDir, '*_dnn_graph.mat'));
if isempty(graphFiles)
    error('DNN graph sonucu bulunamadi: %s', matDir);
end

loopStats = struct([]);

for i = 1:numel(graphFiles)
    graphPath = fullfile(graphFiles(i).folder, graphFiles(i).name);
    prefix = erase(graphFiles(i).name, '_graph.mat');
    G = load(graphPath, 'graph', 'optimizedNodes', 'loopEdges');

    graph = G.graph;
    optimizedNodes = G.optimizedNodes;

    graphLoopCount = 0;
    if ~isempty(graph.edges) && size(graph.edges,2) >= 7
        graphLoopCount = sum(graph.edges(:,7) == 1);
    elseif isfield(G, 'loopEdges')
        graphLoopCount = size(G.loopEdges, 1);
    end

    [potentialLoopCount, distanceThreshold] = countPotentialLoops(optimizedNodes);

    loopStats(i).name = prefix; %#ok<AGROW>
    loopStats(i).nodeCount = size(optimizedNodes, 1);
    loopStats(i).graphLoopCount = graphLoopCount;
    loopStats(i).potentialLoopCount = potentialLoopCount;
    loopStats(i).distanceThreshold = distanceThreshold;
    loopStats(i).loopClosureLimited = graphLoopCount == 0 && potentialLoopCount == 0;
end

outMat = fullfile(matDir, 'loop_feasibility.mat');
save(outMat, 'loopStats');

fprintf('\n%-22s %8s %8s %10s %10s\n', ...
    'Sequence', 'Nodes', 'Graph', 'Potential', 'Thresh');
fprintf('%s\n', repmat('-', 1, 66));
for i = 1:numel(loopStats)
    fprintf('%-22s %8d %8d %10d %10.4f\n', ...
        loopStats(i).name, loopStats(i).nodeCount, ...
        loopStats(i).graphLoopCount, loopStats(i).potentialLoopCount, ...
        loopStats(i).distanceThreshold);
end

names = {loopStats.name};
x = 1:numel(loopStats);

fig = figure('Visible','off','Color','w','Position',[100 100 1300 680]);

subplot(2,1,1);
bar(x, [[loopStats.graphLoopCount]' [loopStats.potentialLoopCount]']);
title('Loop Closure: Graph Edge ve Potansiyel Yakin Donus');
ylabel('Adet'); grid on;
legend({'Graph loop edge', 'Potansiyel loop'}, 'Location','best');
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

subplot(2,1,2);
bar(x, [loopStats.distanceThreshold]);
title('Adaptif Potansiyel Loop Mesafe Esigi');
ylabel('m'); grid on;
set(gca, 'XTick', x, 'XTickLabel', names, 'XTickLabelRotation', 45);

sgtitle('DNN Graph Thermal SLAM Loop Feasibility');
outPng = fullfile(figDir, 'loop_feasibility.png');
exportgraphics(fig, outPng, 'Resolution', 300);
close(fig);

fprintf('\nMAT kaydedildi: %s\n', outMat);
fprintf('PNG kaydedildi: %s\n', outPng);
end

function [count, threshold] = countPotentialLoops(nodes)
if size(nodes, 1) < 8
    count = 0;
    threshold = 0;
    return;
end

xy = nodes(:,1:2);
spanX = max(xy(:,1)) - min(xy(:,1));
spanY = max(xy(:,2)) - min(xy(:,2));
diagSpan = sqrt(spanX^2 + spanY^2);
threshold = max(0.002, min(0.05, 0.15 * diagSpan));

count = 0;
minGap = 5;
for i = 1:size(xy,1)
    for j = i+minGap:size(xy,1)
        if norm(xy(i,:) - xy(j,:)) < threshold
            count = count + 1;
        end
    end
end
end
