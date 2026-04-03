clc; clear; close all;
addpath(genpath('src'));

%% AYARLAR
lwirDir1 = fullfile('data', 'set04', 'V001', 'lwir');

useEveryN   = 5;
scaleMetric = 0.05;
alphaLP     = 0.7;
minInterval = 20;

manualLoopEdges = true;
manualEdgeCount = 8;

%% ILERI + GERI YAPAY LOOP
files1 = dir(fullfile(lwirDir1, '*.jpg'));
[~, idx] = sort({files1.name});
files1 = files1(idx);

maxFrames = min(400, length(files1));
files1 = files1(1:maxFrames);

files2 = files1(end:-1:1);   % ters sıra
files  = [files1; files2];   % ileri + geri

for i = 1:length(files)
    files(i).folder = lwirDir1;
end

N = numel(files);
fprintf('Toplam frame (ileri+geri yapay loop): %d\n', N);
fprintf('Tek yön frame sayısı: %d\n', length(files1));

%% CNN
fprintf('ResNet-18 yükleniyor...\n');
net = load_cnn_model();
fprintf('Hazır.\n');

%% BASLANGIC
graph = PoseGraph();
pos   = [0, 0];
traj  = pos;

dx_filt = 0;
dy_filt = 0;

featDB       = zeros(0, 50176);
frameNodeMap = [];

I_prev    = loadAndPrep(fullfile(files(1).folder, files(1).name));
feat_prev = feature_cnn(I_prev, net);

featDB(end+1,:)     = feat_prev;
graph               = graph.addNode(pos);
frameNodeMap(end+1) = graph.nodeCount;

loopCount = 0;

%% ANA DONGU
for i = 2:useEveryN:N

    if i >= length(files1) && i < length(files1) + useEveryN
        fprintf('\n--- GERI DONUS BASLADI | i=%d | file=%s ---\n', i, files(i).name);
    end

    I_curr    = loadAndPrep(fullfile(files(i).folder, files(i).name));
    feat_curr = feature_cnn(I_curr, net);

    [dx, dy, conf] = pose_estimator(I_prev, I_curr, feat_prev, feat_curr);

    dx_filt = alphaLP * dx_filt + (1 - alphaLP) * dx;
    dy_filt = alphaLP * dy_filt + (1 - alphaLP) * dy;

    delta = [dx_filt, dy_filt] * scaleMetric;
    pos   = pos + delta;
    traj  = [traj; pos];

    prevNode = graph.nodeCount;
    graph    = graph.addNode(pos);
    currNode = graph.nodeCount;

    graph = graph.addEdge(prevNode, currNode, delta, conf);

    featDB(end+1,:)     = feat_curr;
    frameNodeMap(end+1) = currNode;

    [loopIdx, simScore] = loop_detector(featDB, feat_curr, minInterval);

    if loopIdx > 0
        loopCount = loopCount + 1;

        loopNode  = frameNodeMap(loopIdx);
        loopDelta = traj(loopIdx,:) - pos;

        graph = graph.addEdge(currNode, loopNode, loopDelta, 1.0);

        fprintf('LOOP EKLENDI! i=%d -> loopIdx=%d | sim=%.3f\n', ...
            i, loopIdx, simScore);
    end

    if mod(i, 50) == 0
        fprintf('Frame %d/%d | pos=(%.4f, %.4f) | loops=%d\n', ...
            i, N, pos(1), pos(2), loopCount);
    end

    I_prev    = I_curr;
    feat_prev = feat_curr;
end

fprintf('\nToplam bulunan loop: %d\n', loopCount);
fprintf('Toplam node: %d\n', graph.nodeCount);
fprintf('Son poz (ham): (%.4f, %.4f)\n', pos(1), pos(2));

%% MANUEL LOOP EDGE
if manualLoopEdges
    fprintf('\n--- MANUEL YAPAY LOOP EDGE EKLENIYOR ---\n');

    nodeCount = graph.nodeCount;
    edgeCount = min(manualEdgeCount, floor(nodeCount / 3));

    for k = 0:(edgeCount-1)
        fromNode = nodeCount - k;
        toNode   = 1 + k;

        fromPos = graph.nodes(fromNode, :);
        toPos   = graph.nodes(toNode, :);

        loopDelta = toPos - fromPos;

        graph = graph.addEdge(fromNode, toNode, loopDelta, 1.0);

        fprintf('MANUEL LOOP: Node %d -> Node %d | delta=(%.4f, %.4f)\n', ...
            fromNode, toNode, loopDelta(1), loopDelta(2));
    end
end

%% OPTIMIZASYON
fprintf('\nGraph optimize ediliyor...\n');
optimizer      = GraphOptimizer(graph);
optimizedNodes = optimizer.optimize(250, 0.2);
fprintf('Optimizasyon tamamlandı.\n');

%% GRAFIK
figure('Name', 'Loop Closure — Ileri Geri Yapay Loop Testi');
plot(traj(:,1), traj(:,2), 'b-', 'LineWidth', 2); hold on;
plot(optimizedNodes(:,1), optimizedNodes(:,2), 'r--', 'LineWidth', 2);
plot(traj(1,1), traj(1,2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(traj(end,1), traj(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

legend('Ham Traj', 'Loop Optimize', 'Başlangıç', 'Bitiş');
title('FAZ 3 — Ileri + Geri Yapay Loop Testi');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
grid on;

if ~exist('results', 'dir')
    mkdir('results');
end
if ~exist(fullfile('results','figures'), 'dir')
    mkdir(fullfile('results','figures'));
end

save('results/loop_closure_traj.mat', 'traj', 'optimizedNodes', 'featDB');
saveas(gcf, 'results/figures/loop_closure_forward_reverse.png');

fprintf('Kaydedildi: results/loop_closure_traj.mat\n');

function I = loadAndPrep(path)
    I = imread(path);
    if ndims(I) == 3
        I = rgb2gray(I);
    end
    I = double(I);
    mn = min(I(:));
    mx = max(I(:));
    if mx > mn
        I = (I - mn) / (mx - mn);
    else
        I = zeros(size(I));
    end
end