%% run_set05.m — set05 tek sekans işleyici
clc; clear; close all;
addpath(genpath('src'));

resultsDir = fullfile(pwd, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

seqBase = fullfile(pwd, 'data', 'set05', 'set05');
seqDir  = fullfile(seqBase, 'lwir');

imgs = dir(fullfile(seqDir, '*.jpg'));
[~, idx] = sort({imgs.name});
imgs = imgs(idx);
nFrames = numel(imgs);
fprintf('set05 — Toplam frame: %d\n', nFrames);

% === AYARLAR ===
R           = 8;
scale       = 0.02;
scoreThresh = 0.08;
jumpFrac    = 0.8;
useEveryN   = 3;
alphaLP     = 0.7;
keyInterval = 20;
maxPix      = 4;
confK       = 50;

graph    = PoseGraph();
pose     = [0 0];
trajectory = zeros(nFrames, 2);
prev     = [];
prev_dx  = 0;
prev_dy  = 0;
keyframe = [];
keyframeNodeIds  = [];
keyframeFrameIds = [];
invalidCount = 0;
usedCount    = 0;
confList     = [];
pathLength   = 0;

for k = 1:nFrames
    fname = imgs(k).name;
    I_raw = imread(fullfile(seqDir, fname));
    if ndims(I_raw) == 3, I_raw = rgb2gray(I_raw); end
    I = my_preprocess(double(I_raw));

    if isempty(prev)
        prev     = I;
        keyframe = I;
        graph    = graph.addNode([0 0]);
        keyframeNodeIds(end+1)  = 1;
        keyframeFrameIds(end+1) = k;
        continue;
    end

    if mod(k, useEveryN) ~= 0
        prev = I;
        continue;
    end

    usedCount  = usedCount + 1;
    isKeyframe = mod(k, keyInterval) == 0;

    [~, w] = size(I);
    roiW = round(w * 0.25);
    roiH = round(size(I,1) * 0.18);
    x0   = round(w * 0.5 - roiW/2);
    y0   = round(size(I,1) * 0.60 - roiH/2);
    [hk, wk] = size(keyframe);
    roiW = min(roiW, wk);
    roiH = min(roiH, hk);
    x0   = max(1, min(x0, wk - roiW + 1));
    y0   = max(1, min(y0, hk - roiH + 1));
    tmpl = keyframe(y0:y0+roiH-1, x0:x0+roiW-1);

    [xShift, yShift, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R);

    if bestScore > scoreThresh, xShift = 0; yShift = 0; invalidCount = invalidCount+1; end
    if abs(xShift) > R*jumpFrac, xShift = 0; end
    if abs(yShift) > R*jumpFrac, yShift = 0; end
    xShift = max(-maxPix, min(maxPix, xShift));
    yShift = max(-maxPix, min(maxPix, yShift));

    confidence = exp(-bestScore * confK);
    xShift = xShift * confidence;
    yShift = yShift * confidence;
    confList(end+1) = confidence;

    if abs(xShift) > abs(yShift), xShift = 0; end

    xShift  = alphaLP * xShift + (1-alphaLP) * prev_dx;
    yShift  = alphaLP * yShift + (1-alphaLP) * prev_dy;
    prev_dx = xShift;
    prev_dy = yShift;

    dxMetric = xShift * scale;
    dyMetric = yShift * scale;
    pose(1)  = pose(1) + dxMetric;
    pose(2)  = pose(2) + dyMetric;
    trajectory(k,:) = pose;
    pathLength = pathLength + sqrt(dxMetric^2 + dyMetric^2);

    if isKeyframe
        prevNode = graph.nodeCount;
        graph    = graph.addNode(pose);
        newNode  = graph.nodeCount;
        graph    = graph.addEdge(prevNode, newNode, [dxMetric dyMetric], 1.0);
        keyframe = I;
        keyframeNodeIds(end+1)  = newNode;
        keyframeFrameIds(end+1) = k;
    end

    prev = I;
end

% Smoothing + optimize
trajectory(:,1) = smoothdata(trajectory(:,1), 'movmean', 5);
trajectory(:,2) = smoothdata(trajectory(:,2), 'movmean', 5);
optimizer      = GraphOptimizer(graph);
optimizedNodes = optimizer.optimize();

% Kaydet
save(fullfile(resultsDir, 'set05_traj.mat'), 'trajectory');
save(fullfile(resultsDir, 'set05_graph.mat'), 'graph', 'optimizedNodes');

% Görselleştir
opts.title   = 'Thermal SLAM — set05';
opts.saveDir = fullfile(resultsDir, 'figures');
plot_traj(trajectory, optimizedNodes, nFrames, opts);

fprintf('set05 tamamlandı. Nodes: %d | Path: %.3f\n', graph.nodeCount, pathLength);

%% LOCAL FUNCTION
function [dx, dy, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R)
    [th, tw] = size(tmpl);
    [H, W]   = size(I);
    bestScore = inf; dx = 0; dy = 0;
    for yy = -R:R
        for xx = -R:R
            xs = x0+xx; ys = y0+yy;
            if xs<1||ys<1||(xs+tw-1)>W||(ys+th-1)>H, continue; end
            patch = I(ys:ys+th-1, xs:xs+tw-1);
            d = patch - tmpl;
            score = sum(d(:).^2);
            if score < bestScore
                bestScore = score; dx = xx; dy = yy;
            end
        end
    end
    bestScore = bestScore / (th*tw);
end