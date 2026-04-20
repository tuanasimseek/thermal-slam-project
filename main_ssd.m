clc; clear; close all;
addpath(genpath('src'));

%% AYARLAR
setName    = 'set00';
dataRoot   = fullfile(pwd, 'data');
setDir     = fullfile(dataRoot, setName);
resultsDir = fullfile(pwd, 'results_ssd');

if ~exist(setDir,'dir'), error("Set klasörü yok: %s", setDir); end
if ~exist(resultsDir,'dir'), mkdir(resultsDir); end

R           = 8;
scale       = 0.02;
scoreThresh = 0.08;
jumpFrac    = 0.8;
useEveryN   = 3;
alphaLP     = 0.7;
keyInterval = 8;
maxPix      = 4;
confK       = 50;

%% VIDEO KLASÖRLERİ
videoDirs = dir(setDir);
videoDirs = videoDirs([videoDirs.isdir]);
videoDirs = videoDirs(~ismember({videoDirs.name},{'.','..'}));
fprintf("Video sayısı: %d\n", length(videoDirs));

%% HER VIDEO
for v = 1:length(videoDirs)

    graph     = PoseGraph();
    videoName = videoDirs(v).name;
    seqDir    = fullfile(setDir, videoName, 'lwir');

    if ~exist(seqDir,'dir')
        fprintf("SKIP (lwir yok): %s\n", seqDir); continue;
    end

    fprintf("\n=== SSD %s / %s ===\n", setName, videoName);

    imgs = dir(fullfile(seqDir,'*.jpg'));
    [~,idx] = sort({imgs.name});
    imgs = imgs(idx);
    nFrames = numel(imgs);
    if nFrames < 2, continue; end

    pose     = [0 0];
    trajectory = zeros(nFrames-1, 2);
    prev     = [];
    prev_dx  = 0;
    prev_dy  = 0;
    keyframe = [];
    keyframeNodeIds  = [];
    keyframeFrameIds = [];

    for k = 1:min(nFrames, 600)

        I = imread(fullfile(seqDir, imgs(k).name));
        I = my_preprocess(I);

        if isempty(prev)
            prev     = I;
            keyframe = I;
            graph    = graph.addNode([0 0]);
            keyframeNodeIds(end+1)  = 1;
            keyframeFrameIds(end+1) = k;
            continue;
        end

        if mod(k, useEveryN) ~= 0
            prev = I; continue;
        end

        [h, w] = size(I);
        roiW = round(w * 0.25);
        roiH = round(h * 0.18);
        x0   = round(w/2 - roiW/2);
        y0   = round(h*0.6 - roiH/2);

        [hk, wk] = size(keyframe);
        roiW = min(roiW, wk); roiH = min(roiH, hk);
        x0 = max(1, min(x0, wk-roiW+1));
        y0 = max(1, min(y0, hk-roiH+1));

        tmpl = keyframe(y0:y0+roiH-1, x0:x0+roiW-1);
        [xShift, yShift, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R);

        if bestScore > scoreThresh, xShift = 0; yShift = 0; end
        if abs(xShift) > R*jumpFrac, xShift = 0; end
        if abs(yShift) > R*jumpFrac, yShift = 0; end

        xShift = max(-maxPix, min(maxPix, xShift));
        yShift = max(-maxPix, min(maxPix, yShift));

        confidence = exp(-bestScore * confK);
        xShift = xShift * confidence;
        yShift = yShift * confidence;

        if abs(xShift) > abs(yShift), xShift = 0; end

        xShift  = alphaLP*xShift + (1-alphaLP)*prev_dx;
        yShift  = alphaLP*yShift + (1-alphaLP)*prev_dy;
        prev_dx = xShift; prev_dy = yShift;

        dx = xShift * scale;
        dy = yShift * scale;

        pose = pose + [dx dy];
        trajectory(k-1,:) = pose;

        if mod(k, keyInterval) == 0
            prevNode = graph.nodeCount;
            graph    = graph.addNode(pose);
            newNode  = graph.nodeCount;
            graph    = graph.addEdge(prevNode, newNode, dx, dy);
            keyframe = I;
            keyframeNodeIds(end+1)  = newNode;
            keyframeFrameIds(end+1) = k;
        end

        prev = I;
    end

    trajectory(:,1) = smoothdata(trajectory(:,1),'movmean',5);
    trajectory(:,2) = smoothdata(trajectory(:,2),'movmean',5);

    optimizer      = GraphOptimizer(graph);
    optimizedNodes = optimizer.optimize(50, 0.1);

    outMat = fullfile(resultsDir, sprintf('%s_%s_traj.mat', setName, videoName));
    save(outMat, 'trajectory');

    outGraph = fullfile(resultsDir, sprintf('%s_%s_graph.mat', setName, videoName));
    save(outGraph, 'graph', 'optimizedNodes', 'keyframeNodeIds', 'keyframeFrameIds');

    fprintf("Nodes: %d | Edges: %d\n", size(graph.nodes,1), size(graph.edges,1));

    opts.title   = sprintf('Thermal SLAM SSD — %s/%s', setName, videoName);
    opts.saveDir = fullfile(resultsDir, 'figures');
    plot_traj(trajectory, optimizedNodes, nFrames, opts);
end

fprintf("\nSSD BİTTİ\n");

function [dx, dy, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R)
    [th, tw] = size(tmpl);
    [H, W]   = size(I);
    bestScore = inf; dx = 0; dy = 0;
    for yy = -R:R
        for xx = -R:R
            xs = x0+xx; ys = y0+yy;
            if xs<1||ys<1||(xs+tw-1)>W||(ys+th-1)>H, continue; end
            patch = I(ys:ys+th-1, xs:xs+tw-1);
            score = sum((patch(:)-tmpl(:)).^2) / (th*tw);
            if score < bestScore
                bestScore = score; dx = xx; dy = yy;
            end
        end
    end
end