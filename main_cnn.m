clc; clear; close all;

addpath('src');
addpath(fullfile(pwd, 'visualization'));

%% ============ AYARLAR ============
setName    = 'set00';
dataRoot   = fullfile(pwd, 'dataset');
setDir     = fullfile(dataRoot, setName);
resultsDir = fullfile(pwd, 'results_cnn');

if ~exist(setDir, 'dir')
    error("Set klasörü yok: %s", setDir);
end
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

showEvery   = 0;
R           = 3;
scale       = 0.02;
scoreThresh = 0.35;   % CNN için SSD'den daha gevşek
jumpFrac    = 0.8;
useEveryN   = 6;      % ilk testte biraz yavaşlamayı azaltmak için
alphaLP     = 0.7;
keyInterval = 20;
maxPix      = 4;
confK       = 10;     % CNN için daha düşük tut

%% ============ CNN AĞI ============
net = resnet18;

%% ============ VIDEO KLASÖRLERİNİ BUL ============
videoDirs = dir(setDir);
videoDirs = videoDirs([videoDirs.isdir]);
videoDirs = videoDirs(~ismember({videoDirs.name}, {'.','..'}));

fprintf("Bulunan video klasörü sayısı: %d\n", length(videoDirs));

metrics = struct([]);

%% ============ HER VIDEOYU İŞLE ============
for v = 1:length(videoDirs)

    graph     = PoseGraph();
    videoName = videoDirs(v).name;
    seqDir    = fullfile(setDir, videoName, 'lwir');

    if ~exist(seqDir, 'dir')
        fprintf("SKIP (lwir yok): %s\n", seqDir);
        continue;
    end

    fprintf("\n=== CNN Processing %s / %s ===\n", setName, videoName);

    imgs  = dir(fullfile(seqDir, '*'));
    imgs  = imgs(~[imgs.isdir]);
    names = {imgs.name};
    isJ   = endsWith(lower(names), '.jpg') | endsWith(lower(names), '.jpeg');
    imgs  = imgs(isJ);

    [~, idx] = sort({imgs.name});
    imgs = imgs(idx);

    nFrames = numel(imgs);
    fprintf("Toplam frame: %d\n", nFrames);

    if nFrames < 2
        fprintf("SKIP (frame az)\n");
        continue;
    end

    pose       = [0 0];
    trajectory = zeros(nFrames-1, 2);
    prev       = [];
    prev_dx    = 0;
    prev_dy    = 0;
    keyframe   = [];

    keyframeImages   = {};
    keyframeNodeIds  = [];
    keyframeFrameIds = [];

    invalidCount = 0;
    usedCount    = 0;
    confList     = [];
    pathLength   = 0;

    %% ============ ANA DÖNGÜ ============ 
    for k = 1:min(nFrames, 200)

        [I_lwir, I_vis, ok] = load_frame_pair( ...
            fullfile(setDir, videoName), '', k-1);

        if ~ok
            continue;
        end

        I = fuse_modalities(I_lwir, I_vis, 'weighted');

        if isempty(prev)
            prev     = I;
            keyframe = I;
            graph    = graph.addNode([0 0]);
            keyframeImages{end+1}   = I;
            keyframeNodeIds(end+1)  = 1;
            keyframeFrameIds(end+1) = k;
            continue;
        end

        if mod(k, useEveryN) ~= 0
            prev = I;
            continue;
        end

        usedCount  = usedCount + 1;
        isKeyframe = false;

        if isempty(keyframe)
            keyframe   = I;
            isKeyframe = true;
        elseif mod(k, keyInterval) == 0
            isKeyframe = true;
        end

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

        % ===== CNN tabanlı eşleştirme =====
        [xShift, yShift, bestScore] = estimateShiftCNN(I, tmpl, x0, y0, R, net);

        if bestScore > scoreThresh
            xShift       = 0;
            yShift       = 0;
            invalidCount = invalidCount + 1;
        end

        if abs(xShift) > R * jumpFrac
            xShift = 0;
        end
        if abs(yShift) > R * jumpFrac
            yShift = 0;
        end

        xShift = max(-maxPix, min(maxPix, xShift));
        yShift = max(-maxPix, min(maxPix, yShift));

        confidence = exp(-bestScore * confK);
        xShift     = xShift * confidence;
        yShift     = yShift * confidence;

        confList(end+1) = confidence;

        if abs(xShift) > abs(yShift)
            xShift = 0;
        end

        xShift  = alphaLP * xShift + (1 - alphaLP) * prev_dx;
        yShift  = alphaLP * yShift + (1 - alphaLP) * prev_dy;
        prev_dx = xShift;
        prev_dy = yShift;

        dxMetric = xShift * scale;
        dyMetric = yShift * scale;

        pose(1) = pose(1) + dxMetric;
        pose(2) = pose(2) + dyMetric;
        trajectory(k-1, :) = pose;

        pathLength = pathLength + sqrt(dxMetric^2 + dyMetric^2);

        if isKeyframe
            prevNode = graph.nodeCount;
            graph    = graph.addNode(pose);
            newNode  = graph.nodeCount;
            graph    = graph.addEdge(prevNode, newNode, dxMetric, dyMetric);
            keyframe = I;
            keyframeImages{end+1}   = I;
            keyframeNodeIds(end+1)  = newNode;
            keyframeFrameIds(end+1) = k;
        end

        prev = I;

        if showEvery > 0 && mod(k, showEvery) == 0
            figure(1); clf;
            imagesc(I); axis image off; colormap gray;
            title(sprintf('%s/%s frame %d/%d dx=%.2f dy=%.2f score=%.4f', ...
                setName, videoName, k, nFrames, ...
                xShift, yShift, bestScore), 'Interpreter', 'none');
            drawnow;
        end

    end % for k

    trajectory(:,1) = smoothdata(trajectory(:,1), 'movmean', 5);
    trajectory(:,2) = smoothdata(trajectory(:,2), 'movmean', 5);

    optimizedNodes = GraphOptimizer.optimize(graph, 50, 0.1);

    if isempty(confList)
        meanConf = 0;
    else
        meanConf = mean(confList);
    end

    if usedCount == 0
        invalidRatio = 0;
    else
        invalidRatio = invalidCount / usedCount;
    end

    metrics(v).videoName      = videoName;
    metrics(v).nFrames        = nFrames;
    metrics(v).usedCount      = usedCount;
    metrics(v).invalidCount   = invalidCount;
    metrics(v).invalidRatio   = invalidRatio;
    metrics(v).meanConfidence = meanConf;
    metrics(v).pathLength     = pathLength;
    metrics(v).nNodes         = size(graph.nodes,1);
    metrics(v).nEdges         = size(graph.edges,1);

    outMat = fullfile(resultsDir, ...
        sprintf('%s_%s_traj.mat', setName, videoName));
    save(outMat, 'trajectory');

    outGraph = fullfile(resultsDir, ...
        sprintf('%s_%s_graph.mat', setName, videoName));
    save(outGraph, 'graph', 'optimizedNodes', ...
        'keyframeNodeIds', 'keyframeFrameIds');

    fprintf("Kaydedildi: %s\n", outMat);
    fprintf("Graph kaydedildi: %s\n", outGraph);
    fprintf("Nodes: %d | Edges: %d\n", ...
        size(graph.nodes,1), size(graph.edges,1));
    fprintf("Used: %d | Invalid: %d | Ratio: %.3f | Conf: %.3f | Path: %.3f\n", ...
        usedCount, invalidCount, invalidRatio, meanConf, pathLength);

    opts.title   = sprintf('Thermal SLAM CNN — %s/%s', setName, videoName);
    opts.saveDir = fullfile(resultsDir, 'figures');
    plot_traj(trajectory, optimizedNodes, nFrames, opts);

end % for v

metricsPath = fullfile(resultsDir, sprintf('%s_metrics.mat', setName));
save(metricsPath, 'metrics');
fprintf('\nCNN Metrics kaydedildi: %s\n', metricsPath);
fprintf("\nCNN BİTTİ\n");