% MAIN_CNN
% CNN + RANSAC tabanlı SLAM
% Çıktıları hem results_cnn hem results_final içine düzenli kaydeder

clc; clear; close all;
addpath(genpath('src'));

%% AYARLAR
setName  = 'set00';
dataRoot = fullfile(pwd, 'data');
setDir   = fullfile(dataRoot, setName);

resultsDir = fullfile(pwd, 'results_cnn');

finalDir      = fullfile(pwd, 'results_final');
finalMatDir   = fullfile(finalDir, 'mat', 'cnn');
finalFigDir   = fullfile(finalDir, 'figures', 'trajectory');

dirs = {resultsDir, finalDir, finalMatDir, finalFigDir};
for i=1:numel(dirs)
    if ~exist(dirs{i},'dir')
        mkdir(dirs{i});
    end
end

scale       = 0.02;
useEveryN   = 3;
alphaLP     = 0.7;
keyInterval = 8;

%% CNN MODEL
net = load_cnn_model();

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
        fprintf("SKIP: %s\n", seqDir);
        continue;
    end

    fprintf("\n=== CNN %s / %s ===\n", setName, videoName);

    imgs = dir(fullfile(seqDir,'*.jpg'));
    [~,idx] = sort({imgs.name});
    imgs = imgs(idx);

    nFrames = numel(imgs);
    if nFrames < 2
        continue;
    end

    maxFrames = min(nFrames, 600);

    pose       = [0 0];
    trajectory = zeros(maxFrames-1, 2);

    prev = [];
    prev_feat = [];

    prev_dx = 0;
    prev_dy = 0;

    keyframeNodeIds  = [];
    keyframeFrameIds = [];

    %% LOOP
    for k = 1:maxFrames

        I = imread(fullfile(seqDir, imgs(k).name));
        I = my_preprocess(I);

        feat = feature_cnn(I, net);

        if isempty(prev)
            prev = I;
            prev_feat = feat;

            graph = graph.addNode([0 0]);

            keyframeNodeIds(end+1)  = 1;
            keyframeFrameIds(end+1) = k;
            continue;
        end

        if mod(k, useEveryN) ~= 0
            prev = I;
            prev_feat = feat;
            continue;
        end

        %% ===== CNN + RANSAC POSE =====
        [dx_pix, dy_pix, conf] = pose_estimator(prev, I, prev_feat, feat);

        %% SMOOTH
        dx_pix = alphaLP*dx_pix + (1-alphaLP)*prev_dx;
        dy_pix = alphaLP*dy_pix + (1-alphaLP)*prev_dy;

        prev_dx = dx_pix;
        prev_dy = dy_pix;

        dx = dx_pix * scale;
        dy = dy_pix * scale;

        pose = pose + [dx dy];
        trajectory(k-1,:) = pose;

        %% GRAPH
        if mod(k, keyInterval) == 0
            prevNode = graph.nodeCount;

            graph = graph.addNode(pose);
            newNode = graph.nodeCount;

            graph = graph.addEdge(prevNode, newNode, dx, dy);

            keyframeNodeIds(end+1)  = newNode;
            keyframeFrameIds(end+1) = k;
        end

        prev = I;
        prev_feat = feat;
    end

    %% TEMİZLE
    trajectory = trajectory(any(trajectory,2),:);

    if size(trajectory,1) < 2
        continue;
    end

    %% SMOOTH
    trajectory(:,1) = smoothdata(trajectory(:,1),'movmean',5);
    trajectory(:,2) = smoothdata(trajectory(:,2),'movmean',5);

    %% OPTIMIZATION
    optimizer      = GraphOptimizer(graph);
    optimizedNodes = optimizer.optimize(50, 0.1);

    %% MAT SAVE (RAW)
    outMat = fullfile(resultsDir, sprintf('%s_%s_traj.mat', setName, videoName));
    save(outMat, 'trajectory');

    outGraph = fullfile(resultsDir, sprintf('%s_%s_graph.mat', setName, videoName));
    save(outGraph, 'graph', 'optimizedNodes', 'keyframeNodeIds', 'keyframeFrameIds');

    %% MAT SAVE (FINAL)
    finalTrajMat = fullfile(finalMatDir, sprintf('%s_%s_cnn_traj.mat', setName, videoName));
    finalGraphMat = fullfile(finalMatDir, sprintf('%s_%s_cnn_graph.mat', setName, videoName));

    save(finalTrajMat, 'trajectory');
    save(finalGraphMat, 'graph', 'optimizedNodes', 'keyframeNodeIds', 'keyframeFrameIds');

    %% PNG SAVE
    opts.title    = sprintf('Thermal SLAM CNN — %s/%s', setName, videoName);
    opts.saveDir  = finalFigDir;
    opts.fileName = sprintf('%s_%s_cnn_trajectory.png', setName, videoName);

    plot_traj(trajectory, optimizedNodes, maxFrames, opts);

    fprintf("Kaydedildi (CNN): %s\n", finalTrajMat);

end

fprintf("\nCNN BİTTİ\n");