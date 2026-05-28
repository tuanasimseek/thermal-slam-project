function main_dnn_slam(setNames)
% MAIN_DNN_SLAM
% Derin Sinir Agi + Pose Graph tabanli termal SLAM ana pipeline'i.
%
% Once calistirilmasi gerekenler:
%   generate_siamese_odometry_data
%   train_siamese_odometry
%
% Sonra:
%   main_dnn_slam
%   main_dnn_slam({'set00','set01'})

clc; close all;
addpath(genpath('src'));

if nargin < 1 || isempty(setNames)
    setNames = autoDetectSets();
end
if ischar(setNames) || isstring(setNames)
    setNames = cellstr(setNames);
end

modelPath = fullfile('results', 'siamese_odometry_model.mat');
if ~isfile(modelPath)
    error(['DNN odometri modeli bulunamadi: %s\n' ...
           'Once generate_siamese_odometry_data ve train_siamese_odometry calistir.'], modelPath);
end

M = load(modelPath, 'odomModel');
odomModel = M.odomModel;

fprintf('ResNet-18 feature extractor yukleniyor...\n');
baseNet = load_cnn_model();

dataRoot = fullfile(pwd, 'data');

finalDir    = fullfile(pwd, 'results_final');
finalMatDir = fullfile(finalDir, 'mat', 'dnn');
finalFigDir = fullfile(finalDir, 'figures', 'trajectory');
advBaseDir  = fullfile(finalDir, 'figures', 'advanced', 'dnn');

ensureDirs({finalDir, finalMatDir, finalFigDir, advBaseDir});

scaleMetric       = 0.02;
useEveryN         = 3;
keyInterval       = 8;
maxFramesPerVideo = 600;
alphaLP           = 0.65;
minLoopInterval   = 12;
loopThreshold     = 0.9997;
loopMinMargin     = 1.5e-4;
loopCooldown      = 3;
maxLoopEdges      = 5;

allMetrics = struct([]);
mi = 0;

for si = 1:numel(setNames)
    setName = setNames{si};
    setDir  = fullfile(dataRoot, setName);

    if ~isfolder(setDir)
        fprintf('SKIP set yok: %s\n', setDir);
        continue;
    end

    videoDirs = dir(setDir);
    videoDirs = videoDirs([videoDirs.isdir]);
    videoDirs = videoDirs(~ismember({videoDirs.name}, {'.','..'}));

    for vi = 1:numel(videoDirs)
        videoName = videoDirs(vi).name;
        lwirDir   = fullfile(setDir, videoName, 'lwir');

        if ~isfolder(lwirDir)
            fprintf('SKIP lwir yok: %s/%s\n', setName, videoName);
            continue;
        end

        fprintf('\n=== DNN Thermal SLAM | %s/%s ===\n', setName, videoName);

        imgs = dir(fullfile(lwirDir, '*.jpg'));
        [~, idx] = sort({imgs.name});
        imgs = imgs(idx);

        if numel(imgs) < 2
            fprintf('SKIP az frame\n');
            continue;
        end

        maxFrames = min(numel(imgs), maxFramesPerVideo);

        graph = PoseGraph();
        pose = [0, 0, 0];
        trajectory = zeros(maxFrames-1, 3);

        graph = graph.addNode(pose);
        prevKeyPose = pose;
        keyframeNodeIds = 1;
        keyframeFrameIds = 1;

        featDB = zeros(0, 512);
        nodeDB = [];
        loopEdges = [];
        confValues = [];
        lastLoopNode = -inf;

        prev = imread(fullfile(lwirDir, imgs(1).name));
        prev = my_preprocess(prev);

        featPrev = feature_cnn(prev, baseNet);
        featDB(end+1,:) = featPrev;
        nodeDB(end+1) = 1;

        dxLP = 0;
        dyLP = 0;
        dthetaLP = 0;
        lastProcessedFrame = 1;
        processedStep = 0;

        for k = 2:maxFrames
            curr = imread(fullfile(lwirDir, imgs(k).name));
            curr = my_preprocess(curr);

            if (k - lastProcessedFrame) < useEveryN
                continue;
            end

            [dxPix, dyPix, dtheta, conf] = dnn_odometry(odomModel, baseNet, prev, curr);

            dxLP     = alphaLP * dxPix   + (1-alphaLP) * dxLP;
            dyLP     = alphaLP * dyPix   + (1-alphaLP) * dyLP;
            dthetaLP = alphaLP * dtheta  + (1-alphaLP) * dthetaLP;

            localDelta = [dxLP, dyLP] * scaleMetric;
            R = [cos(pose(3)), -sin(pose(3)); sin(pose(3)), cos(pose(3))];
            worldDelta = (R * localDelta(:))';

            pose = pose + [worldDelta, dthetaLP];
            pose(3) = wrapAngleLocal(pose(3));
            trajectory(k-1,:) = pose;

            confValues(end+1) = conf; %#ok<AGROW>
            processedStep = processedStep + 1;

            if mod(processedStep, keyInterval) == 0
                prevNode = graph.nodeCount;
                graph = graph.addNode(pose);
                currNode = graph.nodeCount;

                edgeDelta = pose - prevKeyPose;
                edgeDelta(3) = wrapAngleLocal(edgeDelta(3));

                graph = graph.addEdge(prevNode, currNode, ...
                    edgeDelta(1), edgeDelta(2), edgeDelta(3), max(conf, 0.1), 0);
                prevKeyPose = pose;

                keyframeNodeIds(end+1) = currNode; %#ok<AGROW>
                keyframeFrameIds(end+1) = k; %#ok<AGROW>

                featCurr = feature_cnn(curr, baseNet);
                [loopIdx, simScore, loopInfo] = loop_detector(featDB, featCurr, ...
                    minLoopInterval, loopThreshold, 300, loopMinMargin);

                if loopIdx > 0
                    loopNode = nodeDB(loopIdx);
                    enoughGap = abs(currNode - loopNode) > minLoopInterval;
                    enoughCooldown = (currNode - lastLoopNode) >= loopCooldown;
                    underLimit = size(loopEdges,1) < maxLoopEdges;

                    if enoughGap && enoughCooldown && underLimit
                        loopStrength = min(1, ...
                            max(0, (simScore - loopThreshold) / max(1-loopThreshold, eps)));
                        loopWeight = 0.25 + 0.35 * loopStrength;

                        graph = graph.addEdge(currNode, loopNode, ...
                            0, 0, 0, loopWeight, 1);
                        loopEdges(end+1,:) = [currNode, loopNode, simScore, ...
                            loopInfo.margin, loopWeight]; %#ok<AGROW>
                        lastLoopNode = currNode;
                        fprintf('LOOP edge: node %d -> %d | sim=%.5f margin=%.5f weight=%.3f\n', ...
                            currNode, loopNode, simScore, loopInfo.margin, loopWeight);
                    end
                end

                featDB(end+1,:) = featCurr; %#ok<AGROW>
                nodeDB(end+1) = currNode; %#ok<AGROW>
            end

            prev = curr;
            lastProcessedFrame = k;
        end

        trajectory = trajectory(any(trajectory(:,1:2),2), :);
        if size(trajectory,1) < 2 || graph.nodeCount < 2
            fprintf('SKIP yetersiz trajectory: %s/%s\n', setName, videoName);
            continue;
        end

        trajectory(:,1) = smoothdata(trajectory(:,1), 'movmean', 5);
        trajectory(:,2) = smoothdata(trajectory(:,2), 'movmean', 5);

        optimizer = GraphOptimizer(graph);
        optimizedNodes = optimizer.optimize(120, 0.08);

        meanConf = mean(confValues);
        if isnan(meanConf)
            meanConf = 0;
        end

        prefix = sprintf('%s_%s_dnn', setName, videoName);

        trajPath  = fullfile(finalMatDir, [prefix '_traj.mat']);
        graphPath = fullfile(finalMatDir, [prefix '_graph.mat']);
        confPath  = fullfile(finalMatDir, [prefix '_conf.mat']);

        save(trajPath,  'trajectory');
        save(graphPath, 'graph', 'optimizedNodes', 'keyframeNodeIds', ...
            'keyframeFrameIds', 'loopEdges');
        save(confPath,  'confValues', 'meanConf');

        opts.title = sprintf('Thermal SLAM DNN — %s/%s', setName, videoName);
        opts.saveDir = finalFigDir;
        opts.fileName = [prefix '_trajectory.png'];
        plot_traj(trajectory, optimizedNodes, maxFrames, opts);

        plot_advanced(trajPath, graphPath, '', advBaseDir, 'dnn', confPath);

        mi = mi + 1;
        allMetrics(mi).setName = setName;
        allMetrics(mi).videoName = videoName;
        allMetrics(mi).pathLength = pathLength2D(optimizedNodes(:,1:2));
        allMetrics(mi).finalDrift = norm(optimizedNodes(end,1:2) - optimizedNodes(1,1:2));
        allMetrics(mi).normalizedDrift = allMetrics(mi).finalDrift / max(allMetrics(mi).pathLength, eps);
        allMetrics(mi).loopClosureCount = size(loopEdges,1);
        allMetrics(mi).meanConfidence = meanConf;
        allMetrics(mi).nodeCount = graph.nodeCount;
        allMetrics(mi).edgeCount = size(graph.edges,1);

        fprintf('Kaydedildi: %s\n', trajPath);
        fprintf('Node=%d | Edge=%d | Loop=%d | Endpoint=%.4f | EndRatio=%.4f\n', ...
            graph.nodeCount, size(graph.edges,1), size(loopEdges,1), ...
            allMetrics(mi).finalDrift, allMetrics(mi).normalizedDrift);
    end
end

if ~isempty(allMetrics)
    save(fullfile(finalMatDir, 'dnn_proxy_metrics.mat'), 'allMetrics');
end

fprintf('\nDNN THERMAL SLAM BITTI\n');
end

function ensureDirs(dirs)
for i = 1:numel(dirs)
    if ~isfolder(dirs{i})
        mkdir(dirs{i});
    end
end
end

function setNames = autoDetectSets()
root = fullfile(pwd, 'data');
dirs = dir(root);
dirs = dirs([dirs.isdir]);
dirs = dirs(~ismember({dirs.name}, {'.','..'}));
setNames = {dirs.name};
end

function p = pathLength2D(traj)
if size(traj,1) < 2
    p = 0;
else
    p = sum(sqrt(sum(diff(traj,1,1).^2, 2)));
end
end

function a = wrapAngleLocal(a)
a = mod(a + pi, 2*pi) - pi;
end
