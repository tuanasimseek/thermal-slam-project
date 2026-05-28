function train_siamese_odometry(dataPath, modelPath)
% TRAIN_SIAMESE_ODOMETRY
% ResNet-18 pool5 ozellikleri uzerinden frame-pair odometri regresoru egitir.
%
% Girdi:
%   results/siamese_odometry_data.mat
%
% Cikti:
%   results/siamese_odometry_model.mat

clc; close all;
addpath(genpath('src'));

if nargin < 1 || isempty(dataPath)
    dataPath = fullfile('results', 'siamese_odometry_data.mat');
end
if nargin < 2 || isempty(modelPath)
    modelPath = fullfile('results', 'siamese_odometry_model.mat');
end

if ~isfile(dataPath)
    error('Egitim verisi yok: %s\nOnce generate_siamese_odometry_data calistir.', dataPath);
end

S = load(dataPath, 'odomData');
odomData = S.odomData;

N = numel(odomData.dx);
if N < 10
    error('Egitim icin yetersiz ornek: %d', N);
end

fprintf('Siamese odometri egitimi | ornek=%d\n', N);
fprintf('ResNet-18 yukleniyor...\n');
baseNet = resnet18();
featureLayer = 'pool5';

X = zeros(N, 2048, 'single');
Y = single([odomData.dx(:), odomData.dy(:), odomData.dtheta(:)]);

fprintf('Frame-pair feature cikariliyor...\n');
for i = 1:N
    I1 = imread(odomData.path1{i});
    I2 = imread(odomData.path2{i});

    f1 = extractPairFeature(I1, baseNet, featureLayer);
    f2 = extractPairFeature(I2, baseNet, featureLayer);

    X(i,:) = single([f1, f2, f2 - f1, abs(f2 - f1)]);

    if mod(i, 100) == 0 || i == N
        fprintf('Feature %d/%d\n', i, N);
    end
end

rng(42);
idx = randperm(N);
X = X(idx,:);
Y = Y(idx,:);

nTrain = max(1, floor(0.8 * N));
XTrain = X(1:nTrain,:);
YTrain = Y(1:nTrain,:);
XVal   = X(nTrain+1:end,:);
YVal   = Y(nTrain+1:end,:);

muX = mean(XTrain, 1);
sdX = std(XTrain, 0, 1);
sdX(sdX < 1e-6) = 1;

muY = mean(YTrain, 1);
sdY = std(YTrain, 0, 1);
sdY(sdY < 1e-6) = 1;

XTrainN = (XTrain - muX) ./ sdX;
YTrainN = (YTrain - muY) ./ sdY;
XValN   = (XVal   - muX) ./ sdX;
YValN   = (YVal   - muY) ./ sdY;

layers = [
    featureInputLayer(size(XTrainN,2), 'Name','pair_features', 'Normalization','none')
    fullyConnectedLayer(512, 'Name','fc1')
    reluLayer('Name','relu1')
    dropoutLayer(0.25, 'Name','drop1')
    fullyConnectedLayer(128, 'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(3, 'Name','motion_out')
    regressionLayer('Name','regression')
];

options = trainingOptions('adam', ...
    'MiniBatchSize', 64, ...
    'MaxEpochs', 40, ...
    'InitialLearnRate', 1e-3, ...
    'LearnRateSchedule', 'piecewise', ...
    'LearnRateDropPeriod', 12, ...
    'LearnRateDropFactor', 0.5, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', {XValN, YValN}, ...
    'ValidationFrequency', 25, ...
    'Verbose', true, ...
    'Plots', 'training-progress');

fprintf('Regression head egitiliyor...\n');
regressionNet = trainNetwork(XTrainN, YTrainN, layers, options);

YPredN = predict(regressionNet, XValN);
YPred  = YPredN .* sdY + muY;

rmse = sqrt(mean((YPred - YVal).^2, 1));
fprintf('\nValidation RMSE | dx=%.4f px | dy=%.4f px | dtheta=%.4f rad\n', ...
    rmse(1), rmse(2), rmse(3));

% DNN regresyon modelleri kucuk hareketleri ortalamaya cekme egilimindedir.
% Validation set uzerinde cikti olcegi icin basit affine kalibrasyon
% hesapliyoruz: y_true ~= outputScale * y_pred + outputBias.
outputScale = ones(1, 3);
outputBias  = zeros(1, 3);
for k = 1:3
    if std(YVal(:,k)) < 1e-8 || std(YPred(:,k)) < 1e-8
        outputScale(k) = 1;
        outputBias(k)  = 0;
        continue;
    end

    beta = [double(YPred(:,k)), ones(size(YPred,1),1)] \ double(YVal(:,k));
    outputScale(k) = beta(1);
    outputBias(k)  = beta(2);
end

YPredCal = YPred .* outputScale + outputBias;
rmseCal = sqrt(mean((YPredCal - YVal).^2, 1));
fprintf('Calibrated RMSE | dx=%.4f px | dy=%.4f px | dtheta=%.4f rad\n', ...
    rmseCal(1), rmseCal(2), rmseCal(3));
fprintf('Output calibration scale=[%.3f %.3f %.3f] bias=[%.4f %.4f %.4f]\n', ...
    outputScale(1), outputScale(2), outputScale(3), ...
    outputBias(1), outputBias(2), outputBias(3));

odomModel = struct();
odomModel.regressionNet = regressionNet;
odomModel.featureLayer  = featureLayer;
odomModel.muX           = muX;
odomModel.sdX           = sdX;
odomModel.muY           = muY;
odomModel.sdY           = sdY;
odomModel.rmse          = rmse;
odomModel.rmseCalibrated = rmseCal;
odomModel.outputScale   = outputScale;
odomModel.outputBias    = outputBias;
odomModel.inputNote     = '[feat1 feat2 feat2-feat1 abs(feat2-feat1)] from ResNet-18 pool5';
odomModel.labelNote     = odomData.labelNote;

if ~isfolder('results')
    mkdir('results');
end
save(modelPath, 'odomModel', '-v7.3');
fprintf('Model kaydedildi: %s\n', modelPath);

if ~isfolder(fullfile('results','figures'))
    mkdir(fullfile('results','figures'));
end

fig = figure('Visible','off','Color','w','Position',[100 100 1200 380]);
names = {'dx','dy','dtheta'};
for k = 1:3
    subplot(1,3,k);
    scatter(YVal(:,k), YPredCal(:,k), 16, 'filled', 'MarkerFaceAlpha', 0.45);
    hold on;
    mn = min([YVal(:,k); YPredCal(:,k)]);
    mx = max([YVal(:,k); YPredCal(:,k)]);
    plot([mn mx], [mn mx], 'r--', 'LineWidth', 1.2);
    title(sprintf('%s | RMSE %.4f', names{k}, rmseCal(k)));
    xlabel('Pseudo-label'); ylabel('DNN tahmin');
    grid on; axis equal;
end
sgtitle('Siamese ResNet Odometri Regresyonu');
exportgraphics(fig, fullfile('results','figures','siamese_odometry_validation.png'), 'Resolution', 300);
close(fig);

end

function feat = extractPairFeature(I, net, featureLayer)
I = my_preprocess(I);
if ndims(I) == 3
    I = rgb2gray(I);
end
I = imresize(I, [224 224]);
I = repmat(single(I), [1 1 3]);

featRaw = activations(net, I, featureLayer, 'OutputAs','rows');
feat = double(featRaw(:))';
n = norm(feat);
if n > 1e-8
    feat = feat / n;
end
end
