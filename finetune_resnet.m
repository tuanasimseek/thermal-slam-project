%% finetune_resnet.m — FAZ 4: ResNet-18 Termal Fine-tune
clc; clear; close all;
addpath(genpath('src'));

fprintf('Eğitim verisi yükleniyor...\n');
load('results/training_data.mat', 'trainData');
N = length(trainData.dx);
fprintf('Toplam çift: %d\n', N);

%% VERİYİ KARIŞTIRIR
rng(42);
shuffleIdx = randperm(N);
trainData.path1 = trainData.path1(shuffleIdx);
trainData.path2 = trainData.path2(shuffleIdx);
trainData.dx    = trainData.dx(shuffleIdx);
trainData.dy    = trainData.dy(shuffleIdx);

%% TRAIN / VAL BÖLE (80/20)
splitIdx  = floor(0.8 * N);
trainIdx  = 1:splitIdx;
valIdx    = splitIdx+1:N;
fprintf('Train: %d | Val: %d\n', length(trainIdx), length(valIdx));

%% BASE MODEL
fprintf('ResNet-18 yükleniyor...\n');
net = resnet18();

%% ÖZELLİK ÇIKARICI — res4b_relu'ya kadar dondur
layerGraph = layerGraph(net);

% Son 3 katmanı kaldır (pool5, fc1000, prob, ClassificationLayer)
layerGraph = removeLayers(layerGraph, 'fc1000');
layerGraph = removeLayers(layerGraph, 'prob');
layerGraph = removeLayers(layerGraph, 'ClassificationLayer_predictions');

% Regresyon katmanları ekle
newLayers = [
    fullyConnectedLayer(256, 'Name', 'fc_1', ...
        'WeightLearnRateFactor', 10, 'BiasLearnRateFactor', 10)
    reluLayer('Name', 'relu_fc1')
    dropoutLayer(0.3, 'Name', 'dropout1')
    fullyConnectedLayer(64, 'Name', 'fc_2', ...
        'WeightLearnRateFactor', 10, 'BiasLearnRateFactor', 10)
    reluLayer('Name', 'relu_fc2')
    fullyConnectedLayer(2, 'Name', 'fc_out', ...
        'WeightLearnRateFactor', 10, 'BiasLearnRateFactor', 10)
    regressionLayer('Name', 'output')
];

layerGraph = addLayers(layerGraph, newLayers);
layerGraph = connectLayers(layerGraph, 'pool5', 'fc_1');

fprintf('Yeni mimari hazır.\n');

%% EĞİTİM VERİSİ HAZIRLA
fprintf('Özellikler hazırlanıyor...\n');

% Tüm görüntüleri işle, özellik + etiket matrisi oluştur
% Her çift için: I1 ve I2'yi birleştirip 224x224x6 tensor yap
% Ama basit tutmak için: sadece I1 özelliklerini kullan

[XTrain, YTrain] = prepareData(trainData, trainIdx);
[XVal,   YVal  ] = prepareData(trainData, valIdx);

fprintf('XTrain boyutu: %s\n', mat2str(size(XTrain)));
fprintf('YTrain boyutu: %s\n', mat2str(size(YTrain)));

%% EĞİTİM AYARLARI
options = trainingOptions('adam', ...
    'MiniBatchSize',        32, ...
    'MaxEpochs',            15, ...
    'InitialLearnRate',     1e-4, ...
    'LearnRateSchedule',    'piecewise', ...
    'LearnRateDropFactor',  0.5, ...
    'LearnRateDropPeriod',  5, ...
    'ValidationData',       {XVal, YVal}, ...
    'ValidationFrequency',  20, ...
    'Shuffle',              'every-epoch', ...
    'Verbose',              true, ...
    'Plots',                'training-progress');

%% EĞİT
fprintf('\nEğitim başlıyor...\n');
trainedNet = trainNetwork(XTrain, YTrain, layerGraph, options);

%% KAYDET
if ~exist('results','dir'), mkdir('results'); end
save('results/finetuned_resnet.mat', 'trainedNet');
fprintf('Model kaydedildi: results/finetuned_resnet.mat\n');

%% VALİDASYON TAHMİNİ
YPred = predict(trainedNet, XVal);
rmse_dx = sqrt(mean((YPred(:,1) - YVal(:,1)).^2));
rmse_dy = sqrt(mean((YPred(:,2) - YVal(:,2)).^2));
fprintf('\nValidasyon RMSE — dx: %.4f | dy: %.4f\n', rmse_dx, rmse_dy);

%% SONUÇ GRAFİĞİ
figure('Name','Fine-tune Sonuç');
subplot(1,2,1);
scatter(YVal(:,1), YPred(:,1), 20, 'filled', 'Alpha', 0.5);
hold on; plot([-3 3],[-3 3],'r--','LineWidth',1.5);
xlabel('Gerçek dx'); ylabel('Tahmin dx');
title(sprintf('dx | RMSE=%.4f', rmse_dx)); grid on; axis equal;

subplot(1,2,2);
scatter(YVal(:,2), YPred(:,2), 20, 'filled', 'Alpha', 0.5);
hold on; plot([-3 3],[-3 3],'r--','LineWidth',1.5);
xlabel('Gerçek dy'); ylabel('Tahmin dy');
title(sprintf('dy | RMSE=%.4f', rmse_dy)); grid on; axis equal;

sgtitle('ResNet Fine-tune — Tahmin vs Gerçek');
saveas(gcf, 'results/figures/finetune_result.png');
fprintf('Grafik kaydedildi.\n');

%% YARDIMCI FONKSİYON
function [X, Y] = prepareData(trainData, indices)
    nSamples = length(indices);
    X = zeros(224, 224, 3, nSamples, 'single');
    Y = zeros(nSamples, 2);

    for k = 1:nSamples
        idx = indices(k);
        I = imread(trainData.path1{idx});
        if ndims(I)==3, I=rgb2gray(I); end
        I = single(I);
        mn=min(I(:)); mx=max(I(:));
        if mx>mn, I=(I-mn)/(mx-mn); end
        I = imresize(I,[224 224]);
        X(:,:,:,k) = repmat(I,[1 1 3]);
        Y(k,1) = trainData.dx(idx);
        Y(k,2) = trainData.dy(idx);
    end
end