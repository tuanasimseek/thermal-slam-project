%% generate_training_data.m — FAZ 4: Tüm veri seti eğitim verisi
clc; clear; close all;
addpath(genpath('src'));

%% TÜM SET-VIDEO ÇİFTLERİ
datasets = {
    'set00', 'V000';
    'set00', 'V001';
    'set00', 'V002';
    'set00', 'V003';
    'set00', 'V004';
    'set00', 'V005';
    'set00', 'V006';
    'set00', 'V007';
    'set00', 'V008';
    'set01', 'V000';
    'set01', 'V001';
    'set01', 'V002';
    'set01', 'V003';
    'set01', 'V004';
    'set01', 'V005';
    'set02', 'V000';
    'set02', 'V001';
    'set02', 'V002';
    'set02', 'V003';
    'set02', 'V004';
    'set03', 'V000';
    'set03', 'V001';
    'set04', 'V000';
    'set04', 'V001';
};

useEveryN  = 3;
maxPerSeq  = 100;  % her videodan max 100 çift

trainData.path1 = {};
trainData.path2 = {};
trainData.dx    = [];
trainData.dy    = [];

totalPairs = 0;

for d = 1:size(datasets, 1)
    setName   = datasets{d,1};
    videoName = datasets{d,2};
    lwirDir   = fullfile('data', setName, videoName, 'lwir');

    if ~exist(lwirDir, 'dir')
        fprintf('SKIP (yok): %s/%s\n', setName, videoName);
        continue;
    end

    files = dir(fullfile(lwirDir, '*.jpg'));
    [~,idx] = sort({files.name});
    files = files(idx);
    N = numel(files);

    if N < 2
        fprintf('SKIP (az frame): %s/%s\n', setName, videoName);
        continue;
    end

    fprintf('\n%s/%s — %d frame işleniyor...\n', setName, videoName, N);
    pairCount = 0;

    for i = 1:useEveryN:N-useEveryN
        if pairCount >= maxPerSeq, break; end

        path1 = fullfile(lwirDir, files(i).name);
        path2 = fullfile(lwirDir, files(i+useEveryN).name);

        I1 = loadAndPrep(path1);
        I2 = loadAndPrep(path2);

        [dx, dy] = computeFlow(I1, I2);

        if abs(dx) < 1e-4 && abs(dy) < 1e-4
            continue;
        end

        pairCount  = pairCount + 1;
        totalPairs = totalPairs + 1;

        trainData.path1{totalPairs} = path1;
        trainData.path2{totalPairs} = path2;
        trainData.dx(totalPairs)    = dx;
        trainData.dy(totalPairs)    = dy;
    end

    fprintf('%s/%s → %d çift eklendi\n', setName, videoName, pairCount);
end

fprintf('\nToplam çift: %d\n', totalPairs);
fprintf('dx aralığı: %.4f → %.4f\n', min(trainData.dx), max(trainData.dx));
fprintf('dy aralığı: %.4f → %.4f\n', min(trainData.dy), max(trainData.dy));

%% KAYDET
if ~exist('results','dir'), mkdir('results'); end
save('results/training_data.mat', 'trainData', '-v7.3');
fprintf('Kaydedildi: results/training_data.mat\n');

%% GRAFİKLER
if ~exist('results/figures','dir'), mkdir('results/figures'); end

figure('Name','Eğitim Verisi Dağılımı');
subplot(1,2,1);
histogram(trainData.dx, 40); title('dx Dağılımı');
xlabel('dx (piksel)'); ylabel('Adet'); grid on;
subplot(1,2,2);
histogram(trainData.dy, 40); title('dy Dağılımı');
xlabel('dy (piksel)'); ylabel('Adet'); grid on;
sgtitle(sprintf('Eğitim Verisi — %d Çift', totalPairs));
saveas(gcf, 'results/figures/training_data_dist.png');
fprintf('Grafik kaydedildi.\n');

%% YARDIMCI FONKSİYONLAR
function [dx, dy] = computeFlow(I1, I2)
    I1_u8 = uint8(I1 * 255);
    I2_u8 = uint8(I2 * 255);
    opticFlow = opticalFlowLK('NoiseThreshold', 0.009);
    estimateFlow(opticFlow, I1_u8);
    flow = estimateFlow(opticFlow, I2_u8);
    vx = flow.Vx; vy = flow.Vy;
    magnitude = sqrt(vx.^2 + vy.^2);
    validMask = magnitude > 1e-3;
    if sum(validMask(:)) < 10
        dx = median(vx(:)); dy = median(vy(:)); return;
    end
    vx_v = vx(validMask); vy_v = vy(validMask);
    mag_v = magnitude(validMask);
    thr = prctile(mag_v, 75);
    mask = mag_v > thr;
    if sum(mask) > 5
        dx = median(vx_v(mask)); dy = median(vy_v(mask));
    else
        dx = median(vx_v); dy = median(vy_v);
    end
end

function I = loadAndPrep(path)
    I = imread(path);
    if ndims(I)==3, I=rgb2gray(I); end
    I = double(I);
    mn=min(I(:)); mx=max(I(:));
    if mx>mn, I=(I-mn)/(mx-mn); else, I=zeros(size(I)); end
end