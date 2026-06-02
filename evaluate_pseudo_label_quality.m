function evaluate_pseudo_label_quality(dataPath)
% EVALUATE_PSEUDO_LABEL_QUALITY
% DNN odometri egitiminde kullanilan optik akis tabanli pseudo-label
% verisinin dagilimini ve kalite skorlarini raporlar.
%
% Bu analiz, modelin ground truth ile degil pseudo-label ile egitildigini
% saklamadan; etiketlerin aralik, guven ve hareket dagilimini gosterir.

clc; close all;
addpath(genpath('src'));

if nargin < 1 || isempty(dataPath)
    dataPath = fullfile('results', 'siamese_odometry_data.mat');
end

if ~isfile(dataPath)
    error('Pseudo-label veri dosyasi yok: %s\nOnce generate_siamese_odometry_data calistir.', dataPath);
end

S = load(dataPath, 'odomData');
odomData = S.odomData;

dx = odomData.dx(:);
dy = odomData.dy(:);
dtheta = odomData.dtheta(:);

if isfield(odomData, 'labelConf')
    labelConf = odomData.labelConf(:);
else
    labelConf = NaN(size(dx));
end

motionMag = sqrt(dx.^2 + dy.^2);
N = numel(dx);

quality = struct();
quality.sampleCount = N;
quality.dxMean = mean(dx);
quality.dxStd = std(dx);
quality.dyMean = mean(dy);
quality.dyStd = std(dy);
quality.dthetaMean = mean(dtheta);
quality.dthetaStd = std(dtheta);
quality.motionMean = mean(motionMag);
quality.motionStd = std(motionMag);
quality.labelConfMean = mean(labelConf, 'omitnan');
quality.labelConfMin = min(labelConf, [], 'omitnan');
quality.labelConfMax = max(labelConf, [], 'omitnan');
quality.lowConfidenceRatio = mean(labelConf < 0.35, 'omitnan');
quality.mediumConfidenceRatio = mean(labelConf >= 0.35 & labelConf < 0.65, 'omitnan');
quality.highConfidenceRatio = mean(labelConf >= 0.65, 'omitnan');
quality.note = ['Pseudo-label degerleri Lucas-Kanade optik akis ve robust ', ...
    'kucuk-acili hareket modeli ile uretilmistir; ground truth degildir.'];

outMatDir = fullfile(pwd, 'results_final', 'mat', 'dnn');
outFigDir = fullfile(pwd, 'results_final', 'figures', 'metrics');
ensureDir(outMatDir);
ensureDir(outFigDir);

outMat = fullfile(outMatDir, 'pseudo_label_quality.mat');
save(outMat, 'quality');

fprintf('\nPseudo-label kalite ozeti\n');
fprintf('Ornek sayisi       : %d\n', quality.sampleCount);
fprintf('dx mean/std        : %.4f / %.4f px\n', quality.dxMean, quality.dxStd);
fprintf('dy mean/std        : %.4f / %.4f px\n', quality.dyMean, quality.dyStd);
fprintf('dtheta mean/std    : %.6f / %.6f rad\n', quality.dthetaMean, quality.dthetaStd);
fprintf('motion mean/std    : %.4f / %.4f px\n', quality.motionMean, quality.motionStd);
fprintf('label confidence   : mean %.4f | min %.4f | max %.4f\n', ...
    quality.labelConfMean, quality.labelConfMin, quality.labelConfMax);
fprintf('confidence ratios  : low %.2f | medium %.2f | high %.2f\n', ...
    quality.lowConfidenceRatio, quality.mediumConfidenceRatio, quality.highConfidenceRatio);

fig = figure('Visible','off','Color','w','Position',[100 100 1300 760]);

subplot(2,2,1);
histogram(dx, 60);
title('Pseudo-label dx'); xlabel('dx (px)'); ylabel('Adet'); grid on;

subplot(2,2,2);
histogram(dy, 60);
title('Pseudo-label dy'); xlabel('dy (px)'); ylabel('Adet'); grid on;

subplot(2,2,3);
histogram(dtheta, 60);
title('Pseudo-label dtheta'); xlabel('dtheta (rad)'); ylabel('Adet'); grid on;

subplot(2,2,4);
histogram(labelConf, 40);
title('Pseudo-label Confidence'); xlabel('confidence'); ylabel('Adet'); grid on;

sgtitle(sprintf('Siamese Odometri Pseudo-label Kalite Analizi | N=%d', N));
outPng = fullfile(outFigDir, 'pseudo_label_quality.png');
exportgraphics(fig, outPng, 'Resolution', 300);
close(fig);

fprintf('MAT kaydedildi: %s\n', outMat);
fprintf('PNG kaydedildi: %s\n', outPng);
end

function ensureDir(path)
if ~isfolder(path)
    mkdir(path);
end
end
