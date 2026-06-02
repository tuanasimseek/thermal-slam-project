% EVALUATE_METRICS
% Fine-tune öncesi / sonrası proxy trajectory karşılaştırması.
%
% Not:
% Bu dosya ground truth pose kullanmaz. Bu nedenle burada hesaplanan
% değerler klasik SLAM literatüründeki gerçek ATE/RTE metrikleri değildir.
% Fine-tune modelin base modele göre trajectory davranışını karşılaştıran
% proxy değerler olarak raporlanmalıdır.
%
% Çıktıları results_final içine düzenli kaydeder.

clc; clear; close all;
addpath(genpath('src'));

%% AYARLAR
setName   = 'set00';
videoName = 'V000';

lwirDir = fullfile('data', setName, videoName, 'lwir');

useEveryN   = 3;
scaleMetric = 0.05;
alphaLP     = 0.7;

finalDir    = fullfile(pwd, 'results_final');
figDir      = fullfile(finalDir, 'figures', 'metrics');
matDir      = fullfile(finalDir, 'mat', 'metrics');

dirs = {finalDir, figDir, matDir};
for i = 1:numel(dirs)
    if ~exist(dirs{i}, 'dir')
        mkdir(dirs{i});
    end
end

if ~exist(lwirDir, 'dir')
    error("LWIR klasörü yok: %s", lwirDir);
end

%% VERİ HAZIRLA
files = dir(fullfile(lwirDir, '*.jpg'));
[~,idx] = sort({files.name});
files = files(idx);

N = numel(files);
fprintf('Frame: %d\n', N);

if N < 2
    error("Yeterli frame yok.");
end

%% MODELLER
fprintf('Modeller yükleniyor...\n');

netBase = resnet18();

fineTunePath = fullfile('results', 'finetuned_resnet.mat');

if ~exist(fineTunePath, 'file')
    error("Fine-tune model bulunamadı: %s", fineTunePath);
end

load(fineTunePath, 'trainedNet');

fprintf('Hazır.\n');

%% DÖNGÜ
traj_base = [0 0];
traj_ft   = [0 0];

pos_base = [0 0];
pos_ft   = [0 0];

dx_filt_b = 0;
dy_filt_b = 0;

dx_filt_f = 0;
dy_filt_f = 0;

I_prev = loadAndPrep(fullfile(lwirDir, files(1).name));

for k = 2:useEveryN:N

    I_curr = loadAndPrep(fullfile(lwirDir, files(k).name));

    %% BASE RESNET
    feat_prev_b = extractFeat(I_prev, netBase);
    feat_curr_b = extractFeat(I_curr, netBase);

    [dx_b, dy_b, ~] = pose_estimator(I_prev, I_curr, feat_prev_b, feat_curr_b);

    dx_filt_b = alphaLP * dx_filt_b + (1-alphaLP) * dx_b;
    dy_filt_b = alphaLP * dy_filt_b + (1-alphaLP) * dy_b;

    pos_base  = pos_base + [dx_filt_b, dy_filt_b] * scaleMetric;
    traj_base = [traj_base; pos_base]; %#ok<AGROW>

    %% FINE-TUNE RESNET
    feat_prev_f = extractFeatFT(I_prev, trainedNet);
    feat_curr_f = extractFeatFT(I_curr, trainedNet);

    [dx_f, dy_f, ~] = pose_estimator(I_prev, I_curr, feat_prev_f, feat_curr_f);

    dx_filt_f = alphaLP * dx_filt_f + (1-alphaLP) * dx_f;
    dy_filt_f = alphaLP * dy_filt_f + (1-alphaLP) * dy_f;

    pos_ft  = pos_ft + [dx_filt_f, dy_filt_f] * scaleMetric;
    traj_ft = [traj_ft; pos_ft]; %#ok<AGROW>

    if mod(k, 300) == 0
        fprintf('Frame %d/%d\n', k, N);
    end

    I_prev = I_curr;
end

%% PROXY ATE BENZERI DEGER
minLen = min(size(traj_base,1), size(traj_ft,1));

t_base = traj_base(1:minLen,:);
t_ft   = traj_ft(1:minLen,:);

diff_ate = t_base - t_ft;

% Ground truth olmadigi icin bu deger gercek ATE degildir.
% Base model icin baslangica olan kümülatif uzaklik, fine-tune model icin
% base modele gore trajectory farki proxy olarak hesaplanir.
ATE_base = sqrt(mean(sum(t_base.^2, 2)));
ATE_ft   = sqrt(mean(sum(diff_ate.^2, 2)));

fprintf('\nProxy ATE benzeri — Base: %.4f | Fine-tune: %.4f m\n', ATE_base, ATE_ft);

%% PROXY RTE BENZERI DEGER
windowSize = 10;

rte_base = [];
rte_ft   = [];

for i = 1:windowSize:minLen-windowSize

    rte_base(end+1) = norm(t_base(i+windowSize,:) - t_base(i,:)); %#ok<AGROW>
    rte_ft(end+1)   = norm(t_ft(i+windowSize,:)   - t_ft(i,:));   %#ok<AGROW>

end

RTE_base = mean(rte_base);
RTE_ft   = mean(rte_ft);

fprintf('Proxy RTE benzeri — Base: %.4f | Fine-tune: %.4f\n', RTE_base, RTE_ft);

%% METRICS STRUCT
metrics = struct();

metrics.setName   = setName;
metrics.videoName = videoName;

metrics.ATE_base = ATE_base;
metrics.ATE_ft   = ATE_ft;

metrics.RTE_base = RTE_base;
metrics.RTE_ft   = RTE_ft;
metrics.metricNote = ['Ground truth pose kullanilmadigi icin ATE/RTE alanlari ', ...
    'klasik hata metrigi degil, base ve fine-tune trajectory davranisini ', ...
    'karsilastiran proxy degerlerdir.'];

metrics.frameCount = N;
metrics.usedEveryN = useEveryN;

metrics.traj_base = traj_base;
metrics.traj_ft   = traj_ft;

%% GRAFİK
fig = figure('Visible','off', ...
    'Name','Proxy Trajectory Comparison', ...
    'Color','w', ...
    'Position',[100 100 1200 700]);

subplot(2,2,[1,2]);

plot(t_base(:,1), t_base(:,2), 'b-', 'LineWidth', 2);
hold on;

plot(t_ft(:,1), t_ft(:,2), 'r-', 'LineWidth', 2);

plot(0, 0, 'go', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','g');

legend('Base ResNet', 'Fine-tune ResNet', 'Başlangıç', ...
    'Location','best');

title('Trajektori Karşılaştırması');
xlabel('X (m)');
ylabel('Y (m)');
axis equal;
grid on;

subplot(2,2,3);

bar([ATE_base, ATE_ft]);
set(gca, 'XTickLabel', {'Base','Fine-tune'});

ylabel('Proxy değer (m)');
title('ATE Benzeri Proxy');
grid on;

subplot(2,2,4);

bar([RTE_base, RTE_ft]);
set(gca, 'XTickLabel', {'Base','Fine-tune'});

ylabel('Proxy değer (m)');
title('RTE Benzeri Proxy');
grid on;

sgtitle(sprintf('FAZ 4 — Fine-tune Öncesi vs Sonrası Proxy Analiz (%s/%s)', setName, videoName));

%% SAVE
pngPath = fullfile(figDir, sprintf('%s_%s_ate_rte_comparison.png', setName, videoName));
matPath = fullfile(matDir, sprintf('%s_%s_metrics.mat', setName, videoName));

exportgraphics(fig, pngPath, 'Resolution', 300);
save(matPath, 'metrics');

close(fig);

fprintf('\nGrafik kaydedildi: %s\n', pngPath);
fprintf('MAT kaydedildi: %s\n', matPath);

%% ÖZET
fprintf('\n========== SONUÇ TABLOSU ==========\n');
fprintf('%-20s %-12s %-12s\n', 'Metrik', 'Base', 'Fine-tune');
fprintf('%-20s %-12.4f %-12.4f\n', 'Proxy ATE', ATE_base, ATE_ft);
fprintf('%-20s %-12.4f %-12.4f\n', 'Proxy RTE', RTE_base, RTE_ft);
fprintf('Not: Bu degerler ground truth tabanli klasik ATE/RTE degildir.\n');
fprintf('====================================\n');

%% YARDIMCI FONKSİYONLAR
function feat = extractFeat(I, net)

I_rgb = prepareRGB(I);

feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs','rows');
feat_raw = double(feat_raw(:)');

nrm = norm(feat_raw);

if nrm > 1e-8
    feat = feat_raw / nrm;
else
    feat = feat_raw;
end

end

function feat = extractFeatFT(I, net)

I_rgb = prepareRGB(I);

feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs','rows');
feat_raw = double(feat_raw(:)');

nrm = norm(feat_raw);

if nrm > 1e-8
    feat = feat_raw / nrm;
else
    feat = feat_raw;
end

end

function I_rgb = prepareRGB(I)

I = imresize(I, [224 224]);
I_rgb = single(repmat(I, [1 1 3]));

end

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
