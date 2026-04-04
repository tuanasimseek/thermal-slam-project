%% evaluate_metrics.m — FAZ 4: ATE/RTE Karşılaştırma
% Fine-tune öncesi (ham optik akış) vs sonrası (fine-tune CNN) karşılaştırır
%% evaluate_metrics.m — FAZ 4: ATE/RTE Karşılaştırma
clc; clear; close all;
addpath(genpath('src'));

%% VERİ HAZIRLA
lwirDir     = fullfile('data', 'set00', 'V000', 'lwir');
useEveryN   = 3;
scaleMetric = 0.05;
alphaLP     = 0.7;

files = dir(fullfile(lwirDir, '*.jpg'));
[~,idx] = sort({files.name}); files = files(idx);
N = numel(files);
fprintf('Frame: %d\n', N);

%% MODELLER
fprintf('Modeller yükleniyor...\n');
netBase = resnet18();
load('results/finetuned_resnet.mat', 'trainedNet');
fprintf('Hazır.\n');

%% DÖNGÜ
traj_base = [0 0]; traj_ft = [0 0];
pos_base  = [0 0]; pos_ft  = [0 0];
dx_filt_b = 0; dy_filt_b = 0;
dx_filt_f = 0; dy_filt_f = 0;

I_prev = loadAndPrep(fullfile(lwirDir, files(1).name));

for i = 2:useEveryN:N
    I_curr = loadAndPrep(fullfile(lwirDir, files(i).name));

    % BASE
    feat_prev_b = extractFeat(I_prev, netBase);
    feat_curr_b = extractFeat(I_curr, netBase);
    [dx_b, dy_b, ~] = pose_estimator(I_prev, I_curr, feat_prev_b, feat_curr_b);
    dx_filt_b = alphaLP*dx_filt_b + (1-alphaLP)*dx_b;
    dy_filt_b = alphaLP*dy_filt_b + (1-alphaLP)*dy_b;
    pos_base  = pos_base + [dx_filt_b, dy_filt_b]*scaleMetric;
    traj_base = [traj_base; pos_base];

    % FINE-TUNE
    feat_prev_f = extractFeatFT(I_prev, trainedNet);
    feat_curr_f = extractFeatFT(I_curr, trainedNet);
    [dx_f, dy_f, ~] = pose_estimator(I_prev, I_curr, feat_prev_f, feat_curr_f);
    dx_filt_f = alphaLP*dx_filt_f + (1-alphaLP)*dx_f;
    dy_filt_f = alphaLP*dy_filt_f + (1-alphaLP)*dy_f;
    pos_ft  = pos_ft + [dx_filt_f, dy_filt_f]*scaleMetric;
    traj_ft = [traj_ft; pos_ft];

    if mod(i,300)==0
        fprintf('Frame %d/%d\n', i, N);
    end

    I_prev = I_curr;
end

%% ATE
minLen   = min(size(traj_base,1), size(traj_ft,1));
t_base   = traj_base(1:minLen,:);
t_ft     = traj_ft(1:minLen,:);
diff_ate = t_base - t_ft;
ATE_ft   = sqrt(mean(sum(diff_ate.^2, 2)));
fprintf('\nATE — Base: 0 (referans) | Fine-tune: %.4f m\n', ATE_ft);

%% RTE
windowSize = 10;
rte_base = []; rte_ft = [];
for i = 1:windowSize:minLen-windowSize
    rte_base(end+1) = norm(t_base(i+windowSize,:) - t_base(i,:));
    rte_ft(end+1)   = norm(t_ft(i+windowSize,:)   - t_ft(i,:));
end
RTE_base = mean(rte_base);
RTE_ft   = mean(rte_ft);
fprintf('RTE — Base: %.4f | Fine-tune: %.4f\n', RTE_base, RTE_ft);

%% GRAFİK
figure('Name','ATE/RTE Karşılaştırma');
subplot(2,2,[1,2]);
plot(t_base(:,1), t_base(:,2), 'b-', 'LineWidth', 2); hold on;
plot(t_ft(:,1),   t_ft(:,2),   'r-', 'LineWidth', 2);
plot(0,0,'go','MarkerSize',10,'MarkerFaceColor','g');
legend('Base ResNet','Fine-tune ResNet','Başlangıç');
title('Trajektori Karşılaştırması'); axis equal; grid on;

subplot(2,2,3);
bar([0, ATE_ft], 'FaceColor', [0.3 0.6 0.9]);
set(gca,'XTickLabel',{'Base','Fine-tune'});
ylabel('ATE (m)'); title('Absolute Trajectory Error'); grid on;

subplot(2,2,4);
bar([RTE_base, RTE_ft], 'FaceColor', [0.9 0.4 0.3]);
set(gca,'XTickLabel',{'Base','Fine-tune'});
ylabel('RTE (m)'); title('Relative Trajectory Error'); grid on;

sgtitle('FAZ 4 — Fine-tune Öncesi vs Sonrası');
if ~exist('results/figures','dir'), mkdir('results/figures'); end
saveas(gcf, 'results/figures/ate_rte_comparison.png');
fprintf('Grafik kaydedildi.\n');

%% ÖZET
fprintf('\n========== SONUÇ TABLOSU ==========\n');
fprintf('%-20s %-10s %-10s\n', 'Metrik', 'Base', 'Fine-tune');
fprintf('%-20s %-10.4f %-10.4f\n', 'ATE (m)', 0.0, ATE_ft);
fprintf('%-20s %-10.4f %-10.4f\n', 'RTE (m)', RTE_base, RTE_ft);
fprintf('====================================\n');

%% YARDIMCI FONKSİYONLAR
function feat = extractFeat(I, net)
    I_rgb    = prepareRGB(I);
    feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs','rows');
    feat_raw = double(feat_raw(:)');
    nrm = norm(feat_raw);
    if nrm > 1e-8, feat = feat_raw/nrm; else, feat = feat_raw; end
end

function feat = extractFeatFT(I, net)
    I_rgb    = prepareRGB(I);
    feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs','rows');
    feat_raw = double(feat_raw(:)');
    nrm = norm(feat_raw);
    if nrm > 1e-8, feat = feat_raw/nrm; else, feat = feat_raw; end
end

function I_rgb = prepareRGB(I)
    I     = imresize(I, [224 224]);
    I_rgb = single(repmat(I, [1 1 3]));
end

function I = loadAndPrep(path)
    I  = imread(path);
    if ndims(I)==3, I=rgb2gray(I); end
    I  = double(I);
    mn = min(I(:)); mx = max(I(:));
    if mx>mn, I=(I-mn)/(mx-mn); else, I=zeros(size(I)); end
end