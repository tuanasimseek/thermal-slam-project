%% test_tuana_full.m — CNN + RANSAC test
% Sadece Tuana'nın modülleri: load_cnn_model, feature_cnn, pose_estimator
clc; clear; close all;
addpath(genpath('src'));

%% AYARLAR
lwirDir   = fullfile('data', 'set00', 'V000', 'lwir');
useEveryN = 3;
scaleMetric = 0.05;
alphaLP   = 0.7;

%% DOSYALARI YÜKLEarı
files = dir(fullfile(lwirDir, '*.jpg'));
[~, idx] = sort({files.name});
files = files(idx);
N = numel(files);
fprintf('Toplam frame: %d\n', N);

%% CNN MODELİ
fprintf('ResNet-18 yükleniyor...\n');
net = load_cnn_model();
fprintf('Hazır.\n');

%% BAŞLANGIÇ
I_prev    = loadAndPrep(fullfile(lwirDir, files(1).name));
feat_prev = feature_cnn(I_prev, net);
pos       = [0, 0];
traj      = pos;
dx_filt   = 0;
dy_filt   = 0;

%% ANA DÖNGÜ
for i = 2:useEveryN:N
    I_curr    = loadAndPrep(fullfile(lwirDir, files(i).name));
    feat_curr = feature_cnn(I_curr, net);

    [dx, dy, conf] = pose_estimator(I_prev, I_curr, feat_prev, feat_curr);

    dx_filt = alphaLP * dx_filt + (1-alphaLP) * dx;
    dy_filt = alphaLP * dy_filt + (1-alphaLP) * dy;

    pos  = pos + [dx_filt, dy_filt] * scaleMetric;
    traj = [traj; pos];

    if mod(i, 300) == 0
        fprintf('Frame %d/%d | pos=(%.4f, %.4f) | conf=%.3f | dx=%.4f dy=%.4f\n', ...
            i, N, pos(1), pos(2), conf, dx, dy);
    end

    I_prev    = I_curr;
    feat_prev = feat_curr;
end

%% GÖRSELLEŞTİR
figure('Name', 'CNN + RANSAC Trajektori');
plot(traj(:,1), traj(:,2), 'b-', 'LineWidth', 2); hold on;
plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(traj(end,1), traj(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
legend('Traj', 'Başlangıç', 'Bitiş');
title('CNN + RANSAC — set00/V000');
xlabel('X (m)'); ylabel('Y (m)');
axis equal; grid on;

%% KAYDET
if ~exist('results','dir'), mkdir('results'); end
save('results/cnn_ransac_traj.mat', 'traj');
fprintf('\nTamamlandı. Toplam adım: %d\n', size(traj,1));
fprintf('X: %.4f → %.4f\n', min(traj(:,1)), max(traj(:,1)));
fprintf('Y: %.4f → %.4f\n', min(traj(:,2)), max(traj(:,2)));

%% YARDIMCI
function I = loadAndPrep(path)
    I = imread(path);
    if ndims(I)==3, I = rgb2gray(I); end
    I = double(I);
    mn = min(I(:)); mx = max(I(:));
    if mx > mn, I = (I-mn)/(mx-mn); else, I = zeros(size(I)); end
end