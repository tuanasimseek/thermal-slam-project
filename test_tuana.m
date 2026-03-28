% =========================================================
%  test_tuana.m  —  Tuana'nın modüllerini bağımsız test eder
%  Çalıştırma: >> test_tuana
% =========================================================

clc; clear; close all;

fprintf('========================================\n');
fprintf('  TUANA MODÜLLERİ TEST\n');
fprintf('========================================\n\n');

addpath(fullfile(pwd, 'src', 'odometry'));

%% TEST 1 — CNN modeli
fprintf('[TEST 1] CNN modeli yükleniyor...\n');
try
    net = load_cnn_model();
    fprintf('  GECTI: ResNet-18 yuklendi.\n\n');
catch e
    fprintf('  HATA: %s\n\n', e.message); return;
end

%% TEST 2 — Frame oku
fprintf('[TEST 2] Gercek termal frame okunuyor...\n');
seqDir = fullfile(pwd, 'data', 'set00', 'v000', 'lwir');
fp1 = fullfile(seqDir, 'I00000.jpg');
fp2 = fullfile(seqDir, 'I00003.jpg');

if ~exist(fp1,'file') || ~exist(fp2,'file')
    fprintf('  HATA: Frame bulunamadi: %s\n', seqDir); return;
end

I_raw1 = imread(fp1);
I_raw2 = imread(fp2);
fprintf('  GECTI: Boyut: %dx%d\n\n', size(I_raw1,1), size(I_raw1,2));

%% TEST 3 — Normalize
fprintf('[TEST 3] On isleme...\n');
function I = normalize_frame(I_raw)
    if ndims(I_raw)==3, I = double(rgb2gray(I_raw));
    else, I = double(I_raw); end
    mn=min(I(:)); mx=max(I(:));
    if mx>mn, I=(I-mn)/(mx-mn); else, I=zeros(size(I)); end
end

I1 = normalize_frame(I_raw1);
I2 = normalize_frame(I_raw2);
fprintf('  GECTI: Min=%.3f Max=%.3f\n\n', min(I1(:)), max(I1(:)));

%% TEST 4 — CNN özellik
fprintf('[TEST 4] CNN ozellik cikarimi...\n');
try
    feat1 = feature_cnn(I1, net);
    feat2 = feature_cnn(I2, net);
    fprintf('  GECTI: Boyut=%d  Norm=%.4f\n\n', numel(feat1), norm(feat1));
catch e
    fprintf('  HATA: %s\n\n', e.message); return;
end

%% TEST 5 — Poz tahmini
fprintf('[TEST 5] Poz tahmini (optik akis + CNN)...\n');
try
    [dx, dy, conf] = pose_estimator(I1, I2, feat1, feat2);
    fprintf('  GECTI: dx=%.4f  dy=%.4f  conf=%.4f\n\n', dx, dy, conf);
catch e
    fprintf('  HATA: %s\n\n', e.message); return;
end

%% TEST 6 — 30 frame döngüsü
fprintf('[TEST 6] 30 frame dongusu...\n');
imgs = dir(fullfile(seqDir, 'I*.jpg'));
[~,idx] = sort({imgs.name}); imgs = imgs(idx);
nTest = min(30, numel(imgs));

pose=[0,0]; traj=zeros(nTest,2);
prev_I=[]; prev_feat=[];

for k=1:nTest
    I_raw = imread(fullfile(seqDir, imgs(k).name));
    Ik = normalize_frame(I_raw);
    curr_feat = feature_cnn(Ik, net);

    if ~isempty(prev_feat)
        [dx,dy,conf] = pose_estimator(prev_I, Ik, prev_feat, curr_feat);
        pose(1) = pose(1) + dx;
        pose(2) = pose(2) + dy;
        fprintf('  Frame %2d: pos=(%.4f, %.4f)  dx=%.3f dy=%.3f conf=%.3f\n',...
            k, pose(1), pose(2), dx, dy, conf);
    end
    traj(k,:) = pose;
    prev_I = Ik; prev_feat = curr_feat;
end

fprintf('\n  GECTI: 30 frame tamamlandi.\n\n');

%% Grafik
figure;
plot(traj(:,1), traj(:,2), 'o-', 'LineWidth',2, 'MarkerSize',5);
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Test Trajesi — 30 Frame (Optik Akis + CNN)');
drawnow;

fprintf('========================================\n');
fprintf('  TUM TESTLER TAMAMLANDI\n');
fprintf('========================================\n');