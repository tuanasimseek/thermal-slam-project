% =========================================================
%  test_tuana_full.m  —  Tüm frame'leri işle
%  Çalıştırma: >> test_tuana_full
% =========================================================

clc; clear; close all;

addpath(fullfile(pwd, 'src', 'odometry'));

fprintf('CNN yukleniyor...\n');
net = load_cnn_model();
fprintf('Hazir.\n\n');

seqDir = fullfile(pwd, 'data', 'set00', 'v000', 'lwir');
imgs   = dir(fullfile(seqDir, 'I*.jpg'));
[~,idx] = sort({imgs.name});
imgs    = imgs(idx);
nFrames = numel(imgs);
fprintf('Toplam frame: %d\n\n', nFrames);

% --- Yardımcı: normalize ---
    function I = norm_frame(I_raw)
        if ndims(I_raw)==3, I = double(rgb2gray(I_raw));
        else, I = double(I_raw); end
        mn=min(I(:)); mx=max(I(:));
        if mx>mn, I=(I-mn)/(mx-mn); else, I=zeros(size(I)); end
    end

pose       = [0, 0];
trajectory = zeros(nFrames, 2);
prev_I     = [];
prev_feat  = [];
useEveryN  = 3;   % her 3. frame'i işle

fprintf('İşleniyor...\n');
for k = 1:nFrames

    if mod(k, useEveryN) ~= 0
        continue;
    end

    I_raw = imread(fullfile(seqDir, imgs(k).name));
    Ik    = norm_frame(I_raw);
    curr_feat = feature_cnn(Ik, net);

    if ~isempty(prev_feat)
        [dx, dy, ~] = pose_estimator(prev_I, Ik, prev_feat, curr_feat);
        pose(1) = pose(1) + dx;
        pose(2) = pose(2) + dy;
    end

    trajectory(k,:) = pose;
    prev_I    = Ik;
    prev_feat = curr_feat;

    if mod(k, 300) == 0
        fprintf('  Frame %d / %d  |  pos=(%.3f, %.3f)\n', ...
            k, nFrames, pose(1), pose(2));
    end
end

% Sıfır satırlarını temizle
traj = trajectory(any(trajectory,2), :);

fprintf('\nBitti. Toplam adım: %d\n', size(traj,1));
fprintf('X aralığı: %.3f → %.3f\n', min(traj(:,1)), max(traj(:,1)));
fprintf('Y aralığı: %.3f → %.3f\n', min(traj(:,2)), max(traj(:,2)));

% Grafik
figure;
plot(traj(:,1), traj(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(traj(1,1), traj(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');
plot(traj(end,1), traj(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor','r');
grid on; axis equal;
xlabel('X (piksel)'); ylabel('Y (piksel)');
title('Termal SLAM Trajesi — set00/v000 (Tuana CNN + Optik Akis)');
legend('Traje', 'Başlangıç', 'Bitiş', 'Location','best');
drawnow;

% Kaydet
outDir = fullfile(pwd, 'results');
if ~exist(outDir,'dir'), mkdir(outDir); end
save(fullfile(outDir,'set00_v000_traj_tuana.mat'), 'traj');
fprintf('Traje kaydedildi: results/set00_v000_traj_tuana.mat\n');