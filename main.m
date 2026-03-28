% =========================================================
%  main.m  —  Termal SLAM Ana Dosyası
%  Proje  : Derin Öğrenme Tabanlı Grafik Tabanlı Termal SLAM
%
%  Yazarlar:
%      Tuana  — CNN ön uç (feature_cnn, pose_estimator, load_cnn_model)
%      Zeynep — Poz grafı ve optimizasyon (PoseGraph, GraphOptimizer)
%      Sema   — Ön işleme ve görselleştirme (preprocess_thermal, plot_trajectory)
%
%  Çalıştırma: >> main
% =========================================================

clc; clear; close all;

%% ============ AYARLAR ============
dataRoot   = fullfile(pwd, 'data');
setName    = 'set00';
resultsDir = fullfile(pwd, 'results');

if ~exist(fullfile(dataRoot, setName), 'dir')
    error('Veri seti bulunamadi: %s', fullfile(dataRoot, setName));
end
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

% Modül yolları
addpath(fullfile(pwd, 'src', 'odometry'));
addpath(fullfile(pwd, 'src', 'preprocessing'));
addpath(fullfile(pwd, 'src', 'visualization'));
addpath(fullfile(pwd, 'src', 'PoseGraph'));

% Parametreler
useEveryN   = 3;       % her N. frame'i işle
scaleMetric = 0.05;    % piksel → metrik ölçek faktörü
alphaLP     = 0.7;     % low-pass filtre katsayısı

% Modül kontrol bayrakları
% Zeynep veya Sema'nın dosyaları henüz yoksa otomatik false olur
USE_POSEGRAPH  = exist('PoseGraph',        'file') == 2;
USE_OPTIMIZER  = exist('GraphOptimizer',   'file') == 2;
USE_PREPROCESS = exist('preprocess_thermal','file') == 2;
USE_PLOT       = exist('plot_trajectory',  'file') == 2;

fprintf('Modul durumu:\n');
fprintf('  PoseGraph     : %s\n', ternary(USE_POSEGRAPH,  'HAZIR','BEKLIYOR'));
fprintf('  GraphOptimizer: %s\n', ternary(USE_OPTIMIZER,  'HAZIR','BEKLIYOR'));
fprintf('  preprocess    : %s\n', ternary(USE_PREPROCESS, 'HAZIR','BEKLIYOR'));
fprintf('  plot_trajectory: %s\n\n', ternary(USE_PLOT,    'HAZIR','BEKLIYOR'));

%% ============ CNN MODELİ YÜKLE (Tuana) ============
fprintf('CNN modeli yukleniyor...\n');
net = load_cnn_model();
fprintf('CNN hazir.\n\n');

%% ============ VİDEO KLASÖRLERİNİ BUL ============
setDir    = fullfile(dataRoot, setName);
videoDirs = dir(setDir);
videoDirs = videoDirs([videoDirs.isdir]);
videoDirs = videoDirs(~ismember({videoDirs.name},{'.','..'}));
fprintf('Bulunan video sayisi: %d\n\n', numel(videoDirs));

%% ============ HER VİDEOYU İŞLE ============
for v = 1:numel(videoDirs)

    videoName = videoDirs(v).name;
    seqDir    = fullfile(setDir, videoName, 'lwir');   % sadece lwir kullanılıyor

    if ~exist(seqDir, 'dir')
        fprintf('SKIP (lwir yok): %s\n', videoName);
        continue;
    end

    fprintf('=== Isleniyor: %s / %s ===\n', setName, videoName);

    % Frame listesi
    imgs = dir(fullfile(seqDir, 'I*.jpg'));
    if isempty(imgs)
        imgs = dir(fullfile(seqDir, '*.jpg'));
    end
    [~,idx] = sort({imgs.name});
    imgs    = imgs(idx);
    nFrames = numel(imgs);
    fprintf('Toplam frame: %d\n', nFrames);

    if nFrames < 2, fprintf('SKIP (az frame)\n'); continue; end

    % Bu videoya özel değişkenler
    pose       = [0, 0];
    trajectory = zeros(nFrames, 2);
    prev_I     = [];
    prev_feat  = [];
    prev_dx    = 0;
    prev_dy    = 0;

    % Poz grafı (Zeynep hazırsa)
    if USE_POSEGRAPH
        pg = PoseGraph();
        pg.addNode(pose);
    end

    %% ---- ANA DÖNGÜ ----
    for k = 1:nFrames

        I_raw = imread(fullfile(seqDir, imgs(k).name));

        % Ön işleme
        if USE_PREPROCESS
            Ik = preprocess_thermal(I_raw);
        else
            Ik = local_normalize(I_raw);
        end

        if mod(k, useEveryN) ~= 0
            continue;
        end

        % CNN özellik çıkar (Tuana)
        curr_feat = feature_cnn(Ik, net);

        if isempty(prev_feat)
            prev_feat = curr_feat;
            prev_I    = Ik;
            continue;
        end

        % Poz tahmini (Tuana)
        [dx, dy, conf] = pose_estimator(prev_I, Ik, prev_feat, curr_feat);

        % Low-pass filtre
        dx = alphaLP * dx + (1 - alphaLP) * prev_dx;
        dy = alphaLP * dy + (1 - alphaLP) * prev_dy;
        prev_dx = dx;
        prev_dy = dy;

        % Poz güncelle
        pose(1) = pose(1) + dx * scaleMetric;
        pose(2) = pose(2) + dy * scaleMetric;
        trajectory(k,:) = pose;

        % Poz grafına ekle (Zeynep hazırsa)
        if USE_POSEGRAPH
            pg.addNode(pose);
            pg.addEdge([dx, dy] * scaleMetric, conf);
        end

        prev_feat = curr_feat;
        prev_I    = Ik;

        if mod(k, 300) == 0
            fprintf('  Frame %d/%d  pos=(%.3f, %.3f)  conf=%.3f\n', ...
                k, nFrames, pose(1), pose(2), conf);
        end
    end

    traj_raw = trajectory(any(trajectory,2), :);

    %% ---- OPTİMİZASYON (Zeynep hazırsa) ----
    if USE_POSEGRAPH && USE_OPTIMIZER
        fprintf('Graf optimizasyonu yapiliyor...\n');
        optimizer = GraphOptimizer(pg);
        traj_opt  = optimizer.optimize();
    else
        fprintf('  [Graf optimizasyonu bekliyor — Zeynep modulleri eksik]\n');
        traj_opt = traj_raw;
    end

    %% ---- GÖRSELLEŞTİRME ----
    if USE_PLOT
        plot_trajectory(traj_raw, traj_opt, sprintf('%s/%s', setName, videoName));
    else
        figure;
        plot(traj_raw(:,1), traj_raw(:,2), 'b-', 'LineWidth', 1.5);
        hold on;
        plot(traj_raw(1,1),   traj_raw(1,2),   'go','MarkerSize',10,'MarkerFaceColor','g');
        plot(traj_raw(end,1), traj_raw(end,2), 'rs','MarkerSize',10,'MarkerFaceColor','r');
        grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title(sprintf('Termal SLAM Trajesi — %s/%s', setName, videoName),'Interpreter','none');
        legend('Traje','Baslangic','Bitis','Location','best');
        drawnow;
    end

    %% ---- KAYDET ----
    outFile = fullfile(resultsDir, sprintf('%s_%s_traj.mat', setName, videoName));
    save(outFile, 'traj_raw', 'traj_opt');
    fprintf('Kaydedildi: %s\n\n', outFile);

end

fprintf('BITTI\n');

%% ============ YEREL YARDIMCI FONKSİYONLAR ============

function I = local_normalize(I_raw)
    if ndims(I_raw) == 3
        I = double(rgb2gray(I_raw));
    else
        I = double(I_raw);
    end
    mn = min(I(:)); mx = max(I(:));
    if mx > mn, I = (I-mn)/(mx-mn);
    else, I = zeros(size(I)); end
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end