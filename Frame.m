clc; clear; close all;

%% ============ AYARLAR ============
setName   = 'set00';
dataRoot  = fullfile(pwd, 'data');
setDir    = fullfile(dataRoot, setName);
resultsDir = fullfile(pwd, 'results');

if ~exist(setDir, 'dir')
    error("Set klasörü yok: %s", setDir);
end
if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

% Parametreler (tek yerde)
showEvery   = 0;        % 0 = hiç gösterme (hızlı); 200 gibi yapabilirsin
R           = 8;        % SSD arama yarıçapı
scale       = 0.02;
scoreThresh = 0.08;
jumpFrac    = 0.8;

useEveryN   = 3;
alphaLP     = 0.7;

keyInterval = 20;

maxPix      = 4;
confK       = 50;

%% ============ VIDEO KLASÖRLERİNİ BUL ============
videoDirs = dir(setDir);
videoDirs = videoDirs([videoDirs.isdir]);
videoDirs = videoDirs(~ismember({videoDirs.name},{'.','..'}));

fprintf("Bulunan video klasörü sayısı: %d\n", length(videoDirs));

%% ============ HER VIDEOYU İŞLE ============
for v = 1:length(videoDirs)

    videoName = videoDirs(v).name;
    seqDir = fullfile(setDir, videoName, 'lwir');

    if ~exist(seqDir, 'dir')
        fprintf("SKIP (lwir yok): %s\n", seqDir);
        continue;
    end

    fprintf("\n=== Processing %s / %s ===\n", setName, videoName);

    % --- frame listesini çek ---
    imgs = dir(fullfile(seqDir, '*'));
    imgs = imgs(~[imgs.isdir]);
    names = {imgs.name};
    isJ = endsWith(lower(names), '.jpg') | endsWith(lower(names), '.jpeg');
    imgs = imgs(isJ);

    [~, idx] = sort({imgs.name});
    imgs = imgs(idx);

    nFrames = numel(imgs);
    fprintf("Toplam frame: %d\n", nFrames);

    if nFrames < 2
        fprintf("SKIP (frame az)\n");
        continue;
    end

    % --- bu videoya özel state ---
    pose = [0 0];
    trajectory = zeros(nFrames-1, 2);

    prev = [];
    prev_dx = 0;
    prev_dy = 0;

    keyframe = [];

    %% ============ ANA DÖNGÜ (bu videonun VO'su) ============
    for k = 1:nFrames

        I = imread(fullfile(seqDir, imgs(k).name));
        I = preprocessThermal(I);

        if isempty(prev)
            prev = I;
            keyframe = I;
            continue;
        end

        if mod(k, useEveryN) ~= 0
            prev = I;
            continue;
        end

        % keyframe güncelle
        if isempty(keyframe)
            keyframe = I;
        elseif mod(k, keyInterval) == 0
            keyframe = I;
        end

        % ROI seçimi
        [h,w] = size(I);
        roiW = round(w * 0.25);
        roiH = round(h * 0.18);

        x0 = round(w*0.5 - roiW/2);
        y0 = round(h*0.60 - roiH/2);

        % keyframe sınır güvenliği
        [hk, wk] = size(keyframe);
        roiW = min(roiW, wk);
        roiH = min(roiH, hk);

        x0 = max(1, min(x0, wk - roiW + 1));
        y0 = max(1, min(y0, hk - roiH + 1));

        tmpl = keyframe(y0:y0+roiH-1, x0:x0+roiW-1);

        % SSD shift
        [xShift, yShift, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R);

        % filtreler
        if bestScore > scoreThresh
            xShift = 0; yShift = 0;
        end

        if abs(xShift) > R*jumpFrac, xShift = 0; end
        if abs(yShift) > R*jumpFrac, yShift = 0; end

        xShift = max(-maxPix, min(maxPix, xShift));
        yShift = max(-maxPix, min(maxPix, yShift));

        confidence = exp(-bestScore * confK);
        xShift = xShift * confidence;
        yShift = yShift * confidence;

        % y (ileri) hareketi daha baskın olsun diye:
        if abs(xShift) > abs(yShift)
            xShift = 0;
        end

        % low-pass
        xShift = alphaLP * xShift + (1-alphaLP) * prev_dx;
        yShift = alphaLP * yShift + (1-alphaLP) * prev_dy;
        prev_dx = xShift;
        prev_dy = yShift;

        % pose güncelle
        pose(1) = pose(1) + xShift * scale;
        pose(2) = pose(2) + yShift * scale;

        trajectory(k-1,:) = pose;

        prev = I;

        if showEvery > 0 && mod(k, showEvery) == 0
            figure(1); clf;
            imagesc(I); axis image off; colormap gray;
            title(sprintf("%s/%s  frame %d/%d  dx=%.2f dy=%.2f score=%.4f", ...
                setName, videoName, k, nFrames, xShift, yShift, bestScore), 'Interpreter','none');
            drawnow;
        end
    end

    % smoothing
    trajectory(:,1) = smoothdata(trajectory(:,1), 'movmean', 5);
    trajectory(:,2) = smoothdata(trajectory(:,2), 'movmean', 5);

    % kaydet (her video ayrı)
    outMat = fullfile(resultsDir, sprintf("%s_%s_traj.mat", setName, videoName));
    save(outMat, 'trajectory');

    fprintf("Kaydedildi: %s\n", outMat);

    % (istersen) her video sonunda hızlı çizdir
    figure(2); clf;
    plot(trajectory(:,1), trajectory(:,2), 'LineWidth', 2);
    grid on; axis equal;
    xlabel('X'); ylabel('Y');
    title(sprintf("Trajectory - %s/%s", setName, videoName), 'Interpreter','none');
    drawnow;

end

fprintf("\nBİTTİ ✅\n");

%% ============ LOCAL FUNCTIONS ============

function I = preprocessThermal(I)
    if ndims(I) == 3
        I = I(:,:,1);
    end
    I = double(I);
    mn = min(I(:)); mx = max(I(:));
    if mx > mn
        I = (I - mn) / (mx - mn);
    else
        I = zeros(size(I));
    end
end

function [dx, dy, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R)
    [th, tw] = size(tmpl);
    [H, W] = size(I);

    bestScore = inf;
    dx = 0; dy = 0;

    for yy = -R:R
        for xx = -R:R
            xs = x0 + xx;
            ys = y0 + yy;

            if xs < 1 || ys < 1 || (xs+tw-1) > W || (ys+th-1) > H
                continue;
            end

            patch = I(ys:ys+th-1, xs:xs+tw-1);
            d = patch - tmpl;
            score = sum(d(:).^2);

            if score < bestScore
                bestScore = score;
                dx = xx;
                dy = yy;
            end
        end
    end

    bestScore = bestScore / (th*tw);
end