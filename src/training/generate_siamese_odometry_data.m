function generate_siamese_odometry_data(setNames, maxPerSeq, useEveryN)
% GENERATE_SIAMESE_ODOMETRY_DATA
% Termal frame ciftlerinden Siamese odometri egitimi icin veri uretir.
%
% Not:
% Bu projedeki veri klasorlerinde ground-truth pose bulunmadigi icin
% etiketler optik akis tabanli pseudo-label olarak uretilir. Rapor/metinde
% bunu acikca "pseudo-label" olarak anlatmak gerekir.
%
% Cikti:
%   results/siamese_odometry_data.mat

clc; close all;
addpath(genpath('src'));
rng(42);

if nargin < 1 || isempty(setNames)
    setNames = autoDetectSets();
end
if ischar(setNames) || isstring(setNames)
    setNames = cellstr(setNames);
end
if nargin < 2 || isempty(maxPerSeq)
    maxPerSeq = 250;
end
if nargin < 3 || isempty(useEveryN)
    useEveryN = 3;
end

dataRoot = fullfile(pwd, 'data');

odomData = struct();
odomData.path1     = {};
odomData.path2     = {};
odomData.dx        = [];
odomData.dy        = [];
odomData.dtheta    = [];
odomData.labelConf = [];
odomData.setName   = {};
odomData.videoName = {};
odomData.useEveryN = useEveryN;
odomData.labelNote = 'dx/dy/dtheta pseudo-labels are generated with Lucas-Kanade optical flow and a robust small-angle motion fit; labels are still pseudo-labels, not ground truth.';

totalPairs = 0;

for si = 1:numel(setNames)
    setName = setNames{si};
    setDir  = fullfile(dataRoot, setName);

    if ~isfolder(setDir)
        fprintf('SKIP set yok: %s\n', setDir);
        continue;
    end

    videoDirs = dir(setDir);
    videoDirs = videoDirs([videoDirs.isdir]);
    videoDirs = videoDirs(~ismember({videoDirs.name}, {'.','..'}));

    for vi = 1:numel(videoDirs)
        videoName = videoDirs(vi).name;
        lwirDir   = fullfile(setDir, videoName, 'lwir');

        if ~isfolder(lwirDir)
            fprintf('SKIP lwir yok: %s/%s\n', setName, videoName);
            continue;
        end

        files = dir(fullfile(lwirDir, '*.jpg'));
        [~, idx] = sort({files.name});
        files = files(idx);

        if numel(files) <= useEveryN
            fprintf('SKIP az frame: %s/%s\n', setName, videoName);
            continue;
        end

        fprintf('\n%s/%s | frame=%d | maxPair=%d\n', ...
            setName, videoName, numel(files), maxPerSeq);

        pairCount = 0;
        for k = 1:useEveryN:(numel(files)-useEveryN)
            if pairCount >= maxPerSeq
                break;
            end

            p1 = fullfile(lwirDir, files(k).name);
            p2 = fullfile(lwirDir, files(k+useEveryN).name);

            I1 = loadAndPrep(p1);
            I2 = loadAndPrep(p2);

            [dx, dy, dtheta, conf] = pseudoMotionLabel(I1, I2);

            if conf < 0.25 || (abs(dx) < 1e-4 && abs(dy) < 1e-4 && abs(dtheta) < 1e-5)
                continue;
            end

            pairCount  = pairCount + 1;
            totalPairs = totalPairs + 1;

            odomData.path1{totalPairs,1}     = p1;
            odomData.path2{totalPairs,1}     = p2;
            odomData.dx(totalPairs,1)        = dx;
            odomData.dy(totalPairs,1)        = dy;
            odomData.dtheta(totalPairs,1)    = dtheta;
            odomData.labelConf(totalPairs,1) = conf;
            odomData.setName{totalPairs,1}   = setName;
            odomData.videoName{totalPairs,1} = videoName;
        end

        fprintf('Eklenen cift: %d\n', pairCount);
    end
end

if totalPairs < 10
    error('Yeterli egitim cifti uretilemedi. Veri klasorlerini kontrol et.');
end

if ~isfolder('results')
    mkdir('results');
end
if ~isfolder(fullfile('results','figures'))
    mkdir(fullfile('results','figures'));
end

outPath = fullfile('results', 'siamese_odometry_data.mat');
save(outPath, 'odomData', '-v7.3');

fprintf('\nToplam cift: %d\n', totalPairs);
fprintf('dx araligi: %.4f -> %.4f px\n', min(odomData.dx), max(odomData.dx));
fprintf('dy araligi: %.4f -> %.4f px\n', min(odomData.dy), max(odomData.dy));
fprintf('dtheta araligi: %.5f -> %.5f rad\n', min(odomData.dtheta), max(odomData.dtheta));
fprintf('Kaydedildi: %s\n', outPath);

fig = figure('Visible','off','Color','w','Position',[100 100 1300 420]);
subplot(1,3,1);
histogram(odomData.dx, 50);
title('Siamese odometri dx pseudo-label');
xlabel('dx (px)'); ylabel('Adet'); grid on;
subplot(1,3,2);
histogram(odomData.dy, 50);
title('Siamese odometri dy pseudo-label');
xlabel('dy (px)'); ylabel('Adet'); grid on;
subplot(1,3,3);
histogram(odomData.dtheta, 50);
title('Siamese odometri dtheta pseudo-label');
xlabel('dtheta (rad)'); ylabel('Adet'); grid on;
sgtitle(sprintf('Siamese Odometri Egitim Verisi | %d cift', totalPairs));
exportgraphics(fig, fullfile('results','figures','siamese_odometry_data_dist.png'), 'Resolution', 300);
close(fig);

end

function setNames = autoDetectSets()
root = fullfile(pwd, 'data');
dirs = dir(root);
dirs = dirs([dirs.isdir]);
dirs = dirs(~ismember({dirs.name}, {'.','..'}));
setNames = {dirs.name};
end

function I = loadAndPrep(path)
I = imread(path);
I = my_preprocess(I);
end

function [dx, dy, dtheta, quality] = pseudoMotionLabel(I1, I2)
I1_u8 = uint8(max(0, min(1, I1)) * 255);
I2_u8 = uint8(max(0, min(1, I2)) * 255);

opticFlow = opticalFlowLK('NoiseThreshold', 0.009);
estimateFlow(opticFlow, I1_u8);
flow = estimateFlow(opticFlow, I2_u8);

vx = flow.Vx;
vy = flow.Vy;
mag = sqrt(vx.^2 + vy.^2);
valid = mag > 1e-3 & isfinite(vx) & isfinite(vy);

if sum(valid(:)) < 10
    dx = median(vx(:));
    dy = median(vy(:));
    dtheta = 0;
    quality = 0.05;
    return;
end

[yy, xx] = ndgrid(1:size(vx,1), 1:size(vx,2));
xv = xx(valid);
yv = yy(valid);
vxv = vx(valid);
vyv = vy(valid);

% Cok yogun flow alanini hiz icin ornekle; RANSAC hala tum ornek uzerinde
% yeterince temsilci nokta gorur.
maxSamples = 2500;
if numel(vxv) > maxSamples
    ridx = randperm(numel(vxv), maxSamples);
    xv = xv(ridx);
    yv = yv(ridx);
    vxv = vxv(ridx);
    vyv = vyv(ridx);
end

cx = (size(vx,2) + 1) / 2;
cy = (size(vx,1) + 1) / 2;
xc = xv - cx;
yc = yv - cy;

numIter = 80;
thr = 0.65;
bestMask = false(size(vxv));
bestResidual = inf;

for it = 1:numIter
    if numel(vxv) < 3
        break;
    end

    ridx = randperm(numel(vxv), 3);
    A = zeros(6, 3);
    b = zeros(6, 1);
    for s = 1:3
        rr = 2*s - 1;
        A(rr,:)   = [1, 0, -yc(ridx(s))];
        A(rr+1,:) = [0, 1,  xc(ridx(s))];
        b(rr)     = vxv(ridx(s));
        b(rr+1)   = vyv(ridx(s));
    end

    if rcond(A' * A) < 1e-10
        continue;
    end

    p = A \ b;
    predVx = p(1) - p(3) * yc;
    predVy = p(2) + p(3) * xc;
    residual = sqrt((vxv - predVx).^2 + (vyv - predVy).^2);
    mask = residual < thr;

    if sum(mask) > sum(bestMask) || ...
            (sum(mask) == sum(bestMask) && median(residual(mask)) < bestResidual)
        bestMask = mask;
        if any(mask)
            bestResidual = median(residual(mask));
        end
    end
end

if any(bestMask) && sum(bestMask) >= 12
    A = zeros(2*sum(bestMask), 3);
    b = zeros(2*sum(bestMask), 1);
    inX = xc(bestMask);
    inY = yc(bestMask);
    inVx = vxv(bestMask);
    inVy = vyv(bestMask);

    for s = 1:numel(inVx)
        rr = 2*s - 1;
        A(rr,:)   = [1, 0, -inY(s)];
        A(rr+1,:) = [0, 1,  inX(s)];
        b(rr)     = inVx(s);
        b(rr+1)   = inVy(s);
    end

    p = A \ b;
    dx = p(1);
    dy = p(2);
    dtheta = p(3);

    predVx = dx - dtheta * inY;
    predVy = dy + dtheta * inX;
    residual = sqrt((inVx - predVx).^2 + (inVy - predVy).^2);
    inlierRatio = sum(bestMask) / numel(vxv);
    residualScore = exp(-median(residual));
    quality = max(0.05, min(1.0, inlierRatio * residualScore));
else
    dx = median(vxv);
    dy = median(vyv);
    dtheta = 0;
    quality = 0.1;
end

maxPix = 3.0;
maxTheta = 0.05;
dx = max(-maxPix, min(maxPix, dx));
dy = max(-maxPix, min(maxPix, dy));
dtheta = max(-maxTheta, min(maxTheta, dtheta));
end
