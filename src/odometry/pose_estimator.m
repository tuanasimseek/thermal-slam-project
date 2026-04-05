function [dx, dy, conf] = pose_estimator(I1, I2, feat1, feat2)
% pose_estimator.m — Optik akış + CNN güven + RANSAC filtresi
% GİRİŞ:  I1, I2    → normalize double [0,1] grayscale görüntüler
%         feat1,feat2 → feature_cnn'den gelen 512-boyutlu vektörler
% ÇIKIŞ:  dx, dy    → piksel cinsinden hareket tahmini
%         conf      → [0,1] güven skoru

%% 1. CNN GÜVEN SKORU
conf = max(0, min(1, dot(feat1, feat2)));

%% 2. GÖRÜNTÜ HAZIRLA
if ndims(I1)==3, I1 = rgb2gray(I1); end
if ndims(I2)==3, I2 = rgb2gray(I2); end

% uint8'e çevir (optik akış için)
if isa(I1,'double')
    I1_u8 = uint8(I1 * 255);
    I2_u8 = uint8(I2 * 255);
else
    I1_u8 = I1;
    I2_u8 = I2;
end

%% 3. OPTİK AKIŞ (Lucas-Kanade)
opticFlow = opticalFlowLK('NoiseThreshold', 0.009);
estimateFlow(opticFlow, I1_u8);
flow = estimateFlow(opticFlow, I2_u8);

vx = flow.Vx;
vy = flow.Vy;

%% 4. RANSAC FİLTRESİ
[dx, dy] = ransacMedian(vx, vy);

%% 5. CNN GÜVEN AĞIRLIĞI
if conf < 0.5
    dx = dx * conf * 2;
    dy = dy * conf * 2;
end

%% 6. SINIRLA
max_pix = 15;
dx = max(-max_pix, min(max_pix, dx));
dy = max(-max_pix, min(max_pix, dy));

end

%% ============ RANSAC YARDIMCI FONKSİYONU ============
function [dx, dy] = ransacMedian(vx, vy)
% Aykırı vektörleri eler, kalan inlier'lardan median alır

magnitude = sqrt(vx.^2 + vy.^2);

% Sıfır hareketi olan pikselleri çıkar
validMask = magnitude > 1e-3;

if sum(validMask(:)) < 10
    % Yeterli piksel yoksa tüm görüntünün medianı
    dx = median(vx(:));
    dy = median(vy(:));
    return;
end

vx_valid = vx(validMask);
vy_valid = vy(validMask);
mag_valid = magnitude(validMask);

% RANSAC parametreleri
numIter    = 50;
inlierThr  = 1.5;   % piksel — bu eşiğin altındakiler inlier
bestInliers = [];

for iter = 1:numIter
    % Rastgele 1 piksel seç, o pikselin hareketini hipotez yap
    idx = randi(length(vx_valid));
    hyp_vx = vx_valid(idx);
    hyp_vy = vy_valid(idx);

    % Tüm vektörlerin bu hipoteze uzaklığı
    dist = sqrt((vx_valid - hyp_vx).^2 + (vy_valid - hyp_vy).^2);

    inliers = dist < inlierThr;

    if sum(inliers) > length(inliers) * 0.3  % en az %30 inlier olsun
        if sum(inliers) > length(bestInliers)
            bestInliers = inliers;
        end
    end
end

% En iyi inlier seti ile median al
if ~isempty(bestInliers) && sum(bestInliers) > 5
    dx = median(vx_valid(bestInliers));
    dy = median(vy_valid(bestInliers));
else
    % RANSAC başarısız → üst %25 harekete geri dön
    threshold = prctile(mag_valid, 75);
    mask75 = mag_valid > threshold;
    if sum(mask75) > 5
        dx = median(vx_valid(mask75));
        dy = median(vy_valid(mask75));
    else
        dx = median(vx_valid);
        dy = median(vy_valid);
    end
end
end