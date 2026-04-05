% =========================================================
%  pose_estimator.m  —  Tuana
%  Görev : İki ardışık termal frame arasındaki göreli
%          kamera hareketini (dx, dy) tahmin eder.
%
%  Yöntem:
%      1. Optik akış (Lucas-Kanade) ile piksel hareketi hesapla
%      2. CNN özellik benzerliği ile güven skoru üret
%      3. İkisini birleştirerek (dx, dy, conf) döndür
%
%  Kullanım:
%      [dx, dy, conf] = pose_estimator(I1, I2, feat1, feat2)
%
%  Girdi:
%      I1, I2      — Ardışık normalize termal frame'ler [H x W] double
%      feat1,feat2 — CNN özellik vektörleri [1x512]
%
%  Çıktı:
%      dx, dy — Göreli hareket (piksel cinsinden)
%      conf   — Güven skoru [0, 1]
% =========================================================

function [dx, dy, conf] = pose_estimator(I1, I2, feat1, feat2)

    % --- 1. CNN güven skoru ---
    conf = max(0, min(1, dot(feat1, feat2)));

    
    
    % --- 2. Optik akış ile hareket tahmini ---
    % uint8'e çevir (opticalFlowLK uint8 ister)
    if ndims(I1)==3, I1 = rgb2gray(I1); end
    if ndims(I2)==3, I2 = rgb2gray(I2); end
    I1_u8 = uint8(I1 * 255);
    I2_u8 = uint8(I2 * 255);

    % Lucas-Kanade optik akış
    opticFlow = opticalFlowLK('NoiseThreshold', 0.009);
    estimateFlow(opticFlow, I1_u8);          % 1. frame'i besle
    flow = estimateFlow(opticFlow, I2_u8);   % 2. frame → akış hesapla

    % --- 3. Akış vektörlerinin ortalamasını al ---
    % Tüm piksellerdeki hareketi ortala → global kamera hareketi
    vx = flow.Vx;   % yatay hız [H x W]
    vy = flow.Vy;   % dikey hız [H x W]

    % Küçük akışları filtrele (gürültü)
    magnitude = sqrt(vx.^2 + vy.^2);
    threshold = prctile(magnitude(:), 75);   % üst %25'i kullan
    mask      = magnitude > threshold;

    if sum(mask(:)) > 10
        dx = median(vx(mask));
        dy = median(vy(mask));
    else
        dx = median(vx(:));
        dy = median(vy(:));
    end

    % --- 4. CNN güveniyle ağırlıklandır ---
    % conf yüksekse (benzer frame'ler) hareketi azalt,
    % conf düşükse (farklı frame'ler) hareketi artır.
    % Termal kamerada ardışık frame'ler çok benzer olduğundan
    % conf her zaman yüksek gelir — bu durumda direkt akışı kullan.
    % conf sadece çok düşük olduğunda (0.5 altı) sinyali zayıflat.
    if conf < 0.5
        dx = dx * conf * 2;
        dy = dy * conf * 2;
    end

    % --- 5. Kırp ---
    max_pix = 15;
    dx = max(-max_pix, min(max_pix, dx));
    dy = max(-max_pix, min(max_pix, dy));

end