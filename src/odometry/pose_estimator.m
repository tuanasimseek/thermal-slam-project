% Frame1 → Frame2 arası hareketi buluyor.ODOMETRY.

function [dx, dy, conf] = pose_estimator(I1, I2, feat1, feat2)
% Optik akış + CNN güven + RANSAC filtresi
% GİRİŞ:  I1, I2    → normalize double [0,1] grayscale görüntüler
%         feat1,feat2 → feature_cnn'den gelen 512-boyutlu vektörler
% ÇIKIŞ:  dx, dy    → piksel cinsinden hareket tahmini
%         conf      → [0,1] güven skoru (RANSAC inlier oranı)

%% 1. GÖRÜNTÜ HAZIRLA
if ndims(I1)==3, I1 = rgb2gray(I1); end
if ndims(I2)==3, I2 = rgb2gray(I2); end

if isa(I1,'double')
    I1_u8 = uint8(I1 * 255);
    I2_u8 = uint8(I2 * 255);
else
    I1_u8 = I1;
    I2_u8 = I2;
end

%% 2. OPTİK AKIŞ (Lucas-Kanade)
opticFlow = opticalFlowLK('NoiseThreshold', 0.009); %piksel hareketlerini takip etmek
% yani "Bu sıcak piksel nereye kaydı?" : vx → x yönü hareket , vy → y yönü hareket
estimateFlow(opticFlow, I1_u8);
flow = estimateFlow(opticFlow, I2_u8);

vx = flow.Vx;
vy = flow.Vy;

%% 3. RANSAC FİLTRESİ — conf artık buradan geliyor
%neden gerekli : Optical flow çok gürültülü. Bazı pikseller: yanlış hareket eder,titreşim olur
%ransac gorevi : yanlış motion vectorlerini elemek

[dx, dy, conf] = ransacMedian(vx, vy);

%% 4. CNN GÜVEN AĞIRLIĞI (kosinüs benzerliği yardımcı bilgi olarak kullanılır)
cnn_sim = max(0, min(1, dot(feat1, feat2))); %dot : cosine similarity.Eğer iki frame feature olarak benziyorsa: motion tahmini daha güvenilir olabilir

% İkisini birleştir: RANSAC inlier oranı ağırlıklı, CNN ikincil
conf = 0.7 * conf + 0.3 * cnn_sim; 

%% 5. DÜŞÜK GÜVENDE HAREKETİ KÜÇÜLT
%trajectory patlamasını önlüyor.
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
function [dx, dy, inlierRatio] = ransacMedian(vx, vy)
% Aykırı vektörleri eler, kalan inlier'lardan median alır
% DÜZELTME: conf yerine inlierRatio döndürüyor

magnitude = sqrt(vx.^2 + vy.^2);

validMask = magnitude > 1e-3;

if sum(validMask(:)) < 10
    dx = median(vx(:)); % Median alınarak: çok daha stabil motion bulunuyor.
    dy = median(vy(:));
    inlierRatio = 0.1;  % çok az piksel — düşük güven
    return;
end

vx_valid = vx(validMask);
vy_valid = vy(validMask);

numIter    = 50;
inlierThr  = 1.5;
bestInliers = [];

for iter = 1:numIter
    idx = randi(length(vx_valid));
    hyp_vx = vx_valid(idx); %rastgele bir motion hipotezi seçiyor.
    hyp_vy = vy_valid(idx);

    dist = sqrt((vx_valid - hyp_vx).^2 + (vy_valid - hyp_vy).^2); 
    inliers = dist < inlierThr; % ona benzeyen motion'ları topluyor.

    if sum(inliers) > length(inliers) * 0.3
        if sum(inliers) > length(bestInliers)
            bestInliers = inliers;
        end
    end
end

if ~isempty(bestInliers) && sum(bestInliers) > 5
    dx = median(vx_valid(bestInliers));
    dy = median(vy_valid(bestInliers));
    
    %CONFIDENCE NASIL HESAPLANIYOR?
    inlierRatio = sum(bestInliers) / length(vx_valid);%Eğer: çok fazla piksel aynı hareketi söylüyorsa:yüksek güven
else
    % RANSAC başarısız → üst %25 harekete geri dön
    mag_valid = magnitude(validMask);
    threshold = prctile(mag_valid, 75);
    mask75 = mag_valid > threshold;

    if sum(mask75) > 5
        dx = median(vx_valid(mask75));
        dy = median(vy_valid(mask75));
    else
        dx = median(vx_valid);
        dy = median(vy_valid);
    end

    inlierRatio = 0.2;  % RANSAC tutmadı — düşük güven
end

end