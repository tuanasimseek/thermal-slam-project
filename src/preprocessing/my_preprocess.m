% Termal ön işleme
% Ham frame'i temizler
% Gürültü azalt, kontrast ayarla → CNN'e hazır hale getir

function I = my_preprocess(I)
% my_preprocess.m — Toolbox gerektirmeyen termal görüntü ön işleme
% GİRİŞ:  I — ham görüntü (grayscale veya RGB)
% ÇIKIŞ:  I — işlenmiş, double [0,1]

% ── 1. Grayscale ─────────────────────────────────────────────
if size(I, 3) == 3
    I = 0.2989 * double(I(:,:,1)) + ...
        0.5870 * double(I(:,:,2)) + ...
        0.1140 * double(I(:,:,3));
else
    I = double(I);
end

% ── 2. İlk normalize [0,1] ───────────────────────────────────
I_min = min(I(:));
I_max = max(I(:));

if (I_max - I_min) > 1e-6
    I = (I - I_min) / (I_max - I_min);
else
    I = zeros(size(I));
    return;
end

% ── 3. Kontrollü kontrast artırma ────────────────────────────
% Alt %1 ve üst %99 değerleri kullanılır.
% Böylece histogram equalization kadar sert davranmaz.
v = sort(I(:));
n = numel(v);

lowIdx  = max(1, round(0.01 * n));
highIdx = min(n, round(0.99 * n));

lowVal  = v(lowIdx);
highVal = v(highIdx);

if (highVal - lowVal) > 1e-6
    I = (I - lowVal) / (highVal - lowVal);
    I = max(0, min(1, I));
end

% ── 4. Hafif gamma düzeltme ──────────────────────────────────
% Termal görüntüde orta tonları biraz daha belirginleştirir.
gamma = 0.85;
I = I .^ gamma;

% ── 5. Hafif Gaussian blur ───────────────────────────────────
% Aşırı blur CNN detaylarını azaltabilir, bu yüzden sigma düşük tutuldu.
sigma = 0.5;
hsize = 5;

[x, y] = meshgrid(-(hsize-1)/2 : (hsize-1)/2);
h = exp(-(x.^2 + y.^2) / (2 * sigma^2));
h = h / sum(h(:));

I_blur = conv2(I, h, 'same');

% ── 6. Çok hafif keskinleştirme ──────────────────────────────
% Blur sonrası kenar/kontrast bilgisini geri kazandırır.
amount = 0.25;
I = I + amount * (I - I_blur);
I = max(0, min(1, I));

% ── 7. Son normalize ─────────────────────────────────────────
I_min = min(I(:));
I_max = max(I(:));

if (I_max - I_min) > 1e-6
    I = (I - I_min) / (I_max - I_min);
end

end