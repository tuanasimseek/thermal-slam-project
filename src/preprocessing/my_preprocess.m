% Termal ön işleme
% Ham frame'i temizler
% Gürültü azalt, kontrast ayarla → CNN'e hazır hale getir


function I = my_preprocess(I)
% my_preprocess.m — Toolbox gerektirmeyen versiyon
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

% ── 2. [0,1] normalize ───────────────────────────────────────
I_min = min(I(:));
I_max = max(I(:));
if (I_max - I_min) > 1e-6
    I = (I - I_min) / (I_max - I_min);
else
    I = zeros(size(I));
end

% ── 3. Manuel histogram equalization ─────────────────────────
nBins  = 256;
edges  = linspace(0, 1, nBins);
counts = histc(I(:), edges);
cdf    = cumsum(counts) / sum(counts);
I      = interp1(edges, cdf, I, 'linear', 'extrap');
I      = max(0, min(1, I));

% ── 4. Manuel Gaussian blur ───────────────────────────────────
sigma  = 0.8;
hsize  = 5;
[x, y] = meshgrid(-(hsize-1)/2 : (hsize-1)/2);
h      = exp(-(x.^2 + y.^2) / (2 * sigma^2));
h      = h / sum(h(:));
I      = conv2(I, h, 'same');

% ── 5. Son normalize ──────────────────────────────────────────
I_min = min(I(:));
I_max = max(I(:));
if (I_max - I_min) > 1e-6
    I = (I - I_min) / (I_max - I_min);
end

end