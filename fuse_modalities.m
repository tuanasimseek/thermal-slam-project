function I_fused = fuse_modalities(I_lwir, I_vis, method)
% fuse_modalities.m — Toolbox gerektirmeyen versiyon

if nargin < 3
    method = 'weighted';
end

% Her ikisini preprocess'ten geçir
I_lwir = my_preprocess(I_lwir);
I_vis  = my_preprocess(I_vis);

% Boyut uyuşmazlığını düzelt
if any(size(I_lwir) ~= size(I_vis))
    I_vis = imresize(I_vis, [size(I_lwir,1), size(I_lwir,2)]);
end

switch lower(method)

    case 'weighted'
        % Ağırlıklı ortalama
        I_fused = 0.7 * I_lwir + 0.3 * I_vis;

    case 'max'
        % Piksel bazlı maksimum
        I_fused = max(I_lwir, I_vis);

    case 'laplacian'
        % Manuel Gaussian kernel — toolbox gerektirmez
        sigma  = 2;
        hsize  = 7;
        [x, y] = meshgrid(-(hsize-1)/2 : (hsize-1)/2);
        h      = exp(-(x.^2 + y.^2) / (2 * sigma^2));
        h      = h / sum(h(:));

        % Yüksek frekans lwir'den
        I_blur_lwir = conv2(I_lwir, h, 'same');
        L_lwir      = I_lwir - I_blur_lwir;

        % Düşük frekans visible'dan
        I_blur_vis  = conv2(I_vis, h, 'same');

        % Birleştir
        I_fused = I_blur_vis + L_lwir;
        I_fused = max(0, min(1, I_fused));

    otherwise
        warning('fuse_modalities: bilinmeyen method, weighted kullaniliyor');
        I_fused = 0.7 * I_lwir + 0.3 * I_vis;
end

% Son normalize
I_min = min(I_fused(:));
I_max = max(I_fused(:));
if (I_max - I_min) > 1e-6
    I_fused = (I_fused - I_min) / (I_max - I_min);
end

end