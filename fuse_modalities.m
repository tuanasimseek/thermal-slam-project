function I = fuse_modalities(I_lwir, I_vis, method)

    % LWIR yoksa çık
    if isempty(I_lwir)
        I = [];
        return;
    end

    % Visible yoksa direkt LWIR kullan
    if isempty(I_vis)
        I = I_lwir;
        return;
    end

    % Gerekirse grayscale yap
    if size(I_lwir,3) == 3
        I_lwir = rgb2gray(I_lwir);
    end

    if size(I_vis,3) == 3
        I_vis = rgb2gray(I_vis);
    end

    % Boyut eşitle
    I_vis = imresize(I_vis, [size(I_lwir,1), size(I_lwir,2)]);

    % Double yap
    I_lwir = im2double(I_lwir);
    I_vis  = im2double(I_vis);

    switch lower(method)
        case 'weighted'
            I = 0.7 * I_lwir + 0.3 * I_vis;

        case 'max'
            I = max(I_lwir, I_vis);

        case 'mean'
            I = (I_lwir + I_vis) / 2;

        otherwise
            I = I_lwir;
    end 