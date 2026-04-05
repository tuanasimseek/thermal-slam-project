function feat = feature_cnn(I, net)
    % Görüntü hazırlık
    if ndims(I) == 3
        I = double(rgb2gray(I));
    else
        I = double(I);
    end
    mn = min(I(:)); mx = max(I(:));
    if mx > mn
        I = (I - mn) / (mx - mn);
    else
        I = zeros(size(I));
    end
    I_resized = imresize(I, [224 224]);
    I_rgb     = single(repmat(I_resized, [1 1 3]));

    % res4b_relu katmanından özellik çek (daha discriminative, makul boyut)
    feat_raw = activations(net, I_rgb, 'res4b_relu', 'OutputAs', 'rows');

    % Düzleştir ve normalize et
    feat_raw = double(feat_raw(:)');
    nrm = norm(feat_raw);
    if nrm > 1e-8
        feat = feat_raw / nrm;
    else
        feat = feat_raw;
    end
end