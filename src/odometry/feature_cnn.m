% =========================================================
% Frame'i sayı dizisine çevirir
% AlexNet sinir ağı → 512 sayılık vektör. "Bu frame'in parmak izi."
%
%  Kullanım:
%      feat = feature_cnn(I, net)
%
%  Girdi:
%      I    — Tek kanallı termal görüntü [H x W], double [0,1]
%      net  — load_cnn_model() ile yüklenmiş ResNet-18
%
%  Çıktı:
%      feat — Özellik vektörü [1 x 512], double, L2 normalize
% =========================================================

function feat = feature_cnn(I, net)

    % --- 1. Giriş kontrolü ---
    if ndims(I) == 3
        I = double(rgb2gray(I));
    else
        I = double(I);
    end

    % --- 2. Normalize et [0, 1] ---
    mn = min(I(:));  mx = max(I(:));
    if mx > mn
        I = (I - mn) / (mx - mn);
    else
        I = zeros(size(I));
    end

    % --- 3. ResNet-18 giriş boyutu: 224x224x3 ---
    I_resized = imresize(I, [224 224]);
    I_rgb     = repmat(I_resized, [1 1 3]);
    I_rgb     = single(I_rgb);

    % --- 4. 'pool5' katmanından 512-D özellik çek ---
    feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs', 'rows');

    % --- 5. L2 normalize ---
    nrm = norm(feat_raw);
    if nrm > 1e-8
        feat = feat_raw / nrm;
    else
        feat = feat_raw;
    end

end