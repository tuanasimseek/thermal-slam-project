% Frame'i sayısal kimliğe dönüştürmek ,thermal image → feature vector
% AlexNet sinir ağı → 512 sayılık vektör. "Bu frame'in parmak izi."

%CNN BURADA CLASSIFICATION YAPMIYOR.Bunun yerine: “Bu frame nasıl görünüyor?” diyor.similarity bilgisi üretiyor
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

    %% DERİN ÖĞRENME BURADA
    % --- 4. 'pool5' katmanından 512-D özellik çek ---
    feat_raw = activations(net, I_rgb, 'pool5', 'OutputAs', 'rows'); %ResNet18'in sonlarına yakın bir katmandan özellik çekiyor
    % POOL5 : Bu katman: yüksek seviyeli görsel özellikler çıkarır.
    % mesela sıcak bölgeler ,şekiller, kenarlar, yapılar, termal desenler

    % --- 5. L2 normalize ---
    nrm = norm(feat_raw);
    if nrm > 1e-8
        feat = feat_raw / nrm; %Bu: vektör büyüklüğünü sabitliyor.Amaç: feature karşılaştırmasını daha stabil yapmak
    else
        feat = feat_raw;
    end

end