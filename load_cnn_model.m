% =========================================================
%  load_cnn_model.m  —  Tuana
%  ResNet-18 modelini yükler.
%
%  Kullanım:
%      net = load_cnn_model();
% =========================================================

function net = load_cnn_model()
    try
        net = resnet18();
        fprintf('[CNN] ResNet-18 yüklendi. Giriş: %dx%dx%d\n', ...
            net.Layers(1).InputSize);
    catch ME
        error(['[CNN] ResNet-18 yüklenemedi.\n' ...
               'Deep Learning Toolbox kurulu mu?\n' ...
               'Hata: %s'], ME.message);
    end
end