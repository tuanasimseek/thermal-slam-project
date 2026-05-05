%---function [I_lwir, I_vis, success] = load_frame_pair(baseDir, seqName, frameIdx)
% load_frame_pair.m — Sema'nın dataset yönetim modülü
% Aynı frame numarası için hem termal hem visible görüntüyü okur.
%
% GİRİŞ:
%   baseDir  — dataset kök dizini (örn: 'data/set00/V000')
%   seqName  — şu an kullanılmıyor, ileride çoklu seq için
%   frameIdx — frame numarası (0'dan başlayan tam sayı)
%
% ÇIKIŞ:a
%   I_lwir   — termal görüntü, double [0,1], grayscale
%   I_vis    — visible görüntü, double [0,1], grayscale
%   success  — true/false (her iki dosya da okunduysa true)

% ── Frame dosya adını oluştur ─────────────────────────────────
% Format: I00000.jpg, I00001.jpg, ...
%---fileName = sprintf('I%05d.jpg', frameIdx);

% ── Tam yolları oluştur ───────────────────────────────────────
%---lwirPath = fullfile(baseDir, 'lwir',    fileName);
%---visPath  = fullfile(baseDir, 'visible', fileName);

% ── Varsayılan çıkış (hata durumu için) ──────────────────────
%---I_lwir  = [];
%---I_vis   = [];
%---success = false;

% ── Lwir dosyasını oku ────────────────────────────────────────
%---if ~isfile(lwirPath)
    %---warning('load_frame_pair: lwir dosyası bulunamadı: %s', lwirPath);
    %---return;
%---end
%---I_lwir = imread(lwirPath);

% ── Visible dosyasını oku ─────────────────────────────────────
% Visible yoksa lwir'i kopyala (sistem çalışmaya devam eder)
%---if ~isfile(visPath)
    %---warning('load_frame_pair: visible yok, lwir kullanılıyor: %s', visPath);
    %---I_vis = I_lwir;
%---else
    %---I_vis = imread(visPath);
%---end

%---success = true;

%---end

function [I_lwir, I_vis, ok] = load_frame_pair(baseDir, ~, idx)

    lwirDir = fullfile(baseDir, 'lwir');
    visDir  = fullfile(baseDir, 'visible');

    % LWIR dosyalarını al
    lwirFiles = dir(fullfile(lwirDir, '*.jpg'));
    if isempty(lwirFiles)
        I_lwir = [];
        I_vis  = [];
        ok     = false;
        return;
    end

    % sırala
    [~, order] = sort({lwirFiles.name});
    lwirFiles = lwirFiles(order);

    % index kontrol
    if idx+1 > length(lwirFiles)
        I_lwir = [];
        I_vis  = [];
        ok     = false;
        return;
    end

    % LWIR oku
    lwirPath = fullfile(lwirDir, lwirFiles(idx+1).name);
    I_lwir   = im2double(imread(lwirPath));

    % VIS (opsiyonel)
    I_vis = [];
    if exist(visDir, 'dir')
        visFiles = dir(fullfile(visDir, '*.jpg'));

        if ~isempty(visFiles) && idx+1 <= length(visFiles)
            [~, order] = sort({visFiles.name});
            visFiles = visFiles(order);

            visPath = fullfile(visDir, visFiles(idx+1).name);
            I_vis   = im2double(imread(visPath));
        end
    end

    ok = true;
end