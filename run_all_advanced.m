% RUN_ALL_ADVANCED
% results_cnn / results_ssd içindeki tüm traj dosyaları için
% plot_advanced çalıştırır ve çıktıları results_final içine düzenli kaydeder.
%
% Kullanım:
%   run_all_advanced('cnn')
%   run_all_advanced('ssd')

function run_all_advanced(mode)

if nargin < 1
    mode = 'default';
end

switch lower(mode)
    case 'cnn'
        resultsDir  = fullfile(pwd, 'results_cnn');
        finalMatDir = fullfile(pwd, 'results_final', 'mat', 'cnn');
        methodName  = 'cnn';
    case 'ssd'
        resultsDir  = fullfile(pwd, 'results_ssd');
        finalMatDir = fullfile(pwd, 'results_final', 'mat', 'ssd');
        methodName  = 'ssd';
    case 'dnn'
        resultsDir  = fullfile(pwd, 'results_final', 'mat', 'dnn');
        finalMatDir = resultsDir;
        methodName  = 'dnn';
    otherwise
        resultsDir  = fullfile(pwd, 'results');
        finalMatDir = '';
        methodName  = 'default';
end

addpath(genpath('src'));

finalDir       = fullfile(pwd, 'results_final');
advancedFigDir = fullfile(finalDir, 'figures', 'advanced', methodName);

if ~exist(advancedFigDir, 'dir')
    mkdir(advancedFigDir);
end

fprintf('============================================\n');
fprintf('TOPLU GELISMIS GORSEL URETIMI\n');
fprintf('Kaynak : %s\n', resultsDir);
fprintf('Yontem : %s\n', upper(methodName));
fprintf('Cikti  : %s\n', advancedFigDir);
fprintf('============================================\n\n');

trajFiles = dir(fullfile(resultsDir, '*_traj.mat'));

if isempty(trajFiles)
    fprintf('Hic traj.mat bulunamadi: %s\n', resultsDir);
    return;
end

okCount   = 0;
skipCount = 0;
errCount  = 0;

for fi = 1:length(trajFiles)

    tName  = trajFiles(fi).name;
    prefix = strrep(tName, '_traj.mat', '');

    trajPath  = fullfile(resultsDir, tName);
    graphPath = fullfile(resultsDir, [prefix '_graph.mat']);

    if ~exist(graphPath, 'file')
        fprintf('SKIP graph yok: %s\n', prefix);
        skipCount = skipCount + 1;
        continue;
    end

    % Eski metrics.mat kullanılmıyor çünkü meanConfidence eski dosyada 1.0000 kalabiliyor
    mPath = '';

    % DÜZELTME: conf.mat ara — results_final/mat/cnn/ içinde
    % Dosya adı: set00_V000_cnn_conf.mat
    confPath = '';
    if ~isempty(finalMatDir)
        % prefix örn: set00_V000 → set00_V000_cnn_conf.mat
        confCandidate = fullfile(finalMatDir, [prefix '_' methodName '_conf.mat']);
        altConfCandidate = fullfile(finalMatDir, [prefix '_conf.mat']);
        if exist(confCandidate, 'file')
            confPath = confCandidate;
        elseif exist(altConfCandidate, 'file')
            confPath = altConfCandidate;
        end
    end

    fprintf('Isleniyor: %s\n', prefix);

    try
        plot_advanced(trajPath, graphPath, mPath, advancedFigDir, methodName, confPath);
        close all;
        okCount = okCount + 1;
    catch ME
        fprintf('HATA %s: %s\n', prefix, ME.message);
        errCount = errCount + 1;
        close all;
    end

end

fprintf('\n============================================\n');
fprintf('TAMAMLANDI\n');
fprintf('Basarili : %d\n', okCount);
fprintf('Atlandi  : %d\n', skipCount);
fprintf('Hata     : %d\n', errCount);
fprintf('Gorseller: %s\n', advancedFigDir);
fprintf('============================================\n');

end
