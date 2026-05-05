% RUN_ALL_ADVANCED
% results/ klasoründeki tüm set ve videolar icin
% plot_advanced calistirip gelismis gorsel uretir.
%
% Kullanim:
%   run_all_advanced          % sadece results/
%   run_all_advanced('cnn')   % results_cnn/
%   run_all_advanced('ssd')   % results_ssd/

function run_all_advanced(mode)

if nargin < 1, mode = 'default'; end

switch lower(mode)
    case 'cnn',     resultsDir = 'results_cnn';
    case 'ssd',     resultsDir = 'results_ssd';
    otherwise,      resultsDir = 'results';
end

addpath(genpath('src'));

fprintf('============================================\n');
fprintf('TOPLU GELISMIS GORSEL URETIMI\n');
fprintf('Kaynak: %s/\n', resultsDir);
fprintf('============================================\n\n');

% Tum traj dosyalarini bul
trajFiles = dir(fullfile(resultsDir, '*_traj.mat'));

if isempty(trajFiles)
    fprintf('Hic traj.mat bulunamadi: %s/\n', resultsDir);
    return;
end

okCount   = 0;
skipCount = 0;
errCount  = 0;

for fi = 1:length(trajFiles)
    tName = trajFiles(fi).name;

    % Isim: set00_V000_traj.mat → prefix: set00_V000
    prefix    = strrep(tName, '_traj.mat', '');
    trajPath  = fullfile(resultsDir, tName);
    graphPath = fullfile(resultsDir, [prefix '_graph.mat']);

    % Metrik dosyasi — varsa kullan (video bazli veya set bazli)
    setPrefix    = regexp(prefix, '^set\d+', 'match', 'once');
    metricPath   = fullfile(resultsDir, [prefix '_metrics.mat']);
    metricSetPath= fullfile(resultsDir, [setPrefix '_metrics.mat']);

    if ~exist(graphPath, 'file')
        fprintf('SKIP (graph yok): %s\n', prefix);
        skipCount = skipCount + 1;
        continue;
    end

    % Metrics var mi?
    if exist(metricPath, 'file')
        mPath = metricPath;
    elseif exist(metricSetPath, 'file')
        mPath = metricSetPath;
    else
        mPath = '';
    end

    fprintf('Isleniyor: %s\n', prefix);
    try
        finalDir = fullfile(pwd, 'results_final');

        if isempty(mPath)
            plot_advanced(trajPath, graphPath, [], finalDir);
        else
            plot_advanced(trajPath, graphPath, mPath, finalDir);
        end
        close all;   % bellek icin figürleri kapat
        okCount = okCount + 1;
    catch ME
        fprintf('HATA (%s): %s\n', prefix, ME.message);
        errCount = errCount + 1;
        close all;
    end
end

fprintf('\n============================================\n');
fprintf('TAMAMLANDI\n');
fprintf('  Basarili : %d\n', okCount);
fprintf('  Atlandi  : %d\n', skipCount);
fprintf('  Hata     : %d\n', errCount);
fprintf('  Gorseller: results_final/figures/\n');
fprintf('============================================\n');
end