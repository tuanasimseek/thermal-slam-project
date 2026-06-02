function [poses, gtPath, info] = load_ground_truth_pose(setName, videoName, dataRoot)
% LOAD_GROUND_TRUTH_POSE
% Finds and loads an optional ground-truth pose file for a sequence.
%
% Output:
%   poses  : Nx3 [x, y, theta] if a supported file is found, otherwise []
%   gtPath : loaded file path, otherwise ''
%   info   : struct with status/format details
%
% Supported simple formats:
%   - Nx3: [x, y, theta]
%   - Nx2: [x, y]
%   - Nx4 or wider: [frame, x, y, theta, ...]
%   - Nx12 KITTI-style row-major 3x4 pose matrix

if nargin < 3 || isempty(dataRoot)
    dataRoot = fullfile(pwd, 'data');
end

poses = [];
gtPath = '';
info = struct('found', false, 'format', '', 'message', '');

seqDir = fullfile(dataRoot, setName, videoName);
candidates = {
    fullfile(seqDir, 'poses.txt')
    fullfile(seqDir, 'pose.txt')
    fullfile(seqDir, 'groundtruth.txt')
    fullfile(seqDir, 'ground_truth.txt')
    fullfile(seqDir, 'gt.txt')
    fullfile(seqDir, 'poses.csv')
    fullfile(seqDir, 'groundtruth.csv')
    fullfile(dataRoot, setName, [videoName '_poses.txt'])
    fullfile(dataRoot, setName, [videoName '_groundtruth.txt'])
    fullfile(dataRoot, setName, [videoName '_gt.txt'])
};

for i = 1:numel(candidates)
    p = candidates{i};
    if ~isfile(p)
        continue;
    end

    try
        raw = readmatrix(p);
    catch
        info.message = sprintf('Ground truth dosyasi okunamadi: %s', p);
        continue;
    end

    raw = raw(all(isfinite(raw), 2), :);
    [parsed, fmt] = parsePoseMatrix(raw);
    if isempty(parsed)
        info.message = sprintf('Desteklenmeyen ground truth formati: %s', p);
        continue;
    end

    poses = parsed;
    gtPath = p;
    info.found = true;
    info.format = fmt;
    info.message = sprintf('Ground truth yuklendi: %s', p);
    return;
end

info.message = sprintf('Ground truth bulunamadi: %s/%s', setName, videoName);
end

function [poses, fmt] = parsePoseMatrix(raw)
poses = [];
fmt = '';

if isempty(raw) || size(raw, 1) < 2
    return;
end

cols = size(raw, 2);

if cols == 2
    poses = [raw(:,1), raw(:,2), zeros(size(raw,1), 1)];
    fmt = 'xy';
    return;
end

if cols == 3
    poses = raw(:, 1:3);
    fmt = 'x_y_theta';
    return;
end

if cols == 12
    % KITTI-style 3x4 transform rows:
    % [r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz]
    x = raw(:, 4);
    y = raw(:, 12);
    theta = atan2(raw(:, 5), raw(:, 1));
    poses = [x, y, theta];
    fmt = 'kitti_3x4';
    return;
end

if cols >= 4
    % Common log format: [frame, x, y, theta, ...]
    poses = raw(:, 2:4);
    fmt = 'frame_x_y_theta';
    return;
end
end
