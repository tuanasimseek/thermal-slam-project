function [dx, dy, dtheta, conf] = dnn_odometry(odomModel, baseNet, I1, I2)
% DNN_ODOMETRY
% Siamese ResNet feature regresoru ile iki termal frame arasindaki hareketi
% tahmin eder.
%
% Cikti birimleri:
%   dx, dy   : piksel cinsinden goreli hareket
%   dtheta   : radyan cinsinden goreli aci
%   conf     : model RMSE ve tahmin buyuklugunden turetilmis proxy guven

if nargin < 4
    error('Kullanim: dnn_odometry(odomModel, baseNet, I1, I2)');
end

featureLayer = 'pool5';
if isfield(odomModel, 'featureLayer')
    featureLayer = odomModel.featureLayer;
end

f1 = extractFeature(I1, baseNet, featureLayer);
f2 = extractFeature(I2, baseNet, featureLayer);

x = single([f1, f2, f2 - f1, abs(f2 - f1)]);
xn = (x - single(odomModel.muX)) ./ single(odomModel.sdX);

yn = predict(odomModel.regressionNet, xn);
y  = yn .* single(odomModel.sdY) + single(odomModel.muY);

if isfield(odomModel, 'outputScale') && isfield(odomModel, 'outputBias')
    y = y .* single(odomModel.outputScale) + single(odomModel.outputBias);
end

dx     = double(y(1));
dy     = double(y(2));
dtheta = double(y(3));

% Pseudo-label araligindaki tek-adim hareket sinirlarini koru.
% Bu, nadir regresyon sicrama tahminlerinin pose graph'i bozmasini engeller.
dx = max(-3.0, min(3.0, dx));
dy = max(-3.0, min(3.0, dy));
dtheta = max(-0.05, min(0.05, dtheta));

if isfield(odomModel, 'rmseCalibrated') && ~isempty(odomModel.rmseCalibrated)
    rmseForConf = odomModel.rmseCalibrated;
elseif isfield(odomModel, 'rmse') && ~isempty(odomModel.rmse)
    rmseForConf = odomModel.rmse;
else
    rmseForConf = [];
end

if ~isempty(rmseForConf)
    motionMag = norm([dx, dy, dtheta]);
    errMag = norm(double(rmseForConf(:)));
    conf = exp(-errMag / (motionMag + errMag + eps));
else
    conf = 0.75;
end

conf = max(0.05, min(1.0, conf));
end

function feat = extractFeature(I, net, featureLayer)
I = my_preprocess(I);
if ndims(I) == 3
    I = rgb2gray(I);
end
I = imresize(I, [224 224]);
I = repmat(single(I), [1 1 3]);

featRaw = activations(net, I, featureLayer, 'OutputAs','rows');
feat = double(featRaw(:))';
n = norm(feat);
if n > 1e-8
    feat = feat / n;
end
end
