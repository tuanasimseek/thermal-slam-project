% Aynı şeyi klasik yöntemle yapar
% Piksel piksel karşılaştır, en az fark olan kayma = hareket. CNN yok.


function [dx, dy, bestScore] = estimateShiftSSD(I, tmpl, x0, y0, R)
    [th, tw] = size(tmpl);
    [H, W]   = size(I);
    bestScore = inf; dx = 0; dy = 0;
    for yy = -R:R
        for xx = -R:R
            xs = x0+xx; ys = y0+yy;
            if xs<1||ys<1||(xs+tw-1)>W||(ys+th-1)>H, continue; end
            patch = I(ys:ys+th-1, xs:xs+tw-1);
            score = sum((patch(:)-tmpl(:)).^2) / (th*tw);
            if score < bestScore
                bestScore = score; dx = xx; dy = yy;
            end
        end
    end
end