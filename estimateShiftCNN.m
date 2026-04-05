function [dx, dy, bestScore] = estimateShiftCNN(I, tmpl, x0, y0, R, net)

    [th, tw] = size(tmpl);
    [H, W]   = size(I);

    featTemplate = extractCNNFeature(tmpl, net);

    bestScore = inf;
    dx = 0;
    dy = 0;

    for yy = -R:R
        for xx = -R:R
            xs = x0 + xx;
            ys = y0 + yy;

            if xs < 1 || ys < 1 || (xs+tw-1) > W || (ys+th-1) > H
                continue;
            end

            patch = I(ys:ys+th-1, xs:xs+tw-1);
            featPatch = extractCNNFeature(patch, net);

            score = 1 - dot(featTemplate, featPatch) / ...
                (norm(featTemplate) * norm(featPatch) + eps);

            if score < bestScore
                bestScore = score;
                dx = xx;
                dy = yy;
            end
        end
    end
end