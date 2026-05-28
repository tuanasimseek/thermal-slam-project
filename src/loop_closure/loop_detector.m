% "Buraya daha once gelmistim" kararini verir.
% Benzerlik tek basina yeterli degildir; en iyi eslesme hem yuksek skorlu
% olmali hem de ikinci en iyi eslesmeden belirgin ayrilmalidir.


function [loopIdx, simScore, loopInfo] = loop_detector(featDB, featCurr, minInterval, threshold, maxLookback, minMargin)

    if nargin < 4 || isempty(threshold)
        threshold = 0.9995;
    end
    if nargin < 5 || isempty(maxLookback)
        maxLookback = 300;
    end
    if nargin < 6 || isempty(minMargin)
        minMargin = 1e-4;
    end

    loopIdx  = -1;
    simScore = 0;
    loopInfo = struct('bestSim', 0, 'secondSim', 0, 'margin', 0, ...
        'threshold', threshold, 'minMargin', minMargin);

    N = size(featDB, 1);

    % en az minInterval kadar geçmiş olmalı
    if N <= minInterval + 1
        return;
    end

    searchEnd   = N - minInterval;
    searchStart = max(1, searchEnd - maxLookback + 1);

    if searchStart > searchEnd
        return;
    end

    searchDB = featDB(searchStart:searchEnd, :);

    similarities = searchDB * featCurr';
    [sortedSim, sortedIdx] = sort(similarities, 'descend');

    bestSim = sortedSim(1);
    localIdx = sortedIdx(1);

    if numel(sortedSim) >= 2
        secondSim = sortedSim(2);
    else
        secondSim = 0;
    end

    margin = bestSim - secondSim;

    loopInfo.bestSim = bestSim;
    loopInfo.secondSim = secondSim;
    loopInfo.margin = margin;

    if bestSim >= threshold && margin >= minMargin
        loopIdx  = searchStart + localIdx - 1;
        simScore = bestSim;
    end
end
