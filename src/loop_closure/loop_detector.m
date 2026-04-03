function [loopIdx, simScore] = loop_detector(featDB, featCurr, minInterval)

    LOOP_THRESHOLD = 0.93;   % yapay testte biraz sıkı
    MAX_LOOKBACK   = 300;    % geçmişte en fazla 200 kayıt tara

    loopIdx  = -1;
    simScore = 0;

    N = size(featDB, 1);

    % en az minInterval kadar geçmiş olmalı
    if N <= minInterval + 1
        return;
    end

    searchEnd   = N - minInterval;
    searchStart = max(1, searchEnd - MAX_LOOKBACK + 1);

    if searchStart > searchEnd
        return;
    end

    searchDB = featDB(searchStart:searchEnd, :);

    similarities = searchDB * featCurr';
    [bestSim, localIdx] = max(similarities);

    if bestSim >= LOOP_THRESHOLD
        loopIdx  = searchStart + localIdx - 1;
        simScore = bestSim;
    end
end