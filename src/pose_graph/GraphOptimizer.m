% Birikmiş hatayı dağıtır
% Loop bulununca "aslında buraya 50m değil 48m yürüdüm" diye tüm grafiği düzeltir


classdef GraphOptimizer
    properties
        graph
    end

    methods
        function obj = GraphOptimizer(pg)
            obj.graph = pg;
        end

        function optimizedNodes = optimize(obj, iterations, alpha)
            if nargin < 2
                iterations = 200;
            end
            if nargin < 3
                alpha = 0.2;
            end

            nodes = obj.graph.nodes;
            if size(nodes,2) == 2
                nodes(:,3) = 0;
            end

            edges = obj.graph.edges;
            E = normalizeEdges(edges);

            optimizedNodes = nodes;

            for it = 1:iterations
                for e = 1:size(E,1)
                    i = E(e,1);
                    j = E(e,2);
                    dx = E(e,3);
                    dy = E(e,4);
                    dtheta = E(e,5);
                    weight = E(e,6);
                    edgeType = E(e,7);

                    if i < 1 || j < 1 || ...
                            i > size(optimizedNodes,1) || j > size(optimizedNodes,1)
                        continue;
                    end

                    pred = optimizedNodes(j,:) - optimizedNodes(i,:);
                    pred(3) = wrapAngle(pred(3));

                    err  = [dx, dy, dtheta] - pred;
                    err(3) = wrapAngle(err(3));

                    % Loop closure edge'leri sahte pozitiflere hassas oldugu
                    % icin odometry edge'lerinden daha yumusak uygulanir.
                    if edgeType == 1
                        weight = min(max(weight, 0.2), 0.8);
                    end

                    step = alpha * min(max(weight, 0.05), 1.5);

                    % ilk node sabit kalsın
                    if i ~= 1
                        optimizedNodes(i,:) = optimizedNodes(i,:) - step * 0.5 * err;
                        optimizedNodes(i,3) = wrapAngle(optimizedNodes(i,3));
                    end
                    if j ~= 1
                        optimizedNodes(j,:) = optimizedNodes(j,:) + step * 0.5 * err;
                        optimizedNodes(j,3) = wrapAngle(optimizedNodes(j,3));
                    end
                end
            end
        end
    end
end

function E = normalizeEdges(edges)
% Standart format: [from, to, dx, dy, dtheta, weight, type]
if isempty(edges)
    E = zeros(0,7);
    return;
end

if isstruct(edges)
    E = zeros(numel(edges), 7);
    for k = 1:numel(edges)
        E(k,1) = edges(k).from;
        E(k,2) = edges(k).to;
        tr = edges(k).transform;
        E(k,3) = tr(1);
        E(k,4) = tr(2);
        if numel(tr) >= 3
            E(k,5) = tr(3);
        end
        E(k,6) = 1;
        E(k,7) = 0;
    end
    return;
end

if size(edges,2) >= 7
    E = edges(:,1:7);
elseif size(edges,2) == 6
    E = [edges(:,1:6), zeros(size(edges,1),1)];
elseif size(edges,2) == 5
    % Eski kayit formati: [i, j, dx, dy, weight]
    E = [edges(:,1:4), zeros(size(edges,1),1), edges(:,5), zeros(size(edges,1),1)];
elseif size(edges,2) == 4
    E = [edges(:,1:4), zeros(size(edges,1),1), ones(size(edges,1),1), zeros(size(edges,1),1)];
else
    E = zeros(0,7);
end
end

function a = wrapAngle(a)
a = mod(a + pi, 2*pi) - pi;
end
