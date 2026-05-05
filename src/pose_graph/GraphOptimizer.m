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
            edges = obj.graph.edges;

            % edges ister struct ister numeric olsun, normalize et
            if isstruct(edges)
                E = zeros(numel(edges), 4);
                for k = 1:numel(edges)
                    E(k,1)=edges(k).from; E(k,2)=edges(k).to;
                    E(k,3)=edges(k).transform(1); E(k,4)=edges(k).transform(2);
                end
            elseif size(edges,2) >= 4
                E = edges(:,1:4);
            else
                E = zeros(0,4);
            end

            optimizedNodes = nodes;

            for it = 1:iterations
                for e = 1:size(E,1)
                    i = E(e,1);
                    j = E(e,2);
                    dx = E(e,3);
                    dy = E(e,4);

                    pred = optimizedNodes(j,:) - optimizedNodes(i,:);
                    err  = [dx, dy] - pred;

                    % ilk node sabit kalsın
                    if i ~= 1
                        optimizedNodes(i,:) = optimizedNodes(i,:) - alpha * 0.5 * err;
                    end
                    if j ~= 1
                        optimizedNodes(j,:) = optimizedNodes(j,:) + alpha * 0.5 * err;
                    end
                end
            end
        end
    end
end