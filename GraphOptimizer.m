classdef GraphOptimizer
    methods(Static)
        function optimizedNodes = optimize(graph, iterations, alpha)
            if nargin < 2
                iterations = 50;
            end
            if nargin < 3
                alpha = 0.1;
            end

            % Başlangıç node konumları
            optimizedNodes = graph.nodes;

            % Edge ve node'ları local değişkene al
            edges = graph.edges;
            nodes = graph.nodes;

            % ===== GELİŞMİŞ LOOP CLOSURE =====
            for i = 1:size(nodes,1)
                for j = i+10:size(nodes,1)   % daha uzak node'ları karşılaştır

                    dist = norm(nodes(i,:) - nodes(j,:));

                    % hem yakın olsun hem de tamamen aynı nokta olmasın
                    if dist < 0.5 && dist > 0.05

                        dx = nodes(j,1) - nodes(i,1);
                        dy = nodes(j,2) - nodes(i,2);

                        % ağırlıklı edge (daha güvenli)
                        weight = 0.7;

                        edges(end+1,:) = [i, j, dx*weight, dy*weight];
                    end
                end
            end

            % ===== OPTİMİZASYON =====
            % İlk node sabit tutulur
            for it = 1:iterations
                for e = 1:size(edges,1)
                    i = edges(e,1);
                    j = edges(e,2);
                    dx = edges(e,3);
                    dy = edges(e,4);

                    pred = optimizedNodes(j,:) - optimizedNodes(i,:);
                    err = [dx dy] - pred;

                    if i ~= 1
                        optimizedNodes(i,:) = optimizedNodes(i,:) - alpha * 0.3 * err;
                    end
                    if j ~= 1
                        optimizedNodes(j,:) = optimizedNodes(j,:) + alpha * 0.3 * err;
                    end
                end
            end
        end
    end
end