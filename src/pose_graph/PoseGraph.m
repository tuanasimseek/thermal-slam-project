% Tüm konumları bir grafikte saklar
%robotun geçtiği yerleri ve hareket ilişkilerini saklamak
% Düğüm = konum, kenar = "A'dan B'ye şu kadar hareket ettim" bilgisi


classdef PoseGraph
    properties
        nodes; edges; nodeCount
    end
    methods
        function obj = PoseGraph()
            obj.nodes     = []; %Her node: bir pozisyon, Örneğin: Node1 = [0 0] Node2 = [1 0.3] Node3 = [2 0.8] Robotun geçmiş konumları.
            obj.edges     = []; %iki node arasındaki hareket bilgisi, Node1 → Node2, dx = 1 ,dy = 0.3
            obj.nodeCount = 0;
        end
        %Robot ilerledikçe: yeni pozisyon → node, hareket bilgisi → edge  olarak ekleniyor.

        function obj = addNode(obj, pose) 
            obj.nodeCount = obj.nodeCount + 1;
            obj.nodes(obj.nodeCount, :) = pose;
        end
        function obj = addEdge(obj, i, j, dx, dy, weight)
            if nargin < 6, weight = 1.0; end
            obj.edges(end+1, :) = [i, j, dx, dy, weight];
        end
    end
end