% Tüm konumları bir grafikte saklar
% Düğüm = konum, kenar = "A'dan B'ye şu kadar hareket ettim" bilgisi


classdef PoseGraph
    properties
        nodes; edges; nodeCount
    end
    methods
        function obj = PoseGraph()
            obj.nodes     = [];
            obj.edges     = [];
            obj.nodeCount = 0;
        end
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