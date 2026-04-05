classdef PoseGraph
    properties
        nodes      % Nx2 [x y]
        edges      % Mx4 [i j dx dy]
        nodeCount
    end

    methods
        function obj = PoseGraph()
            obj.nodes = [];
            obj.edges = [];
            obj.nodeCount = 0;
        end

        function obj = addNode(obj, pose)
            obj.nodeCount = obj.nodeCount + 1;
            obj.nodes(obj.nodeCount,:) = pose;
        end

        function obj = addEdge(obj, i, j, dx, dy)
            obj.edges(end+1,:) = [i j dx dy];
        end
    end
end