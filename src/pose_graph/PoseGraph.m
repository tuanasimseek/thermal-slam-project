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
            if numel(pose) == 2
                pose = [pose(:)' 0];
            else
                pose = pose(:)';
            end
            obj.nodeCount = obj.nodeCount + 1;
            obj.nodes(obj.nodeCount, :) = pose;
        end
        function obj = addEdge(obj, i, j, varargin)
            % Standart edge formati:
            % [from, to, dx, dy, dtheta, weight, type]
            % type: 0 = odometry, 1 = loop closure
            [dx, dy, dtheta, weight, edgeType] = parseEdgeArgs(varargin{:});
            obj.edges(end+1, :) = [i, j, dx, dy, dtheta, weight, edgeType];
        end
    end
end

function [dx, dy, dtheta, weight, edgeType] = parseEdgeArgs(varargin)
dx = 0;
dy = 0;
dtheta = 0;
weight = 1.0;
edgeType = 0;

if isempty(varargin)
    return;
end

if numel(varargin) == 1
    delta = varargin{1};
    dx = delta(1);
    dy = delta(2);
    if numel(delta) >= 3
        dtheta = delta(3);
    end
    return;
end

if numel(varargin) == 2 && numel(varargin{1}) >= 2
    delta = varargin{1};
    dx = delta(1);
    dy = delta(2);
    if numel(delta) >= 3
        dtheta = delta(3);
    end
    weight = varargin{2};
    return;
end

if numel(varargin) >= 2
    dx = varargin{1};
    dy = varargin{2};
end

if numel(varargin) == 3
    % Eski kullanim: addEdge(i,j,dx,dy,weight)
    weight = varargin{3};
elseif numel(varargin) == 4
    % Yeni kullanim: addEdge(i,j,dx,dy,dtheta,weight)
    dtheta = varargin{3};
    weight = varargin{4};
elseif numel(varargin) >= 5
    dtheta = varargin{3};
    weight = varargin{4};
    edgeType = varargin{5};
end
end
