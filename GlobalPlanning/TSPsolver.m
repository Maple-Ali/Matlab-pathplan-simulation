function [orderedPoints, segPaths, totalCost] = TSPsolver(startPoint, targets, goalPoint, map, algoName, tspAlgo)
%TSPSOLVER TSP 多目标排序求解器
%   [orderedPoints, segPaths, totalCost] = TSPsolver(startPoint, targets, goalPoint, map, algoName, tspAlgo)
%   startPoint: [row, col] 起点栅格
%   targets: K×2 目标点 [row, col]
%   goalPoint: [row, col] 终点栅格
%   map: Map 对象
%   algoName: 'AStar' | 'Dijkstra' | 'RRT' 等全局规划器
%   tspAlgo: 'TSP_GA' | 'TSP_Permutation' 等 TSP 求解算法（可选，默认 'TSP_GA'）
%   输出:
%   orderedPoints: 有序访问点（含起点终点）
%   segPaths: 每段之间的全局路径 cell 数组
%   totalCost: 总路径成本

if nargin < 6 || isempty(tspAlgo)
    tspAlgo = 'TSP_GA';
end

% 所有点：起点 + 目标点集 + 终点
allPoints = [startPoint; targets; goalPoint];
nPts = size(allPoints, 1);   % 第1个=起点, 最后1个=终点

% --- 计算成本矩阵（所有点对之间的最短路径长度）---
costMatrix = inf(nPts, nPts);
pathCache = cell(nPts, nPts);

for i = 1:nPts
    for j = 1:nPts
        if i == j
            costMatrix(i, j) = 0;
            continue;
        end
        % 计算 i→j 的最短路径
        p = callPlanner(algoName, map, allPoints(i, :), allPoints(j, :));
        if ~isempty(p)
            costMatrix(i, j) = pathLength(p);
            pathCache{i, j} = p;
        end
    end
end

% 检查可达性
if any(isinf(costMatrix(1, 2:end-1)))
    orderedPoints = allPoints;
    segPaths = {};
    totalCost = inf;
    warning('存在不可达的目标点');
    return;
end

% --- TSP 求解：委托给选定的 TSP 算法 ---
nMid = nPts - 2;

if nMid == 0
    % 没有中间目标点，起点直达终点
    bestOrder = [1, nPts];
    totalCost = costMatrix(1, nPts);
else
    tspFunc = str2func(tspAlgo);
    [bestOrder, totalCost] = tspFunc(costMatrix, nPts);
end

% --- 组装输出 ---
orderedPoints = allPoints(bestOrder, :);
segPaths = cell(length(bestOrder) - 1, 1);
for k = 1:(length(bestOrder) - 1)
    i = bestOrder(k);
    j = bestOrder(k + 1);
    if ~isempty(pathCache{i, j})
        segPaths{k} = pathCache{i, j};
    else
        segPaths{k} = callPlanner(algoName, map, allPoints(i, :), allPoints(j, :));
    end
end
end

% -------------------------------------------------------------------------
function path = callPlanner(algoName, map, startGrid, goalGrid)
    switch algoName
        case 'AStar'
            path = AStar(map, startGrid, goalGrid, 0);
        case 'AStar_v0'
            path = AStar_v0(map, startGrid, goalGrid, 0);
        case 'AStar_v1'
            path = AStar_v1(map, startGrid, goalGrid, 0);
        case 'AStar_v2'
            path = AStar_v2(map, startGrid, goalGrid, 0);
        case 'AStar_v3'
            path = AStar_v3(map, startGrid, goalGrid, 0);
        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0);
        case 'Dijkstra_v1'
            path = Dijkstra_v1(map, startGrid, goalGrid, 0);
        case 'RRT'
            path = RRT(map, startGrid, goalGrid, 0);
        otherwise
            path = AStar(map, startGrid, goalGrid, 0);
    end
end

function len = pathLength(path)
    if isempty(path)
        len = inf;
        return;
    end
    len = 0;
    for i = 2:size(path, 1)
        dr = path(i, 1) - path(i - 1, 1);
        dc = path(i, 2) - path(i - 1, 2);
        if dr ~= 0 && dc ~= 0
            len = len + sqrt(2);
        else
            len = len + 1;
        end
    end
end
