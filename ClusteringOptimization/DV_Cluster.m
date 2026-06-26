function [assignment, medoids, history] = DV_Cluster(points, k, ~, startPoints, map, goalPoints, algoName)
%DV_CLUSTER 基于偏离量 (Deviation Value) 的多机器人任务分配
%   对每个机器人和目标点，计算"经过该目标的路径 / 直达路径"的比值 DV，
%   将每个目标分配给 DV 最小的机器人（路径干扰最小）。
%
%   [assignment, medoids, history] = DV_Cluster(points, k, ~, startPoints, map, goalPoints, algoName)
%
%   Inputs:
%     points      - K×2 目标点坐标 [row, col]
%     k           - 机器人数
%     ~           - 占位（接口兼容）
%     startPoints - k×2 各机器人起点 [row, col]
%     map         - Map 对象（栅格地图）
%     goalPoints  - k×2 各机器人终点 [row, col]
%     algoName    - 全局规划器名称（默认 'AStar_v1'）
%
%   Outputs:
%     assignment  - K×1 每个目标点的分配编号 (1..k)
%     medoids     - []（非 K-Medoids 算法，返回空）
%     history     - 结构体:
%       .DV         - k×K 偏离量矩阵
%       .baseCost   - 1×k 各机器人基线路径长度
%       .detourCost - k×K 各机器人经各目标的绕行路径长度
%       .algoName   - 使用的规划器名称

%% 输入校验
numTargets = size(points, 1);
numRobots = k;

if numTargets == 0
    assignment = [];
    medoids = [];
    history = struct('DV', [], 'baseCost', [], 'detourCost', [], 'algoName', algoName);
    return;
end

if nargin < 7 || isempty(algoName)
    algoName = 'AStar_v1';
end

if nargin < 6 || isempty(goalPoints)
    goalPoints = startPoints;  % 无终点信息时退化为起点=终点
end

%% Step 1: 计算各机器人基线路径长度 baseCost(r) = A*(start_r → goal_r)
%   所有路径均先经 SimplifyPath 拐角裁剪简化后再计算长度
occGrid = map.getOccupancyGrid();
n = map.mapSize;

baseCost = zeros(1, numRobots);
for r = 1:numRobots
    path = callPlanner(algoName, map, startPoints(r, :), goalPoints(r, :));
    path = SimplifyPath(path, occGrid, n);
    baseCost(r) = computePathLength(path);
end

% 防止除零（起点=终点时 baseCost=0，用 1 替代）
baseCost(baseCost == 0) = 1;

%% Step 2: 构建偏离量矩阵 DV(k×K)
%   DV(r,t) = [A*(start_r → t) + A*(t → goal_r)] / baseCost(r)
detourCost = zeros(numRobots, numTargets);

for r = 1:numRobots
    for t = 1:numTargets
        pathToTarget = callPlanner(algoName, map, startPoints(r, :), points(t, :));
        pathToTarget = SimplifyPath(pathToTarget, occGrid, n);
        pathToGoal = callPlanner(algoName, map, points(t, :), goalPoints(r, :));
        pathToGoal = SimplifyPath(pathToGoal, occGrid, n);
        detourCost(r, t) = computePathLength(pathToTarget) + computePathLength(pathToGoal);
    end
end

DV = detourCost ./ baseCost(:);  % k×K 矩阵，每行除以对应机器人的 baseCost

%% Step 3: 贪心分配 — 每个目标分配给 DV 最小的机器人
assignment = zeros(numTargets, 1);
for t = 1:numTargets
    [~, bestR] = min(DV(:, t));
    assignment(t) = bestR;
end

medoids = [];

%% 输出历史
history = struct();
history.DV = DV;
history.baseCost = baseCost;
history.detourCost = detourCost;
history.algoName = algoName;

end

%% =========================================================================
%  内部函数：调用全局规划器
%  =========================================================================
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

%% =========================================================================
%  内部函数：计算路径总长度
%  =========================================================================
function len = computePathLength(path)
    if size(path, 1) < 2
        len = 0;
        return;
    end
    diffs = diff(path, 1, 1);
    len = sum(sqrt(sum(diffs.^2, 2)));
end
