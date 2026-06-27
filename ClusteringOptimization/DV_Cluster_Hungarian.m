function [assignment, medoids, history] = DV_Cluster_Hungarian(points, k, ~, startPoints, map, goalPoints, algoName)
%DV_CLUSTER_HUNGARIAN 基于偏离量 (DV) + 匈牙利算法的多机器人任务分配
%   与 DV_Cluster 相同的 DV 矩阵构建，但分配使用匈牙利算法（matchpairs）
%   求解全局最优匹配，使所有目标的总偏离量最小。
%
%   [assignment, medoids, history] = DV_Cluster_Hungarian(points, k, ~, startPoints, map, goalPoints, algoName)
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
    goalPoints = startPoints;
end

%% Step 1: 计算各机器人基线路径长度 baseCost(r) = A*(start_r → goal_r)
occGrid = map.getOccupancyGrid();
n = map.mapSize;

baseCost = zeros(1, numRobots);
for r = 1:numRobots
    path = callPlanner(algoName, map, startPoints(r, :), goalPoints(r, :));
    path = SimplifyPath(path, occGrid, n);
    baseCost(r) = computePathLength(path);
end

baseCost(baseCost == 0) = 1;

%% Step 2: 构建偏离量矩阵 DV(k×K)
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

DV = detourCost ./ baseCost(:);

%% Step 3: 匈牙利算法全局最优分配
%   matchpairs 只做一对一匹配，K>N 时需扩展为 K×K 方阵
%   每个机器人复制 ceil(K/N) 次，列 j 对应机器人 mod(j-1,N)+1
costMatrix = DV';  % 转置为 K×N
expandedCost = zeros(numTargets, numTargets);
colToRobot = zeros(1, numTargets);
for j = 1:numTargets
    r = mod(j - 1, numRobots) + 1;
    expandedCost(:, j) = costMatrix(:, r);
    colToRobot(j) = r;
end

matches = matchpairs(expandedCost, 1e6);

assignment = zeros(numTargets, 1);
for m = 1:size(matches, 1)
    assignment(matches(m, 1)) = colToRobot(matches(m, 2));
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
