function [assignment, medoids, history] = DV_Cluster_v2(points, k, ~, startPoints, map, goalPoints, algoName, tspAlgo)
%DV_CLUSTER_V2 基于偏离量 (DV) 的多机器人任务分配 + 路径缓存 + 二次迁移优化
%   v2 在 DV_Cluster_v1 基础上引入全局路径缓存：
%     - DV 矩阵构建阶段计算的 A* 路径全部缓存到 containers.Map
%     - 二次迁移阶段 TSPsolver 复用缓存路径，避免重复计算
%     - 相同起点→终点对只需计算一次 A*
%
%   [assignment, medoids, history] = DV_Cluster_v2(points, k, ~, startPoints, map, goalPoints, algoName, tspAlgo)
%
%   Inputs:
%     points      - K×2 目标点坐标 [row, col]
%     k           - 机器人数
%     ~           - 占位（接口兼容）
%     startPoints - k×2 各机器人起点 [row, col]
%     map         - Map 对象（栅格地图）
%     goalPoints  - k×2 各机器人终点 [row, col]
%     algoName    - 全局规划器名称（默认 'AStar_v1'）
%     tspAlgo     - TSP 求解算法名称（默认 'TSP_ACO_v2'）
%
%   Outputs:
%     assignment  - K×1 每个目标点的分配编号 (1..k)
%     medoids     - []（非 K-Medoids 算法，返回空）
%     history     - 结构体:
%       .DV           - k×K 偏离量矩阵
%       .baseCost     - 1×k 各机器人基线路径长度
%       .detourCost   - k×K 各机器人经各目标的绕行路径长度
%       .algoName     - 使用的规划器名称
%       .nMigrations  - 二次迁移成功次数
%       .totalCostBefore - 初始分配总 TSP 代价
%       .totalCostAfter  - 迁移后总 TSP 代价
%       .cacheSize    - 路径缓存命中次数
%       .cacheMiss    - 路径缓存未命中次数

%% 输入校验
numTargets = size(points, 1);
numRobots = k;

if numTargets == 0
    assignment = [];
    medoids = [];
    history = struct('DV', [], 'baseCost', [], 'detourCost', [], 'algoName', algoName, ...
        'nMigrations', 0, 'totalCostBefore', 0, 'totalCostAfter', 0, ...
        'cacheSize', 0, 'cacheMiss', 0);
    return;
end

if nargin < 8 || isempty(tspAlgo)
    tspAlgo = 'TSP_ACO_v2';
end
if nargin < 7 || isempty(algoName)
    algoName = 'AStar_v1';
end
if nargin < 6 || isempty(goalPoints)
    goalPoints = startPoints;
end

%% 初始化全局路径缓存
pathCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
nCacheHit = 0;
nCacheMiss = 0;

occGrid = map.getOccupancyGrid();
n = map.mapSize;

%% Step 1: 计算各机器人基线路径长度 baseCost(r) = A*(start_r → goal_r)
baseCost = zeros(1, numRobots);
for r = 1:numRobots
    [path, nCacheHit, nCacheMiss] = cachedPlanner(algoName, map, startPoints(r, :), goalPoints(r, :), pathCache, nCacheHit, nCacheMiss);
    path = SimplifyPath(path, occGrid, n);
    baseCost(r) = computePathLength(path);
end

% 防止除零（起点=终点时 baseCost=0，用 1 替代）
baseCost(baseCost == 0) = 1;

%% Step 2: 构建偏离量矩阵 DV(k×K) — 所有路径缓存
detourCost = zeros(numRobots, numTargets);

for r = 1:numRobots
    for t = 1:numTargets
        [pathToTarget, nCacheHit, nCacheMiss] = cachedPlanner(algoName, map, startPoints(r, :), points(t, :), pathCache, nCacheHit, nCacheMiss);
        pathToTarget = SimplifyPath(pathToTarget, occGrid, n);
        [pathToGoal, nCacheHit, nCacheMiss] = cachedPlanner(algoName, map, points(t, :), goalPoints(r, :), pathCache, nCacheHit, nCacheMiss);
        pathToGoal = SimplifyPath(pathToGoal, occGrid, n);
        detourCost(r, t) = computePathLength(pathToTarget) + computePathLength(pathToGoal);
    end
end

DV = detourCost ./ baseCost(:);  % k×K 矩阵

%% Step 3: 贪心初始分配 — 每个目标分配给 DV 最小的机器人
assignment = zeros(numTargets, 1);
for t = 1:numTargets
    [~, bestR] = min(DV(:, t));
    assignment(t) = bestR;
end

%% Step 4: 计算初始 TSP 代价并缓存每台机器人的代价
robotTSPCost = zeros(1, numRobots);
for r = 1:numRobots
    robotTSPCost(r) = computeSingleTSPCost(r, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n, pathCache);
end
totalCostBefore = sum(robotTSPCost);

%% Step 5: 二次迁移优化 — 基于 DV 相似度 + TSP 代价缓存
%   costBefore 直接从 robotTSPCost 读取（0 次 TSPsolver），
%   只对 costAfter 调用 TSPsolver（2 次而非 4 次），
%   迁移成功后更新 robotTSPCost。
nMigrations = 0;

for t = 1:numTargets
    currentR = assignment(t);
    currentDV = DV(currentR, t);

    % 按 DV 值排序（从小到大）
    [sortedDV, sortedIdx] = sort(DV(:, t), 'ascend');

    % 遍历 DV 值接近的其他机器人（跳过自己）
    for si = 2:numRobots
        otherR = sortedIdx(si);
        otherDV = sortedDV(si);

        % 计算相似度：minDV / otherDV
        similarity = currentDV / otherDV;

        if similarity < 0.95
            % 相似度不够，后续更不满足，跳出
            break;
        end

        % DV 相似度 > 0.95，尝试迁移
        % costBefore 直接从缓存读取，无需重复计算 TSP
        costBeforeR1 = robotTSPCost(currentR);
        costBeforeR2 = robotTSPCost(otherR);

        % 模拟迁移，只计算 costAfter（2 次 TSPsolver）
        trialAssign = assignment;
        trialAssign(t) = otherR;

        costAfterR1 = computeSingleTSPCost(currentR, trialAssign, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n, pathCache);
        costAfterR2 = computeSingleTSPCost(otherR, trialAssign, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n, pathCache);

        % 迁移后总代价减少则采纳
        if (costAfterR1 + costAfterR2) < (costBeforeR1 + costBeforeR2)
            assignment(t) = otherR;
            % 更新缓存：两个机器人代价均已变化
            robotTSPCost(currentR) = costAfterR1;
            robotTSPCost(otherR) = costAfterR2;
            nMigrations = nMigrations + 1;
            break;  % 该目标已迁移，不再尝试其他机器人
        end
    end
end

%% Step 6: 迁移后总 TSP 代价 — 直接从缓存求和，无需重算
totalCostAfter = sum(robotTSPCost);

medoids = [];

%% 输出历史
history = struct();
history.DV = DV;
history.baseCost = baseCost;
history.detourCost = detourCost;
history.algoName = algoName;
history.tspAlgo = tspAlgo;
history.nMigrations = nMigrations;
history.totalCostBefore = totalCostBefore;
history.totalCostAfter = totalCostAfter;
history.cacheSize = length(pathCache.keys());
history.cacheHit = nCacheHit;
history.cacheMiss = nCacheMiss;

end

%% =========================================================================
%  内部函数：带缓存的全局规划器调用
%  =========================================================================
function [path, nHit, nMiss] = cachedPlanner(algoName, map, startGrid, goalGrid, pathCache, nHit, nMiss)
    cacheKey = sprintf('%d,%d_%d,%d', startGrid(1), startGrid(2), goalGrid(1), goalGrid(2));

    if pathCache.isKey(cacheKey)
        path = pathCache(cacheKey);
        nHit = nHit + 1;
    else
        path = callPlanner(algoName, map, startGrid, goalGrid);
        % 只缓存有效路径（非空），避免缓存不可达结果
        if ~isempty(path)
            pathCache(cacheKey) = path; %#ok<NASGU> handle class, modifies caller's map
        end
        nMiss = nMiss + 1;
    end
end

%% =========================================================================
%  内部函数：计算单台机器人的 TSP 代价（传入外部缓存）
%  =========================================================================
function cost = computeSingleTSPCost(r, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n, pathCache)
    myTargets = points(assignment == r, :);
    if isempty(myTargets)
        % 无分配目标：直接起点→终点
        [path, ~, ~] = cachedPlanner(algoName, map, startPoints(r, :), goalPoints(r, :), pathCache, 0, 0);
        path = SimplifyPath(path, occGrid, n);
        cost = computePathLength(path);
    else
        % 将路径缓存传入 TSPsolver，复用已计算的 A* 路径
        [~, segPaths, ~] = TSPsolver(startPoints(r, :), myTargets, goalPoints(r, :), map, algoName, tspAlgo, false, [], [], pathCache);
        cost = 0;
        for s = 1:length(segPaths)
            seg = SimplifyPath(segPaths{s}, occGrid, n);
            cost = cost + computePathLength(seg);
        end
    end
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
