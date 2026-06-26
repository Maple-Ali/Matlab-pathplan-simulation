function [assignment, medoids, history] = DV_Cluster_v1(points, k, ~, startPoints, map, goalPoints, algoName, tspAlgo)
%DV_CLUSTER_V1 基于偏离量 (DV) 的多机器人任务分配 + 相似度二次迁移优化
%   v1 在 DV_Cluster 基础上增加二次检测：
%     当某目标的最小 DV 与其他机器人的 DV 相似度 > 0.9 时，
%     尝试迁移并用 TSP 实际代价验证，总代价减少则采纳。
%
%   [assignment, medoids, history] = DV_Cluster_v1(points, k, ~, startPoints, map, goalPoints, algoName, tspAlgo)
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

%% 输入校验
numTargets = size(points, 1);
numRobots = k;

if numTargets == 0
    assignment = [];
    medoids = [];
    history = struct('DV', [], 'baseCost', [], 'detourCost', [], 'algoName', algoName, ...
        'nMigrations', 0, 'totalCostBefore', 0, 'totalCostAfter', 0);
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

%% Step 3: 贪心初始分配 — 每个目标分配给 DV 最小的机器人
assignment = zeros(numTargets, 1);
for t = 1:numTargets
    [~, bestR] = min(DV(:, t));
    assignment(t) = bestR;
end

%% Step 4: 计算初始总 TSP 代价
totalCostBefore = computeAllTSPCost(numRobots, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);

%% Step 5: 二次迁移优化 — 基于 DV 相似度
%   对每个目标，检查其最小 DV 与其他机器人 DV 的相似度。
%   若 minDV / DV(other) > 0.9，尝试迁移到 other 并用 TSP 代价验证。
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

        % DV 相似度 > 0.9，尝试迁移
        % 计算迁移前两机器人 TSP 代价
        costBeforeR1 = computeSingleTSPCost(currentR, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);
        costBeforeR2 = computeSingleTSPCost(otherR, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);

        % 模拟迁移
        trialAssign = assignment;
        trialAssign(t) = otherR;

        % 计算迁移后两机器人 TSP 代价
        costAfterR1 = computeSingleTSPCost(currentR, trialAssign, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);
        costAfterR2 = computeSingleTSPCost(otherR, trialAssign, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);

        % 迁移后总代价减少则采纳
        if (costAfterR1 + costAfterR2) < (costBeforeR1 + costBeforeR2)
            assignment(t) = otherR;
            nMigrations = nMigrations + 1;
            break;  % 该目标已迁移，不再尝试其他机器人
        end
    end
end

%% Step 6: 计算迁移后总 TSP 代价
totalCostAfter = computeAllTSPCost(numRobots, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);

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

end

%% =========================================================================
%  内部函数：计算所有机器人 TSP 总代价
%  =========================================================================
function totalCost = computeAllTSPCost(numRobots, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n)
    totalCost = 0;
    for r = 1:numRobots
        totalCost = totalCost + computeSingleTSPCost(r, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n);
    end
end

%% =========================================================================
%  内部函数：计算单台机器人的 TSP 代价
%  =========================================================================
function cost = computeSingleTSPCost(r, assignment, startPoints, goalPoints, points, map, algoName, tspAlgo, occGrid, n)
    myTargets = points(assignment == r, :);
    if isempty(myTargets)
        % 无分配目标：直接起点→终点
        path = callPlanner(algoName, map, startPoints(r, :), goalPoints(r, :));
        path = SimplifyPath(path, occGrid, n);
        cost = computePathLength(path);
    else
        % 获取 TSP 排序后的各段路径，逐段 SimplifyPath 后计算实际长度
        [~, segPaths, ~] = TSPsolver(startPoints(r, :), myTargets, goalPoints(r, :), map, algoName, tspAlgo);
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
