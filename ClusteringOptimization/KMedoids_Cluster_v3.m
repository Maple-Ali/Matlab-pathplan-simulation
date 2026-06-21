function [assignment, medoids, history] = KMedoids_Cluster_v3(points, k, ~, initMedoids, map, goalPoints, algoName, tspAlgo)
%KMEDOIDS_CLUSTER_V3 K-Medoids 聚类 + 排列搜索初始分配 + 推拉双策略负载均衡
%   v3 在 v2.1 基础上的改进：
%     1. 默认路径距离计算使用 AStar_v1，TSP 求解使用 TSP_ACO_v2
%     2. 初始聚类后用排列搜索优化"哪个机器人负责哪个簇"的映射
%     3. 迁移策略改为推拉双策略：
%        - Direction A（推）：重载机器人中距离轻载簇最近的目标 → 推给该轻载机器人
%        - Direction B（拉）：最轻载机器人从重载簇中拉走距离自己最近的目标
%     4. 负载 = 机器人从起点遍历所有任务点回到终点的总距离
%     5. 终止条件：去掉 5% 均衡阈值，纯靠 noImprove 计数（连续 3 轮无改进即停）
%
%   [assignment, medoids, history] = KMedoids_Cluster_v3(points, k, ...
%       [], startPoints, map, goalPoints, algoName, tspAlgo)
%
%   Inputs:
%     points      - K×2 目标点坐标 [row, col]
%     k           - 聚类数量（机器人数）
%     ~           - 占位（接口兼容）
%     initMedoids - k×2 各机器人起点坐标（初始 medoid）
%     map         - Map 对象（栅格地图）
%     goalPoints  - k×2 各机器人终点坐标 [row, col]
%     algoName    - 全局规划器名称（用于路径距离计算，默认 AStar_v1）
%     tspAlgo     - TSP 求解算法名称（默认 TSP_ACO_v2）
%
%   Outputs:
%     assignment  - K×1 每个目标点的聚类编号 (1..k)
%     medoids     - k×2 最终 medoid 坐标
%     history     - 迭代历史结构体:
%       .initialCosts   - 初始各机器人 TSP 代价 (1×k)
%       .finalCosts     - 最终各机器人 TSP 代价 (1×k)
%       .costHistory    - iter×k 每轮迭代各机器人代价
%       .nIter          - 负载均衡迭代次数
%       .nRelocations   - 成功迁移次数
%       .elapsedTime    - 总耗时(秒)
%       .initialMaxCost - 初始最大代价
%       .finalMaxCost   - 最终最大代价

%% 输入校验与默认值
if nargin < 6 || isempty(goalPoints)
    if nargin >= 5 && ~isempty(map)
        [assignment, medoids, history] = KMedoids_Cluster_v1_1(points, k, [], initMedoids, map);
    else
        [assignment, medoids, history] = KMedoids_Cluster(points, k, [], initMedoids);
    end
    return;
end

if nargin < 7 || isempty(algoName)
    algoName = 'AStar_v1';  % v3 默认使用 AStar_v1 计算路径距离
end
if nargin < 8 || isempty(tspAlgo)
    tspAlgo = 'TSP_ACO_v2';  % v3 默认使用 TSP_ACO_v2 求解 TSP
end

tStart = tic;

nTargets = size(points, 1);
nRobots = k;

if nTargets == 0
    assignment = zeros(0, 1);
    medoids = initMedoids;
    history = struct('initialCosts', [], 'finalCosts', [], ...
        'costHistory', [], 'nIter', 0, 'nRelocations', 0, 'elapsedTime', 0);
    return;
end

% 获取 TSP 算法函数句柄
tspFunc = str2func(tspAlgo);

%% Step 1: 构建全量点集，预计算路径距离矩阵 D

% allPts 布局: [startPoints(N); targets(K); goalPoints(N)]
allPts = [initMedoids; points; goalPoints];     % (N+K+N) × 2
totalPts = size(allPts, 1);                     % = 2*N + K

idxStart  = 1:nRobots;                          % 起点在 allPts 中的索引
idxTarget = nRobots + (1:nTargets);              % 目标点在 allPts 中的索引
idxGoal   = nRobots + nTargets + (1:nRobots);    % 终点在 allPts 中的索引

% 预计算全部点对之间的路径距离（使用选定的全局规划器）
D = zeros(totalPts, totalPts);
for i = 1:totalPts
    for j = i+1:totalPts
        p = callPlanner(algoName, map, allPts(i,:), allPts(j,:));
        if ~isempty(p)
            d = pathLength(p);
        else
            d = 1e6;  % 不可达惩罚
        end
        D(i, j) = d;
        D(j, i) = d;
    end
end

%% Step 2: 初始粗分配（K-Medoids + 路径距离）

% 提取目标点之间的 K×K 路径距离子矩阵
targetDistMatrix = D(idxTarget, idxTarget);

% 调用基础 K-Medoids，用起点作为初始 medoid
[assignment, ~, ~] = KMedoids_Cluster(points, nRobots, ...
    targetDistMatrix, initMedoids);

% assignment 为 K×1，值为 1..nRobots

%% Step 2.5（v3 新增）: 排列搜索优化"簇→机器人"映射
% K-Medoids 聚类后，尝试所有 K! 种 (机器人→簇) 映射，
% 选最大单机器人代价最小的排列（minimax，而非总代价最小）

if nRobots <= 5
    % K≤5 时穷举所有排列
    allPerms = perms(1:nRobots);
    nPerms = size(allPerms, 1);
    bestPermMaxCost = inf;
    bestPerm = allPerms(1, :);

    for p = 1:nPerms
        perm = allPerms(p, :);
        permMaxCost = 0;
        for r = 1:nRobots
            clusterId = perm(r);
            c = computeClusterTSPCost(r, clusterId, assignment, D, ...
                idxStart, idxTarget, idxGoal, tspFunc);
            if c > permMaxCost
                permMaxCost = c;
            end
        end
        if permMaxCost < bestPermMaxCost
            bestPermMaxCost = permMaxCost;
            bestPerm = perm;
        end
    end
else
    % K>5 时贪心近似：每轮选 TSP 代价最小的 (机器人, 簇) 配对
    availableClusters = 1:nRobots;
    bestPerm = zeros(1, nRobots);
    for r = 1:nRobots
        bestCost = inf;
        bestC = availableClusters(1);
        for ci = 1:length(availableClusters)
            c = availableClusters(ci);
            cost = computeClusterTSPCost(r, c, assignment, D, ...
                idxStart, idxTarget, idxGoal, tspFunc);
            if cost < bestCost
                bestCost = cost;
                bestC = c;
            end
        end
        bestPerm(r) = bestC;
        availableClusters(availableClusters == bestC) = [];
    end
end

% 按最优排列重新映射 assignment
% 原 assignment 中值 v 表示"属于簇 v"，需要映射为"属于机器人 r"，其中 bestPerm(r) = v
newAssignment = zeros(nTargets, 1);
for r = 1:nRobots
    clusterId = bestPerm(r);
    newAssignment(assignment == clusterId) = r;
end
assignment = newAssignment;

%% Step 3: 计算每台机器人的初始完整 TSP 代价

robotCosts = zeros(1, nRobots);
for r = 1:nRobots
    robotCosts(r) = computeRobotTSPCost(r, assignment, D, idxStart, idxTarget, idxGoal, tspFunc);
end

initialCosts = robotCosts;

%% Step 4: 迭代负载均衡（v3 核心：多配对穷举 + 推拉双策略）

maxIter = 30;
noImproveLimit = 3;  % 固定值，无 5% 阈值
costHistory = zeros(maxIter, nRobots);
costHistory(1, :) = robotCosts;
nRelocations = 0;
iter = 0;
noImprove = 0;

while iter < maxIter
    iter = iter + 1;

    % 按代价排序，从重到轻
    [sortedCosts, sortedIdx] = sort(robotCosts, 'descend');
    maxCost = sortedCosts(1);

    improved = false;

    % ======== Phase 1: 多配对穷举（v2.1 策略：遍历多组 (重,轻) 配对）========
    nHeavy = min(2, nRobots);
    nLight = min(2, nRobots);

    for hi = 1:nHeavy
        heavyR = sortedIdx(hi);
        heavyTargets = find(assignment == heavyR);
        if isempty(heavyTargets)
            continue;
        end

        for li = nRobots:-1:(nRobots - nLight + 1)
            lightR = sortedIdx(li);
            if lightR == heavyR; continue; end

            % 穷举 heavyR 的所有目标，找使 max(newH, newL) 最小的候选
            bestNewMax = inf;
            bestTarget = 0;
            bestNewCosts = [0, 0];

            for t = 1:length(heavyTargets)
                candidateTarget = heavyTargets(t);
                trialAssign = assignment;
                trialAssign(candidateTarget) = lightR;

                newCostH = computeRobotTSPCost(heavyR, trialAssign, D, ...
                    idxStart, idxTarget, idxGoal, tspFunc);
                newCostL = computeRobotTSPCost(lightR, trialAssign, D, ...
                    idxStart, idxTarget, idxGoal, tspFunc);

                newMax = max(newCostH, newCostL);
                if newMax < bestNewMax
                    bestNewMax = newMax;
                    bestTarget = candidateTarget;
                    bestNewCosts = [newCostH, newCostL];
                end
            end

            if bestNewMax < maxCost
                assignment(bestTarget) = lightR;
                robotCosts(heavyR) = bestNewCosts(1);
                robotCosts(lightR) = bestNewCosts(2);
                nRelocations = nRelocations + 1;
                improved = true;
                break;
            end
        end
        if improved; break; end
    end

    % ======== Phase 2: 推（Direction A）— 仅在 Phase 1 失败时执行 ========
    if ~improved
        heavyR = sortedIdx(1);
        heavyTargets = find(assignment == heavyR);

        if ~isempty(heavyTargets)
            % 对 heavyR 的每个目标，找最近的轻载机器人（以路径距离衡量）
            bestPushDist = inf;
            bestPushTarget = 0;
            bestPushDest = 0;

            for t = 1:length(heavyTargets)
                cand = heavyTargets(t);
                for li = nRobots:-1:2
                    lightR = sortedIdx(li);
                    if lightR == heavyR; continue; end
                    d = D(idxTarget(cand), idxStart(lightR));
                    if d < bestPushDist
                        bestPushDist = d;
                        bestPushTarget = cand;
                        bestPushDest = lightR;
                    end
                end
            end

            if bestPushTarget > 0
                trialAssign = assignment;
                trialAssign(bestPushTarget) = bestPushDest;
                newCostH = computeRobotTSPCost(heavyR, trialAssign, D, ...
                    idxStart, idxTarget, idxGoal, tspFunc);
                newCostL = computeRobotTSPCost(bestPushDest, trialAssign, D, ...
                    idxStart, idxTarget, idxGoal, tspFunc);

                if max(newCostH, newCostL) < maxCost
                    assignment(bestPushTarget) = bestPushDest;
                    robotCosts(heavyR) = newCostH;
                    robotCosts(bestPushDest) = newCostL;
                    nRelocations = nRelocations + 1;
                    improved = true;
                end
            end
        end
    end

    % ======== Phase 3: 拉（Direction B）— 仅在 Phase 1+2 均失败时执行 ========
    if ~improved
        lightR = sortedIdx(end);

        bestPullDist = inf;
        bestPullTarget = 0;
        bestPullSource = 0;

        for hi = 1:(nRobots - 1)
            heavyR2 = sortedIdx(hi);
            heavyTargets2 = find(assignment == heavyR2);
            for t = 1:length(heavyTargets2)
                cand = heavyTargets2(t);
                d = D(idxTarget(cand), idxStart(lightR));
                if d < bestPullDist
                    bestPullDist = d;
                    bestPullTarget = cand;
                    bestPullSource = heavyR2;
                end
            end
        end

        if bestPullTarget > 0
            trialAssign = assignment;
            trialAssign(bestPullTarget) = lightR;
            newCostS = computeRobotTSPCost(bestPullSource, trialAssign, D, ...
                idxStart, idxTarget, idxGoal, tspFunc);
            newCostL = computeRobotTSPCost(lightR, trialAssign, D, ...
                idxStart, idxTarget, idxGoal, tspFunc);

            if max(newCostS, newCostL) < maxCost
                assignment(bestPullTarget) = lightR;
                robotCosts(bestPullSource) = newCostS;
                robotCosts(lightR) = newCostL;
                nRelocations = nRelocations + 1;
                improved = true;
            end
        end
    end

    % 记录本轮代价
    costHistory(iter, :) = robotCosts;

    % 终止判断：纯靠 noImprove 计数，无 5% 阈值
    if ~improved
        noImprove = noImprove + 1;
        if noImprove >= noImproveLimit
            break;
        end
    else
        noImprove = 0;
    end
end

%% Step 5: 输出

% 截断 costHistory 到实际迭代数
costHistory = costHistory(1:iter, :);

% 重新计算 medoids（每个簇选到簇内其他目标点距离之和最小的目标点）
medoids = zeros(nRobots, 2);
for r = 1:nRobots
    members = find(assignment == r);
    if isempty(members)
        medoids(r, :) = initMedoids(r, :);
    elseif isscalar(members)
        medoids(r, :) = points(members, :);
    else
        bestCost = inf;
        for m = 1:length(members)
            c = 0;
            mi = members(m);
            for o = 1:length(members)
                if members(o) ~= mi
                    c = c + targetDistMatrix(mi, members(o));
                end
            end
            if c < bestCost
                bestCost = c;
                medoids(r, :) = points(mi, :);
            end
        end
    end
end

history.initialCosts = initialCosts;
history.finalCosts   = robotCosts;
history.costHistory   = costHistory;
history.nIter         = iter;
history.nRelocations  = nRelocations;
history.elapsedTime   = toc(tStart);
history.initialMaxCost = max(initialCosts);
history.finalMaxCost   = max(robotCosts);

end

% =========================================================================
function cost = computeRobotTSPCost(r, assignment, D, idxStart, idxTarget, idxGoal, tspFunc)
% 计算机器人 r 的完整 TSP 代价（起点 → 分配目标点 → 终点）
% 使用预计算的路径距离矩阵 D 查表，不调用路径规划器

myTargetLocal = find(assignment == r);
nMyTargets = length(myTargetLocal);

if nMyTargets == 0
    cost = D(idxStart(r), idxGoal(r));
    return;
end

if nMyTargets == 1
    tIdx = idxTarget(myTargetLocal);
    cost = D(idxStart(r), tIdx) + D(tIdx, idxGoal(r));
    return;
end

subGlobalIdx = [idxStart(r), idxTarget(myTargetLocal), idxGoal(r)];
subSize = length(subGlobalIdx);
subMatrix = D(subGlobalIdx, subGlobalIdx);
[~, cost] = tspFunc(subMatrix, subSize);

end

% =========================================================================
function cost = computeClusterTSPCost(r, clusterId, assignment, D, idxStart, idxTarget, idxGoal, tspFunc)
% 计算机器人 r 负责簇 clusterId 时的 TSP 代价
% 用于排列搜索阶段，assignment 中值为 clusterId 的目标分配给机器人 r

myTargetLocal = find(assignment == clusterId);
nMyTargets = length(myTargetLocal);

if nMyTargets == 0
    cost = D(idxStart(r), idxGoal(r));
    return;
end

if nMyTargets == 1
    tIdx = idxTarget(myTargetLocal);
    cost = D(idxStart(r), tIdx) + D(tIdx, idxGoal(r));
    return;
end

subGlobalIdx = [idxStart(r), idxTarget(myTargetLocal), idxGoal(r)];
subSize = length(subGlobalIdx);
subMatrix = D(subGlobalIdx, subGlobalIdx);
[~, cost] = tspFunc(subMatrix, subSize);

end

% =========================================================================
function d = pathLength(path)
d = 0;
for i = 1:size(path, 1) - 1
    d = d + norm(path(i+1, :) - path(i, :));
end
end

% =========================================================================
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
        path = AStar_v1(map, startGrid, goalGrid, 0);  % v3 默认 fallback
end
end
