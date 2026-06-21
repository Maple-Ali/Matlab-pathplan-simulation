function [assignment, medoids, history] = KMedoids_Cluster_v2_1(points, k, ~, initMedoids, map, goalPoints, algoName, tspAlgo)
%KMEDOIDS_CLUSTER_V2_1 K-Medoids 聚类 + 全路径代价迭代负载均衡（v2.1 改进版）
%   在 K-Medoids 初始聚类基础上，通过反复检查每台机器人完整 TSP 代价
%   （起点→目标点→终点），遍历重载机器人所有目标寻找最优迁移，
%   支持多配对探索，实现更稳定的负载均衡。
%
%   v2.1 相比 v2 的改进：
%   - 穷举候选目标：用真实 TSP 代价选出最优迁移目标（而非启发式最近距离）
%   - 多配对探索：尝试 top-2 最重 × bottom-2 最轻配对（而非仅最重→最轻）
%   - 失败换候选：配对失败自动换下一组，避免卡死在单一 (heavyR, lightR)
%
%   [assignment, medoids, history] = KMedoids_Cluster_v2_1(points, k, ...
%       [], startPoints, map, goalPoints, algoName, tspAlgo)
%
%   Inputs:
%     points      - K×2 目标点坐标 [row, col]
%     k           - 聚类数量（机器人数）
%     ~           - 占位（接口兼容）
%     initMedoids - k×2 各机器人起点坐标（初始 medoid）
%     map         - Map 对象（栅格地图）
%     goalPoints  - k×2 各机器人终点坐标 [row, col]
%     algoName    - 全局规划器名称（用于路径距离计算）
%     tspAlgo     - TSP 求解算法名称
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

%% 输入校验与默认值
if nargin < 6 || isempty(goalPoints)
    % 未提供终点 → 回退到 v1_1（纯路径距离聚类，无负载均衡）
    if nargin >= 5 && ~isempty(map)
        [assignment, medoids, history] = KMedoids_Cluster_v1_1(points, k, [], initMedoids, map);
    else
        [assignment, medoids, history] = KMedoids_Cluster(points, k, [], initMedoids);
    end
    return;
end

if nargin < 7 || isempty(algoName)
    algoName = 'AStar_v3';
end
if nargin < 8 || isempty(tspAlgo)
    tspAlgo = 'TSP_GA';
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

%% Step 3: 计算每台机器人的初始完整 TSP 代价

robotCosts = zeros(1, nRobots);
for r = 1:nRobots
    robotCosts(r) = computeRobotTSPCost(r, assignment, D, idxStart, idxTarget, idxGoal, tspFunc);
end

initialCosts = robotCosts;

%% Step 4: 迭代负载均衡（v2.1 改进版：多配对 + 穷举最优候选）

maxIter = 30;
noImproveLimit = 3;
costHistory = zeros(maxIter, nRobots);
costHistory(1, :) = robotCosts;
nRelocations = 0;
iter = 0;
noImprove = 0;

while iter < maxIter
    iter = iter + 1;

    % 按代价排序，从重到轻
    [sortedCosts, sortedIdx] = sort(robotCosts, 'descend');
    oldMaxCost = sortedCosts(1);

    % 已足够均衡（差距 < 5%）→ 提前退出
    if oldMaxCost > 0 && (sortedCosts(1) - sortedCosts(end)) < oldMaxCost * 0.05
        break;
    end

    improved = false;

    % 尝试前 2 最重 × 后 2 最轻 的配对
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
            if lightR == heavyR
                continue;
            end

            % 穷举 heavyR 的所有目标，找使 max(newH, newL) 最小的候选
            bestNewMax = inf;
            bestTarget = 0;
            bestNewCosts = [0, 0];

            for t = 1:length(heavyTargets)
                candidateTarget = heavyTargets(t);

                % 模拟迁移：暂将该目标分配给 lightR
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

            % 接受准则：全局最大代价下降（不恶化全局最重负担）
            if bestNewMax < oldMaxCost
                assignment(bestTarget) = lightR;
                robotCosts(heavyR) = bestNewCosts(1);
                robotCosts(lightR) = bestNewCosts(2);
                nRelocations = nRelocations + 1;
                improved = true;
                break;  % 跳出 lightR 循环
            end
        end
        if improved
            break;  % 跳出 heavyR 循环
        end
    end

    % 记录本轮代价
    costHistory(iter, :) = robotCosts;

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
        % 空簇：保留原起点作为 medoid
        medoids(r, :) = initMedoids(r, :);
    elseif isscalar(members)
        medoids(r, :) = points(members, :);
    else
        % 使用路径距离选最优 medoid
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

myTargetLocal = find(assignment == r);           % 本机器人目标点的本地索引 (1..K)
nMyTargets = length(myTargetLocal);

if nMyTargets == 0
    % 无分配目标：直接起点 → 终点
    cost = D(idxStart(r), idxGoal(r));
    return;
end

if nMyTargets == 1
    % 仅1个目标：唯一路线 起点 → 目标 → 终点
    tIdx = idxTarget(myTargetLocal);
    cost = D(idxStart(r), tIdx) + D(tIdx, idxGoal(r));
    return;
end

% 构建子矩阵对应的全局索引列表: [start, target1, ..., targetM, goal]
subGlobalIdx = [idxStart(r), idxTarget(myTargetLocal), idxGoal(r)];
subSize = length(subGlobalIdx);

% 从 D 中提取子矩阵
subMatrix = D(subGlobalIdx, subGlobalIdx);

% 调用 TSP 算法（接口统一: [bestOrder, bestCost] = tspFunc(costMatrix, nPts)）
[~, cost] = tspFunc(subMatrix, subSize);

end

% =========================================================================
function d = pathLength(path)
% 计算路径总长度
d = 0;
for i = 1:size(path, 1) - 1
    d = d + norm(path(i+1, :) - path(i, :));
end
end

% =========================================================================
function path = callPlanner(algoName, map, startGrid, goalGrid)
% 根据 algoName 调用对应的全局规划器
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
        path = AStar_v3(map, startGrid, goalGrid, 0);
end
end
