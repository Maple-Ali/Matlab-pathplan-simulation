%% temp_profile_DV_v2.m — DV_Cluster_v2 分阶段耗时分析脚本
%  临时脚本：记录 DV_Cluster_v2 各阶段的运算时间、步骤、数据量。
%  场景：杂乱不规则_1 | AStar_v1 | TSP_ACO_v2

clear variables;
close all;

%% ======================== 环境初始化 ========================
rootDir = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(rootDir));

%% ======================== 加载预设地图 ========================
[map, mapData] = loadPresetMap('杂乱不规则_1');
startPoints = mapData.startPoints;
targets     = mapData.targetPoints;
goalPoints  = mapData.goalPoints;

numRobots  = size(startPoints, 1);
numTargets = size(targets, 1);

algoName = 'AStar_v1';
tspAlgo  = 'TSP_ACO_v2';

fprintf('\n========================================\n');
fprintf('  DV_Cluster_v2 分阶段性能分析\n');
fprintf('========================================\n');
fprintf('  场景: 杂乱不规则_1\n');
fprintf('  机器人数: %d\n', numRobots);
fprintf('  目标点数: %d\n', numTargets);
fprintf('  静态障碍: %d\n', size(mapData.staticObstacles, 1));
fprintf('  全局规划: %s\n', algoName);
fprintf('  TSP求解:  %s\n\n', tspAlgo);

%% ======================== 运行分析 ========================
% 直接调用原版 DV_Cluster_v2，在各阶段前后插入 tic/toc
%
% DV_Cluster_v2 内部阶段划分（源码行号参考）：
%   [0] Init         — pathCache, occGrid, n (L57-63)
%   [1] baseCost     — N× A* start→goal (L65-74)
%   [2] DV Matrix    — N×K×2 A* calls (L76-89)
%   [3] Greedy Assign— min DV per target (L91-96)
%   [4] Initial TSP  — N× TSPsolver (L98-103)
%   [5] Migration    — DV similarity + TSP re-eval (L105-153)
%   [6] Final&Out    — sum cost + build history (L155-173)

% 我们重放 DV_Cluster_v2 的逻辑，手动分段计时
% 这种方式不会修改原文件，同时给出精确的阶段耗时

profileData = runInstrumentedDV2(map, startPoints, targets, goalPoints, algoName, tspAlgo);

%% ======================== 输出报告 ========================
fprintf('\n');
fprintf('┌─────────────────────────────────────────────────────────────────────┐\n');
fprintf('│  阶段            耗时(ms)     占比    说明                          │\n');
fprintf('├─────────────────────────────────────────────────────────────────────┤\n');

totalMs = profileData.totalTime * 1000;

stageNames = {
    '0-Init', ...
    '1-baseCost (起点→终点)', ...
    '2-DV矩阵 (偏离量)', ...
    '3-贪心初始分配', ...
    '4-初始TSP代价', ...
    '5-二次迁移优化', ...
    '6-输出汇总'};

for s = 1:7
    ms = profileData.stageTimes(s) * 1000;
    pct = ms / totalMs * 100;
    desc = profileData.stageDescriptions{s};
    fprintf('│  %-18s %8.1f   %5.1f%%%%   %-40s│\n', stageNames{s}, ms, pct, desc);
end

fprintf('├─────────────────────────────────────────────────────────────────────┤\n');
fprintf('│  %-18s %8.1f   %5.1f%%%%                                          │\n', ...
    '总计', totalMs, 100.0);
fprintf('└─────────────────────────────────────────────────────────────────────┘\n');

fprintf('\n--- 关键指标 ---\n');
fprintf('  总 A* 调用次数:   %d\n', profileData.nAStarCalls);
fprintf('  路径缓存命中:     %d\n', profileData.nCacheHit);
fprintf('  路径缓存未命中:   %d\n', profileData.nCacheMiss);
fprintf('  缓存条目数:       %d\n', profileData.nCacheEntries);
fprintf('  迁移尝试/成功:    %d / %d\n', profileData.nMigrationAttempts, profileData.nMigrations);
fprintf('  TSP 初始总代价:   %.2f\n', profileData.totalCostBefore);
fprintf('  TSP 迁移后总代价: %.2f\n', profileData.totalCostAfter);
fprintf('  DV 矩阵范围:      [%.3f, %.3f]\n', min(profileData.DV(:)), max(profileData.DV(:)));
for r = 1:numRobots
    fprintf('  机器人%d 分配目标: %d 个\n', r, profileData.targetsPerRobot(r));
end

fprintf('\n--- 缓存效率 ---\n');
fprintf('  命中率: %.1f%% (%d/%d)\n', ...
    profileData.nCacheHit / max(1, profileData.nCacheHit + profileData.nCacheMiss) * 100, ...
    profileData.nCacheHit, profileData.nCacheHit + profileData.nCacheMiss);

fprintf('\n分析完成。\n');

%% ======================== 内部函数 ========================
function profileData = runInstrumentedDV2(map, startPoints, targets, goalPoints, algoName, tspAlgo)
% runInstrumentedDV2  手动分段重放 DV_Cluster_v2 逻辑并计时
%   保持与原算法完全相同的计算逻辑，在各阶段插入 tic/toc

    numTargets = size(targets, 1);
    numRobots  = size(startPoints, 1);

    profileData = struct();
    profileData.stageTimes = zeros(7, 1);
    profileData.stageDescriptions = cell(7, 1);
    profileData.nAStarCalls = 0;
    profileData.nCacheHit = 0;
    profileData.nCacheMiss = 0;

    if numTargets == 0
        profileData.totalTime = 0;
        profileData.nCacheEntries = 0;
        profileData.nMigrations = 0;
        profileData.nMigrationAttempts = 0;
        profileData.totalCostBefore = 0;
        profileData.totalCostAfter = 0;
        profileData.DV = [];
        profileData.targetsPerRobot = zeros(1, numRobots);
        return;
    end

    %% ============ [0] Init ============
    t0 = tic;

    occGrid = map.getOccupancyGrid();
    n = map.mapSize;

    % 路径缓存
    pathCache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    nCacheHit = 0;
    nCacheMiss = 0;
    nAStar = 0;

    % 辅助闭包：带计数的缓存规划器（使用嵌套函数句柄）
    % MATLAB 中我们直接用内联逻辑

    profileData.stageTimes(1) = toc(t0);
    profileData.stageDescriptions{1} = sprintf('pathCache=Map(), occGrid=%d×%d', n, n);

    %% ============ [1] baseCost ============
    t1 = tic;

    baseCost = zeros(1, numRobots);
    for r = 1:numRobots
        [path, nCacheHit, nCacheMiss, nAStar] = ...
            cp(map, startPoints(r, :), goalPoints(r, :), pathCache, nCacheHit, nCacheMiss, nAStar, algoName);
        path = SimplifyPath(path, occGrid, n);
        baseCost(r) = pathLen(path);
    end
    baseCost(baseCost == 0) = 1;  % 防除零

    profileData.stageTimes(2) = toc(t1);
    bcStr = strjoin(cellstr(num2str(round(baseCost, 1))), ', ');
    profileData.stageDescriptions{2} = sprintf('%d× A* start→goal, baseCost=[%s]', ...
        numRobots, bcStr);

    %% ============ [2] DV Matrix ============
    t2 = tic;

    detourCost = zeros(numRobots, numTargets);
    nAStar_beforeDV = nAStar;
    for r = 1:numRobots
        for t = 1:numTargets
            [pathToTarget, nCacheHit, nCacheMiss, nAStar] = ...
                cp(map, startPoints(r, :), targets(t, :), pathCache, nCacheHit, nCacheMiss, nAStar, algoName);
            pathToTarget = SimplifyPath(pathToTarget, occGrid, n);
            [pathToGoal, nCacheHit, nCacheMiss, nAStar] = ...
                cp(map, targets(t, :), goalPoints(r, :), pathCache, nCacheHit, nCacheMiss, nAStar, algoName);
            pathToGoal = SimplifyPath(pathToGoal, occGrid, n);
            detourCost(r, t) = pathLen(pathToTarget) + pathLen(pathToGoal);
        end
    end
    nAStar_duringDV = nAStar - nAStar_beforeDV;

    DV = detourCost ./ baseCost(:);

    profileData.stageTimes(3) = toc(t2);
    profileData.stageDescriptions{3} = sprintf('%d×%d=%.0f 对 A* (%d新+%d缓存), DV %.0f×%.0f', ...
        numRobots, numTargets, numRobots*numTargets*2, ...
        nCacheMiss - profileData.nCacheMiss, nCacheHit - profileData.nCacheHit, ...
        size(DV,1), size(DV,2));

    %% ============ [3] Greedy Initial Assignment ============
    t3 = tic;

    assignment = zeros(numTargets, 1);
    for t = 1:numTargets
        [~, bestR] = min(DV(:, t));
        assignment(t) = bestR;
    end

    targetsPerRobot = zeros(1, numRobots);
    for r = 1:numRobots
        targetsPerRobot(r) = sum(assignment == r);
    end

    profileData.stageTimes(4) = toc(t3);
    robotAllocStr = strjoin(arrayfun(@(r) sprintf('R%d=%d', r, targetsPerRobot(r)), 1:numRobots, 'UniformOutput', false), ', ');
    profileData.stageDescriptions{4} = sprintf('分配: %s', robotAllocStr);

    %% ============ [4] Initial TSP Cost ============
    t4 = tic;

    robotTSPCost = zeros(1, numRobots);
    for r = 1:numRobots
        robotTSPCost(r) = computeSingleTSPCost(r, assignment, startPoints, goalPoints, ...
            targets, map, algoName, tspAlgo, occGrid, n, pathCache);
    end
    totalCostBefore = sum(robotTSPCost);

    profileData.stageTimes(5) = toc(t4);
    tcStr = strjoin(cellstr(num2str(round(robotTSPCost, 1))), ', ');
    profileData.stageDescriptions{5} = sprintf('%d× TSPsolver, cost=%s, total=%.1f', ...
        numRobots, tcStr, totalCostBefore);

    %% ============ [5] Migration Optimization ============
    t5 = tic;

    nMigrations = 0;
    nMigrationAttempts = 0;

    for t = 1:numTargets
        currentR = assignment(t);
        currentDV = DV(currentR, t);

        [sortedDV, sortedIdx] = sort(DV(:, t), 'ascend');

        for si = 2:numRobots
            otherR = sortedIdx(si);
            otherDV = sortedDV(si);
            similarity = currentDV / otherDV;

            if similarity < 0.95
                break;
            end

            nMigrationAttempts = nMigrationAttempts + 1;

            costBeforeR1 = robotTSPCost(currentR);
            costBeforeR2 = robotTSPCost(otherR);

            trialAssign = assignment;
            trialAssign(t) = otherR;

            costAfterR1 = computeSingleTSPCost(currentR, trialAssign, startPoints, goalPoints, ...
                targets, map, algoName, tspAlgo, occGrid, n, pathCache);
            costAfterR2 = computeSingleTSPCost(otherR, trialAssign, startPoints, goalPoints, ...
                targets, map, algoName, tspAlgo, occGrid, n, pathCache);

            if (costAfterR1 + costAfterR2) < (costBeforeR1 + costBeforeR2)
                assignment(t) = otherR;
                robotTSPCost(currentR) = costAfterR1;
                robotTSPCost(otherR) = costAfterR2;
                nMigrations = nMigrations + 1;
                break;
            end
        end
    end

    profileData.stageTimes(6) = toc(t5);
    profileData.stageDescriptions{6} = sprintf('检查%d目标, %d次尝试, %d次成功迁移', ...
        numTargets, nMigrationAttempts, nMigrations);

    %% ============ [6] Final Summary ============
    t6 = tic;

    totalCostAfter = sum(robotTSPCost);
    nCacheEntries = length(pathCache.keys());

    profileData.stageTimes(7) = toc(t6);
    profileData.stageDescriptions{7} = sprintf('totalCost: %.1f→%.1f, 缓存%d条', ...
        totalCostBefore, totalCostAfter, nCacheEntries);

    %% 汇总
    profileData.totalTime = sum(profileData.stageTimes);
    profileData.nAStarCalls = nAStar;
    profileData.nCacheHit = nCacheHit;
    profileData.nCacheMiss = nCacheMiss;
    profileData.nCacheEntries = nCacheEntries;
    profileData.nMigrations = nMigrations;
    profileData.nMigrationAttempts = nMigrationAttempts;
    profileData.totalCostBefore = totalCostBefore;
    profileData.totalCostAfter = totalCostAfter;
    profileData.DV = DV;
    profileData.targetsPerRobot = targetsPerRobot;

    %% ======================== 嵌套辅助函数 ========================

    function [path, nH, nM, nA] = cp(mapObj, sGrid, gGrid, cache, nH, nM, nA, algo)
    % cp — cachedPlanner: 带计数的缓存包装
        cacheKey = sprintf('%d,%d_%d,%d', sGrid(1), sGrid(2), gGrid(1), gGrid(2));
        if cache.isKey(cacheKey)
            path = cache(cacheKey);
            nH = nH + 1;
        else
            path = callAlgo(algo, mapObj, sGrid, gGrid);
            nA = nA + 1;
            nM = nM + 1;
            if ~isempty(path)
                cache(cacheKey) = path;
            end
        end
    end

    function path = callAlgo(name, mapObj, sGrid, gGrid)
        switch name
            case 'AStar_v1'
                path = AStar_v1(mapObj, sGrid, gGrid, 0);
            case 'AStar'
                path = AStar(mapObj, sGrid, gGrid, 0);
            case 'Dijkstra'
                path = Dijkstra(mapObj, sGrid, gGrid, 0);
            case 'RRT'
                path = RRT(mapObj, sGrid, gGrid, 0);
            otherwise
                path = AStar(mapObj, sGrid, gGrid, 0);
        end
    end

    function len = pathLen(p)
        if size(p, 1) < 2
            len = 0;
        else
            d = diff(p, 1, 1);
            len = sum(sqrt(sum(d.^2, 2)));
        end
    end

    function cost = computeSingleTSPCost(r, assign, sPts, gPts, pts, mapObj, algo, tsp, og, nn, cache)
        myTargets = pts(assign == r, :);
        if isempty(myTargets)
            [path, ~, ~, ~] = cp(mapObj, sPts(r, :), gPts(r, :), cache, 0, 0, 0, algo);
            path = SimplifyPath(path, og, nn);
            cost = pathLen(path);
        else
            [~, segPaths, ~] = TSPsolver(sPts(r, :), myTargets, gPts(r, :), mapObj, algo, tsp, false, [], [], cache);
            cost = 0;
            for s = 1:length(segPaths)
                seg = SimplifyPath(segPaths{s}, og, nn);
                cost = cost + pathLen(seg);
            end
        end
    end
end
