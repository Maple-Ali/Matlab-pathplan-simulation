function [orderedPoints, segPaths, totalCost] = TSPsolver(startPoint, targets, goalPoint, map, algoName)
%TSPSOLVER TSP 多目标排序求解器
%   [orderedPoints, segPaths, totalCost] = TSPsolver(startPoint, targets, goalPoint, map, algoName)
%   startPoint: [row, col] 起点栅格
%   targets: K×2 目标点 [row, col]
%   goalPoint: [row, col] 终点栅格
%   map: Map 对象
%   algoName: 'AStar' | 'Dijkstra' | 'RRT'
%   输出:
%   orderedPoints: 有序访问点（含起点终点）
%   segPaths: 每段之间的全局路径 cell 数组
%   totalCost: 总路径成本

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

% --- TSP 求解：起点(1)固定第一个，终点(nPts)固定最后一个，中间点排列 ---
midIdx = 2:(nPts - 1);
nMid = length(midIdx);

if nMid == 0
    % 没有中间目标点，起点直达终点
    bestOrder = [1, nPts];
    totalCost = costMatrix(1, nPts);
elseif nMid <= 5
    % 全排列枚举
    perms_all = perms(midIdx);
    bestCost = inf;
    bestPerm = midIdx;
    for p = 1:size(perms_all, 1)
        order = [1, perms_all(p, :), nPts];
        c = 0;
        valid = true;
        for k = 1:(length(order) - 1)
            if isinf(costMatrix(order(k), order(k + 1)))
                valid = false;
                break;
            end
            c = c + costMatrix(order(k), order(k + 1));
        end
        if valid && c < bestCost
            bestCost = c;
            bestPerm = perms_all(p, :);
        end
    end
    bestOrder = [1, bestPerm, nPts];
    totalCost = bestCost;
else
    % 遗传算法
    [bestOrder, totalCost] = gaTSP(costMatrix, nPts);
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
        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0);
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

function [bestOrder, bestCost] = gaTSP(costMatrix, nPts)
    % 简单遗传算法求解 TSP
    nMid = nPts - 2;
    midIdx = 2:(nPts - 1);
    popSize = 50;
    nGen = 100;
    mutationRate = 0.1;

    % 初始化种群
    pop = zeros(popSize, nMid);
    for i = 1:popSize
        pop(i, :) = midIdx(randperm(nMid));
    end

    function c = fitness(order)
        fullOrder = [1, order, nPts];
        c = 0;
        for k = 1:(length(fullOrder) - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
    end

    for gen = 1:nGen
        % 评估适应度
        fit = zeros(popSize, 1);
        for i = 1:popSize
            fit(i) = fitness(pop(i, :));
        end

        % 选择（锦标赛）
        newPop = zeros(popSize, nMid);
        for i = 1:popSize
            candidates = randi(popSize, [2, 1]);
            [~, winner] = min(fit(candidates));
            newPop(i, :) = pop(candidates(winner), :);
        end

        % 交叉（OX 交叉）
        for i = 1:2:popSize
            if rand < 0.8 && i + 1 <= popSize
                p1 = newPop(i, :);
                p2 = newPop(i + 1, :);
                cp = sort(randi(nMid, [1, 2]));
                % 子代1
                child1 = zeros(1, nMid);
                child1(cp(1):cp(2)) = p1(cp(1):cp(2));
                remain = setdiff(p2, child1(cp(1):cp(2)), 'stable');
                idx = 1;
                for j = 1:nMid
                    if child1(j) == 0
                        child1(j) = remain(idx);
                        idx = idx + 1;
                    end
                end
                % 子代2
                child2 = zeros(1, nMid);
                child2(cp(1):cp(2)) = p2(cp(1):cp(2));
                remain = setdiff(p1, child2(cp(1):cp(2)), 'stable');
                idx = 1;
                for j = 1:nMid
                    if child2(j) == 0
                        child2(j) = remain(idx);
                        idx = idx + 1;
                    end
                end
                newPop(i, :) = child1;
                newPop(i + 1, :) = child2;
            end
        end

        % 变异
        for i = 1:popSize
            if rand < mutationRate
                swap = randperm(nMid, 2);
                newPop(i, swap) = newPop(i, swap(end:-1:1));
            end
        end

        pop = newPop;
    end

    fit = zeros(popSize, 1);
    for i = 1:popSize
        fit(i) = fitness(pop(i, :));
    end
    [bestCost, bestIdx] = min(fit);
    bestOrder = [1, pop(bestIdx, :), nPts];
end
