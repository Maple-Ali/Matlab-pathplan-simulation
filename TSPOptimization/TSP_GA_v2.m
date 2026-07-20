function [bestOrder, bestCost, history] = TSP_GA_v2(costMatrix, nPts)
%TSP_GA_V2 改进遗传算法求解 TSP（启发式初始化 + 精英保留 + 自适应变异 + 2-opt）
%   相对 TSP_GA_v1 的新增:
%     A. 最近邻启发式初始化部分种群，提升起点质量
%     B. 精英保留策略：每代最优个体直接进入下一代
%     C. 自适应变异：连续无改进时提高变异率，跳出局部最优
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体

nMid = nPts - 2;
midIdx = 2:(nPts - 1);
popSize = 50;
nGen = 1000;
baseMutationRate = 0.1;
maxMutationRate = 0.5;
nElite = 2;           % 精英保留个数
nHeuristic = 10;      % 启发式初始化个数

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nGen, 1);
    avgCostHistory = zeros(nGen, 1);
    timeHistory = zeros(nGen, 1);
end

% ---- 初始化种群 ----
pop = zeros(popSize, nMid);

% 前 nHeuristic 个用最近邻启发式
for s = 1:min(nHeuristic, nMid)
    visited = false(1, nMid);
    visited(s) = true;
    tour = zeros(1, nMid);
    tour(1) = midIdx(s);
    cur = s;
    for step = 2:nMid
        bestD = inf;
        bestJ = 0;
        for j = 1:nMid
            if ~visited(j)
                d = costMatrix(midIdx(cur), midIdx(j));
                if d < bestD
                    bestD = d;
                    bestJ = j;
                end
            end
        end
        visited(bestJ) = true;
        tour(step) = midIdx(bestJ);
        cur = bestJ;
    end
    pop(s, :) = tour;
end

% 其余随机初始化
for i = (nHeuristic + 1):popSize
    pop(i, :) = midIdx(randperm(nMid));
end

    function c = fitness(order)
        fullOrder = [1, order, nPts];
        c = 0;
        for k = 1:(length(fullOrder) - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
    end

globalBestCost = inf;
globalBestMid = [];
noImproveCount = 0;
mutationRate = baseMutationRate;

for gen = 1:nGen
    % 评估适应度
    fit = zeros(popSize, 1);
    for i = 1:popSize
        fit(i) = fitness(pop(i, :));
    end

    [genBestCost, genBestIdx] = min(fit);

    % 更新全局最优
    if genBestCost < globalBestCost - 1e-10
        globalBestCost = genBestCost;
        globalBestMid = pop(genBestIdx, :);
        noImproveCount = 0;
        mutationRate = baseMutationRate;
    else
        noImproveCount = noImproveCount + 1;
    end

    % 自适应变异：连续 5 代无改进则提高变异率
    if noImproveCount >= 5
        mutationRate = min(mutationRate * 1.5, maxMutationRate);
    end

    % 记录收敛历史
    if trackHistory
        bestCostHistory(gen) = globalBestCost;
        avgCostHistory(gen) = mean(fit);
        timeHistory(gen) = toc(tStart);
    end

    % ---- 选择（锦标赛）----
    newPop = zeros(popSize, nMid);
    for i = 1:popSize
        candidates = randi(popSize, [2, 1]);
        [~, winner] = min(fit(candidates));
        newPop(i, :) = pop(candidates(winner), :);
    end

    % ---- 交叉（OX 交叉）----
    for i = 1:2:popSize
        if rand < 0.8 && i + 1 <= popSize
            p1 = newPop(i, :);
            p2 = newPop(i + 1, :);
            cp = sort(randi(nMid, [1, 2]));
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

    % ---- 变异 ----
    for i = 1:popSize
        if rand < mutationRate
            swap = randperm(nMid, 2);
            newPop(i, swap) = newPop(i, swap(end:-1:1));
        end
    end

    % ---- 精英保留：将最优个体替换到新种群 ----
    [~, sortIdx] = sort(fit);
    for e = 1:nElite
        newPop(e, :) = pop(sortIdx(e), :);
    end

    pop = newPop;

    % ---- 精英 2-opt ----
    [pop(1, :), ~] = twoOpt(pop(1, :), costMatrix, nPts);
end

% 最终 2-opt 精炼
[globalBestMid, bestCost] = twoOpt(globalBestMid, costMatrix, nPts);
bestOrder = [1, globalBestMid, nPts];

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory;
    history.avgCostHistory = avgCostHistory;
    history.timeHistory = timeHistory;
    history.iterCount = nGen;
    history.elapsedTime = toc(tStart);
end
end

% =========================================================================
%  2-opt 局部搜索
% =========================================================================
function [tour, cost] = twoOpt(tour, costMatrix, nPts)
nMid = length(tour);
fullOrder = [1, tour, nPts];

improved = true;
while improved
    improved = false;
    for i = 1:(nMid - 1)
        for j = (i + 1):nMid
            fi = i + 1;
            fj = j + 1;
            oldCost = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + ...
                      costMatrix(fullOrder(fj), fullOrder(fj + 1));
            newCost = costMatrix(fullOrder(fi), fullOrder(fj)) + ...
                      costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
            if newCost < oldCost - 1e-10
                tour((i + 1):j) = tour(j:-1:(i + 1));
                improved = true;
                fullOrder = [1, tour, nPts];
            end
        end
    end
end

cost = 0;
for k = 1:(nPts - 1)
    cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1));
end
end
