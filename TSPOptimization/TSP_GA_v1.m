function [bestOrder, bestCost, history] = TSP_GA_v1(costMatrix, nPts)
%TSP_GA_V1 改进遗传算法求解 TSP（GA + 2-opt 局部搜索）
%   相对 TSP_GA 的新增:
%     A. GA 结束后对全局最优做 2-opt 精炼
%     B. 每代最优个体也做 2-opt 加速收敛
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体:
%       .bestCostHistory  - 每代最优成本
%       .avgCostHistory   - 每代平均成本
%       .timeHistory      - 每代累计耗时（秒）
%       .iterCount        - 总迭代代数
%       .elapsedTime      - 纯计算耗时（秒）

nMid = nPts - 2;
midIdx = 2:(nPts - 1);
popSize = 50;
nGen = 500;
mutationRate = 0.1;

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nGen, 1);
    avgCostHistory = zeros(nGen, 1);
    timeHistory = zeros(nGen, 1);
end

% 初始化种群（存储实际点索引）
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

    % 记录收敛历史
    if trackHistory
        bestCostHistory(gen) = min(fit);
        avgCostHistory(gen) = mean(fit);
        timeHistory(gen) = toc(tStart);
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

    % 精英 2-opt：对当代最优个体做局部搜索
    [~, bestIdx] = min(fit);
    [pop(bestIdx, :), ~] = twoOpt(pop(bestIdx, :), costMatrix, nPts);
end

% 最终评估
fit = zeros(popSize, 1);
for i = 1:popSize
    fit(i) = fitness(pop(i, :));
end
[~, bestIdx] = min(fit);
bestMid = pop(bestIdx, :);

% 对全局最优做 2-opt 精炼
[bestMid, bestCost] = twoOpt(bestMid, costMatrix, nPts);
bestOrder = [1, bestMid, nPts];

% 输出收敛历史
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
%  2-opt 局部搜索（直接操作实际点索引）
% =========================================================================
function [tour, cost] = twoOpt(tour, costMatrix, nPts)
%TWOOPT 对中间点排列做 2-opt 边交换优化
%   tour    - 1×nMid 中间点排列（实际点索引 2..nPts-1）
%   输出优化后的 tour 和对应的总成本

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

% 从 costMatrix 重算真实代价
cost = 0;
for k = 1:(nPts - 1)
    cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1));
end
end
