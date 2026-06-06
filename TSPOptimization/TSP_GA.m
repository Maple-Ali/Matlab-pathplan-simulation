function [bestOrder, bestCost, history] = TSP_GA(costMatrix, nPts)
%TSP_GA 遗传算法求解 TSP
%   适用于目标点数 > 5 的中大规模问题，采用 OX 交叉 + 锦标赛选择
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
nGen = 100;
mutationRate = 0.1;

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nGen, 1);
    avgCostHistory = zeros(nGen, 1);
    timeHistory = zeros(nGen, 1);
end

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
end

fit = zeros(popSize, 1);
for i = 1:popSize
    fit(i) = fitness(pop(i, :));
end
[bestCost, bestIdx] = min(fit);
bestOrder = [1, pop(bestIdx, :), nPts];

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
