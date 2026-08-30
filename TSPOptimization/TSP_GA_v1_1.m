function [bestOrder, bestCost, history] = TSP_GA_v1_1(costMatrix, nPts)
%TSP_GA_V1_1 遗传算法求解 TSP（精英保留 + 自适应变异）
%   基于传统 GA，仅保留两项改进:
%     A. 精英保留策略: 每代最优 2 个个体直接进入下一代
%     B. 自适应变异: 连续无改进时提高变异率，跳出局部最优
%
%   移除 v2 的:
%     - 2-opt 局部搜索（每代精英 + 最终精炼）
%     - 最近邻启发式初始化
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 可调参数 =====
% 以下参数直接影响遗传算法的性能与收敛行为，请根据问题规模与特性调整。

popSize = 300;                % 种群大小：每一代中个体的数量。
                              % 较大的种群可增加多样性，但会提高计算成本。
                              % 建议范围：50 ~ 500。对中小规模 TSP（nPts < 100）可取 100~200。

nGen = 1200;                  % 最大进化代数：算法停止迭代的代数上限。
                              % 若收敛较早，可通过自适应变异或精英策略提前跳出。
                              % 建议根据问题复杂度调整，可结合收敛历史观察是否需要更多代数。

crossoverRate = 0.80;         % 交叉概率：每对父代进行 OX 交叉的概率。
                              % 高交叉率促进探索，但可能破坏优良基因；低交叉率收敛慢。
                              % 常用范围：0.6 ~ 0.9。

baseMutationRate = 0.20;       % 基础变异率：每个个体发生交换变异的初始概率。
                              % 变异引入新基因，防止早熟。针对 TSP 通常不宜过高，否则退化为随机搜索。
                              % 常用范围：0.01 ~ 0.2，较大值适合高多样性需求。

maxMutationRate = 0.5;         % 最大变异率：自适应变异时变异率的上限。
                              % 防止变异率无限增大导致搜索完全随机化。
                              % 建议不超过 0.5，否则算法失去导向性。

nElite = 25;                   % 精英保留个数：每代直接保留的最优个体数量。
                              % 精英策略保证最优解不丢失，加速收敛。
                              % 取值通常为种群大小的 5%~10%，过大可能降低多样性。

noImproveThr = 60;             % 连续无改善阈值：若连续这么多代全局最优未改进，则触发自适应变异。
                              % 阈值太小会频繁提高变异率，太大则难以跳出局部最优。
                              % 建议与总代数配合，例如设为总代数的 5%~10%。

mutationBoost = 2;              % 变异率放大因子：触发自适应变异时，当前变异率乘以该因子。
                              % 例如当前变异率为 0.2，放大后为 0.4，但不超过 maxMutationRate。
                              % 常用值 1.5 ~ 3，决定跳出局部最优的力度。

% 注意：调整参数时建议先固定其他参数，逐一测试，并观察收敛曲线（可通过输出 history 分析）。

trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nGen, 1);
    avgCostHistory = zeros(nGen, 1);
    timeHistory = zeros(nGen, 1);
end

if nMid == 0
    bestOrder = [1, nPts];
    bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% ---- 随机初始化种群 ----
pop = zeros(popSize, nMid);
for i = 1:popSize
    pop(i, :) = midIdx(randperm(nMid));
end

% 适应度函数（成本越低越优）
    function c = fitness(order)
        fullOrder = [1, order, nPts];
        c = 0;
        for k = 1:(nPts - 1)
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
        mutationRate = baseMutationRate;  % 恢复基础变异率
    else
        noImproveCount = noImproveCount + 1;
    end

    % ---- 方案 B：自适应变异 ----
    if noImproveCount >= noImproveThr
        mutationRate = min(mutationRate * mutationBoost, maxMutationRate);
    end

    % 记录收敛历史
    if trackHistory
        bestCostHistory(gen) = globalBestCost;
        avgCostHistory(gen) = mean(fit);
        timeHistory(gen) = toc(tStart);
    end

    % ---- 选择（锦标赛） ----
    newPop = zeros(popSize, nMid);
    for i = 1:popSize
        candidates = randi(popSize, [2, 1]);
        [~, winner] = min(fit(candidates));
        newPop(i, :) = pop(candidates(winner), :);
    end

    % ---- 交叉（OX 交叉） ----
    for i = 1:2:popSize
        if rand < crossoverRate && i + 1 <= popSize
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

    % ---- 变异（交换变异） ----
    for i = 1:popSize
        if rand < mutationRate
            swap = randperm(nMid, 2);
            newPop(i, swap) = newPop(i, swap(end:-1:1));
        end
    end

    % ---- 方案 A：精英保留 ----
    [~, sortIdx] = sort(fit);
    for e = 1:nElite
        newPop(e, :) = pop(sortIdx(e), :);
    end

    pop = newPop;
end

bestCost = globalBestCost;
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
