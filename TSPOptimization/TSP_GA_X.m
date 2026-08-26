function [bestOrder, bestCost, history] = TSP_GA_X(costMatrix, nPts)
%TSP_GA_X 遗传算法求解 TSP（虚拟节点法：起点终点固定 + 中间点全排列优化）
%   在原 TSP_GA 基础上，将起点终点固定问题通过虚拟节点转化为闭合环路 TSP。
%   虚拟节点 V 与起点(1)、终点(nPts) 距离为 0，与中间点距离 BIG。
%   求解闭合环路后，去掉虚拟节点并校正方向，得到 起点→中间点→终点 的开路径。
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵 (1=起点, nPts=终点)
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量, 1 在首、nPts 在尾）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体

nNodes = nPts;  % 闭合环路: 排列 1:nPts, 虚拟节点隐含
popSize = 150;
nGen = 2000;
mutationRate = 0.02;

% 扩展成本矩阵: 添加虚拟节点 V = nPts+1
Vidx = nPts + 1;
nExt = nPts + 1;
maxFinite = max(costMatrix(costMatrix < inf));
if isempty(maxFinite), maxFinite = 1; end
BIG = (nPts + 1) * maxFinite * 2;

costExt = zeros(nExt);
costExt(1:nPts, 1:nPts) = costMatrix;
costExt(Vidx, 1) = 0;  costExt(1, Vidx) = 0;
costExt(Vidx, nPts) = 0;  costExt(nPts, Vidx) = 0;
midIdx = 2:(nPts - 1);
costExt(Vidx, midIdx) = BIG;  costExt(midIdx, Vidx) = BIG;

trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nGen, 1);
    avgCostHistory = zeros(nGen, 1);
    timeHistory = zeros(nGen, 1);
end

% 初始化种群: 每个个体为 1:nPts 的随机排列
pop = zeros(popSize, nNodes);
for i = 1:popSize
    pop(i, :) = randperm(nNodes);
end

    function c = fitness(order)
        % 闭合环路成本 (V 隐含在 costExt 中)
        c = 0;
        for k = 1:nNodes
            c = c + costExt(order(k), order(mod(k, nNodes) + 1));
        end
    end

for gen = 1:nGen
    fit = zeros(popSize, 1);
    for i = 1:popSize
        fit(i) = fitness(pop(i, :));
    end

    if trackHistory
        bestCostHistory(gen) = min(fit);
        avgCostHistory(gen) = mean(fit);
        timeHistory(gen) = toc(tStart);
    end

    % 选择（锦标赛）
    newPop = zeros(popSize, nNodes);
    for i = 1:popSize
        candidates = randi(popSize, [2, 1]);
        [~, winner] = min(fit(candidates));
        newPop(i, :) = pop(candidates(winner), :);
    end

    % 交叉（OX 交叉）
    for i = 1:2:popSize
        if rand < 0.9 && i + 1 <= popSize
            p1 = newPop(i, :);
            p2 = newPop(i + 1, :);
            cp = sort(randi(nNodes, [1, 2]));
            child1 = zeros(1, nNodes);
            child1(cp(1):cp(2)) = p1(cp(1):cp(2));
            remain = setdiff(p2, child1(cp(1):cp(2)), 'stable');
            idx = 1;
            for j = 1:nNodes
                if child1(j) == 0
                    child1(j) = remain(idx);
                    idx = idx + 1;
                end
            end
            child2 = zeros(1, nNodes);
            child2(cp(1):cp(2)) = p2(cp(1):cp(2));
            remain = setdiff(p1, child2(cp(1):cp(2)), 'stable');
            idx = 1;
            for j = 1:nNodes
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
            swap = randperm(nNodes, 2);
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
bestCycle = pop(bestIdx, :);

% 提取开路径: 去掉虚拟节点, 校正方向, 返回开路径成本
bestOrder = extractPath(bestCycle, nPts, Vidx);
bestCost = bestCost - costExt(bestCycle(nNodes), bestCycle(1));  % 减去闭合边

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
function order = extractPath(cycle, nPts, Vidx)
n = length(cycle);
vPos = find(cycle == Vidx, 1);
if isempty(vPos)
    % cycle 不含 V (排列 1:nPts), 直接作为路径并校正方向
    order = cycle;
else
    order = zeros(1, nPts);
    idx = mod(vPos, n) + 1;
    for t = 1:nPts
        order(t) = cycle(idx);
        idx = mod(idx, n) + 1;
    end
end
if order(1) == nPts
    order = fliplr(order);
end
if order(1) ~= 1
    p1 = find(order == 1, 1);
    order = [order(p1:end), order(1:p1-1)];
end
end
