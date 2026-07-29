function [bestOrder, bestCost, history] = TSP_ACO_v1_1(costMatrix, nPts)
%TSP_ACO_V1_1 蚁群算法 + 2-opt 局部搜索求解 TSP
%   基于原始 TSP_ACO，唯一改进：每只蚂蚁构造解后执行 2-opt 局部搜索
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

% 算法参数
nAnts = 60;          % 蚂蚁数量
nIter = 300;         % 最大迭代次数
alpha = 1.0;         % 信息素权重
beta = 2.0;          % 启发式信息权重（距离倒数）
rho = 0.3;           % 信息素蒸发率
Q = 150;             % 信息素沉积常数
tau0 = 0.1;          % 初始信息素浓度

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nIter, 1);
    avgCostHistory = zeros(nIter, 1);
    timeHistory = zeros(nIter, 1);
end

% 边界情况：无中间目标点
if nMid == 0
    bestOrder = [1, nPts];
    bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, ...
            'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% 初始化信息素矩阵（仅中间点之间的边）
tau = ones(nMid, nMid) * tau0;

% 启发式信息矩阵 η = 1/d（仅中间点之间）
eta = zeros(nMid, nMid);
for i = 1:nMid
    for j = 1:nMid
        if i ~= j
            d = costMatrix(midIdx(i), midIdx(j));
            if d > 0 && ~isinf(d)
                eta(i, j) = 1.0 / d;
            end
        end
    end
end

% 全局最优
globalBestCost = inf;
globalBestTour = midIdx;

for iter = 1:nIter
    % --- 蚂蚁构造解 ---
    antTours = cell(nAnts, 1);
    antCosts = zeros(nAnts, 1);

    for a = 1:nAnts
        tour = zeros(1, nMid);
        visited = false(1, nMid);

        % 随机选择起始中间点
        start = randi(nMid);
        tour(1) = start;
        visited(start) = true;

        % 逐步构建路径
        for step = 2:nMid
            cur = tour(step - 1);
            % 计算转移概率
            prob = zeros(1, nMid);
            for j = 1:nMid
                if ~visited(j)
                    prob(j) = (tau(cur, j) ^ alpha) * (eta(cur, j) ^ beta);
                end
            end
            totalProb = sum(prob);
            if totalProb == 0
                unvisited = find(~visited);
                next = unvisited(randi(length(unvisited)));
            else
                prob = prob / totalProb;
                cumProb = cumsum(prob);
                r = rand();
                next = find(cumProb >= r, 1, 'first');
            end
            tour(step) = next;
            visited(next) = true;
        end

        % --- 2-opt 局部搜索（v1.1 唯一改进） ---
        [tour, c] = twoOpt(tour, midIdx, costMatrix, nPts);

        antTours{a} = tour;
        antCosts(a) = c;
    end

    % 更新本代最优
    [iterBestCost, bestAntIdx] = min(antCosts);
    iterBestTour = antTours{bestAntIdx};

    if iterBestCost < globalBestCost
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
    end

    % 记录历史
    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % --- 信息素蒸发 ---
    tau = tau * (1 - rho);

    % --- 信息素沉积（本代最优路径）---
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1)
        i = iterBestTour(k);
        j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end

    % 额外沉积全局最优路径（精英策略）
    eliteDeposit = Q / globalBestCost * 0.5;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k);
        j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end
end

% 输出结果
bestCost = globalBestCost;
bestOrder = [1, midIdx(globalBestTour), nPts];

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory;
    history.avgCostHistory = avgCostHistory;
    history.timeHistory = timeHistory;
    history.iterCount = nIter;
    history.elapsedTime = toc(tStart);
end
end

% =========================================================================
%  2-opt 局部搜索（v1.1 唯一改进）
% =========================================================================
function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
%TWOOPT 对中间点排列做 2-opt 边交换优化
%   反复检查所有边对，若交换后总成本下降则执行，直到无改进

nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];

cost = 0;
for k = 1:(length(fullOrder) - 1)
    cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1));
end

improved = true;
while improved
    improved = false;
    for i = 1:(nMid - 1)
        for j = (i + 1):nMid
            fi = i + 1; fj = j + 1;
            oldCost = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + ...
                      costMatrix(fullOrder(fj), fullOrder(fj + 1));
            newCost = costMatrix(fullOrder(fi), fullOrder(fj)) + ...
                      costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
            if newCost < oldCost - 1e-10
                tour((i + 1):j) = tour(j:-1:(i + 1));
                cost = cost - oldCost + newCost;
                improved = true;
                fullOrder = [1, midIdx(tour), nPts];
            end
        end
    end
end
end
