function [bestOrder, bestCost, history] = TSP_ACO_v1(costMatrix, nPts)
%TSP_ACO_V1 改进蚁群算法求解 TSP（MMAS + 排名沉积 + 启发式初始化）
%   相对 TSP_ACO 的改进:
%     A. MMAS 信息素限幅：防止信息素垄断，避免过早收敛
%     B. 排名沉积：前 K 只蚂蚁按排名沉积信息素，增加多样性
%     C. 启发式初始信息素：用最近邻贪心初始化，加速收敛
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
nAnts   = 60;       % 蚂蚁数量
nIter   = 2000;      % 最大迭代次数
alpha   = 1.0;      % 信息素权重
beta    = 2.0;      % 启发式信息权重
rho     = 0.55;      % 信息素蒸发率
Q       = 100;      % 信息素沉积常数
nRank   = 6;        % 排名沉积的蚂蚁数量（方案 B）

% ===== 自适应停止参数（方案C：种群多样性 + 停滞检测） =====
enableAdaptiveStop = 0;     % 1=开启自适应停止, 0=关闭（使用固定 nIter）
cvThreshold   = 0.005;      % 种群代价变异系数阈值（CV < cvThreshold → 同质收敛）
stagnationLim = 100;         % 最优解连续停滞上限（代）
minIter       = 50;         % 自适应停止最少迭代次数

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

% 启发式信息矩阵 η = 1/d
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

% --- 方案 C：启发式初始信息素（最近邻贪心）---
nnCosts = zeros(nMid, 1);
nnTours = cell(nMid, 1);
for s = 1:nMid
    visited = false(1, nMid);
    visited(s) = true;
    tour = zeros(1, nMid);
    tour(1) = s;
    cost = costMatrix(1, midIdx(s));  % start → 第一个点
    cur = s;
    for step = 2:nMid
        bestD = inf;
        bestJ = -1;
        for j = 1:nMid
            if ~visited(j)
                d = costMatrix(midIdx(cur), midIdx(j));
                if d < bestD
                    bestD = d;
                    bestJ = j;
                end
            end
        end
        cost = cost + bestD;
        visited(bestJ) = true;
        tour(step) = bestJ;
        cur = bestJ;
    end
    cost = cost + costMatrix(midIdx(cur), nPts);  % 最后一个点 → goal
    nnCosts(s) = cost;
    nnTours{s} = tour;
end
[~, nnBestIdx] = min(nnCosts);
nnBestTour = nnTours{nnBestIdx};

% 对最近邻最优解做 2-opt 优化（借鉴 SA_v1 方案 B）
[nnBestTour, nnBestCost] = twoOpt(nnBestTour, midIdx, costMatrix, nPts);

tau0 = Q / nnBestCost;  % 初始信息素基于 2-opt 优化后的最近邻解

% 初始化信息素矩阵
tau = ones(nMid, nMid) * tau0;

% MMAS 信息素上下界（方案 A）
tauMax = 1 / (rho * nnBestCost);
tauMin = tauMax / (2 * nMid);
tauMin = max(tauMin, 1e-6);  % 防止过小
tau = min(tau, tauMax);
tau = max(tau, tauMin);

% 全局最优（用最近邻解作为初始全局最优）
globalBestCost = nnBestCost;
globalBestTour = nnBestTour;

% 自适应停止状态
stagnationCount = 0;        % 连续无改善代数
stopReason = '';            % 停止原因记录

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

        antTours{a} = tour;

        % 计算路径成本
        fullOrder = [1, midIdx(tour), nPts];
        c = 0;
        for k = 1:(length(fullOrder) - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
        antCosts(a) = c;
    end

    % 更新本代最优和全局最优
    [iterBestCost, bestAntIdx] = min(antCosts);
    iterBestTour = antTours{bestAntIdx};

    improved = false;
    if iterBestCost < globalBestCost
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
        improved = true;
    end

    % 记录历史
    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % --- 自适应停止检测（方案C：种群多样性 + 停滞） ---
    if enableAdaptiveStop && iter >= minIter
        cv = std(antCosts) / mean(antCosts);

        % 通道1（快速）：种群同质收敛
        if cv < cvThreshold
            stopReason = sprintf('种群同质(CV=%.4f<%g)', cv, cvThreshold);
            break;
        end

        % 通道2（常规）：最优解停滞
        if improved
            stagnationCount = 0;
        else
            stagnationCount = stagnationCount + 1;
            if stagnationCount >= stagnationLim
                stopReason = sprintf('最优停滞(%d代未改善)', stagnationCount);
                break;
            end
        end
    end

    % --- 信息素蒸发 ---
    tau = tau * (1 - rho);

    % --- 方案 B：排名沉积（前 nRank 只蚂蚁 + 全局最优）---
    [sortedCosts, sortIdx] = sort(antCosts);
    for rank = 1:min(nRank, nAnts)
        deposit = (Q / sortedCosts(rank)) * (nRank - rank + 1) / nRank;
        tour = antTours{sortIdx(rank)};
        for k = 1:(length(tour) - 1)
            i = tour(k); j = tour(k + 1);
            tau(i, j) = tau(i, j) + deposit;
            tau(j, i) = tau(j, i) + deposit;
        end
    end

    % 全局最优额外沉积
    eliteDeposit = Q / globalBestCost;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % --- 方案 A：MMAS 信息素限幅 ---
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);

    % 更新 tauMax（基于全局最优）
    tauMax = 1 / (rho * globalBestCost);
    tauMin = tauMax / (2 * nMid);
    tauMin = max(tauMin, 1e-6);
end

% 输出结果
bestCost = globalBestCost;
bestOrder = [1, midIdx(globalBestTour), nPts];
actualIter = iter;

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory(1:actualIter);
    history.avgCostHistory = avgCostHistory(1:actualIter);
    history.timeHistory = timeHistory(1:actualIter);
    history.iterCount = actualIter;
    history.elapsedTime = toc(tStart);
    if isempty(stopReason)
        stopReason = 'maxIter';
    end
    history.stopReason = stopReason;
end
end

% =========================================================================
%  2-opt 局部搜索（用于初始解优化）
% =========================================================================
function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
%TWOOPT 对中间点排列做 2-opt 边交换优化
%   反复检查所有边对，若交换后总成本下降则执行，直到无改进
%
%   tour    - 1×nMid 中间点排列（midIdx 的索引）
%   midIdx  - 中间点在 costMatrix 中的实际索引
%   costMatrix, nPts - 问题数据
%
%   输出优化后的 tour 和对应的总成本

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

