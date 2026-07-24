function [bestOrder, bestCost, history] = TSP_ACO_v1_2(costMatrix, nPts)
%TSP_ACO_V1_2 蚁群算法 + 2-opt + 三项增强求解 TSP
%   基于 TSP_ACO_v1_1（原始 ACO + 2-opt），新增:
%     A. MMAS 信息素限幅 — 防止 2-opt 导致信息素爆炸
%     B. 自适应停止 — 种群多样性 + 停滞检测，避免 500 代全跑
%     C. NN 启发式初始信息素 — 从最近邻解出发，加速早期收敛
%
%   Inputs/Outputs: 同 TSP_ACO

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 50;              % 蚂蚁数量
nIter  = 500;            % 最大迭代次数
alpha  = 1.0;            % 信息素权重
beta   = 2.0;            % 启发式信息权重
rho    = 0.5;            % 信息素蒸发率
Q      = 100;            % 信息素沉积常数

% ===== 方案 B：自适应停止参数 =====
enableAdaptiveStop = 1;  % 1=开启
cvThreshold  = 0.001;    % 种群代价变异系数阈值（<cvThreshold → 同质收敛）
stagnationLim = 80;      % 最优解连续停滞上限（代）
minIter      = 40;       % 最少迭代代数

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nIter, 1);
    avgCostHistory = zeros(nIter, 1);
    timeHistory = zeros(nIter, 1);
end

if nMid == 0
    bestOrder = [1, nPts];
    bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
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

% ---- 方案 C：NN 启发式初始信息素 ----
nnCosts = zeros(nMid, 1);
nnTours = cell(nMid, 1);
for s = 1:nMid
    visited = false(1, nMid); visited(s) = true;
    tour = zeros(1, nMid); tour(1) = s;
    cur = s;
    cost = costMatrix(1, midIdx(s));
    for step = 2:nMid
        bestD = inf; bestJ = -1;
        for j = 1:nMid
            if ~visited(j)
                d = costMatrix(midIdx(cur), midIdx(j));
                if d < bestD, bestD = d; bestJ = j; end
            end
        end
        cost = cost + bestD; visited(bestJ) = true;
        tour(step) = bestJ; cur = bestJ;
    end
    cost = cost + costMatrix(midIdx(cur), nPts);
    nnCosts(s) = cost;
    nnTours{s} = tour;
end
[~, nnBestIdx] = min(nnCosts);
nnBestTour = nnTours{nnBestIdx};

% 2-opt 优化 NN 最佳解
[nnBestTour, nnBestCost] = twoOpt(nnBestTour, midIdx, costMatrix, nPts);

% 基于 2-opt 优化后的 NN 代价设定信息素
tau0 = Q / nnBestCost;
tau = ones(nMid, nMid) * tau0;

% ---- 方案 A：MMAS 信息素限幅 ----
tauMax = 1 / (rho * nnBestCost);
tauMin = tauMax / (2 * nMid);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

% 全局最优初始化
globalBestCost = nnBestCost;
globalBestTour = nnBestTour;

% ---- 方案 B：自适应停止状态 ----
stagnationCount = 0;
stopReason = '';

for iter = 1:nIter
    antTours = cell(nAnts, 1);
    antCosts = zeros(nAnts, 1);

    for a = 1:nAnts
        tour = zeros(1, nMid);
        visited = false(1, nMid);
        start = randi(nMid);
        tour(1) = start;
        visited(start) = true;

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

        % 2-opt 局部搜索
        [tour, c] = twoOpt(tour, midIdx, costMatrix, nPts);

        antTours{a} = tour;
        antCosts(a) = c;
    end

    [iterBestCost, bestAntIdx] = min(antCosts);
    iterBestTour = antTours{bestAntIdx};

    improved = false;
    if iterBestCost < globalBestCost
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
        improved = true;
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % ---- 方案 B：自适应停止检测 ----
    if enableAdaptiveStop && iter >= minIter
        cv = std(antCosts) / mean(antCosts);
        if cv < cvThreshold
            stopReason = sprintf('种群同质(CV=%.4f<%g)', cv, cvThreshold);
            break;
        end
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

    % --- 信息素沉积（本代最优）---
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1)
        i = iterBestTour(k); j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end

    % 精英策略：全局最优额外沉积
    eliteDeposit = Q / globalBestCost * 0.5;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % ---- 方案 A：MMAS 信息素限幅 + 动态更新 ----
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);
    tauMax = 1 / (rho * globalBestCost);
    tauMin = tauMax / (2 * nMid);
    tauMin = max(tauMin, 1e-6);
end

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
%  2-opt 局部搜索
% =========================================================================
function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
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
