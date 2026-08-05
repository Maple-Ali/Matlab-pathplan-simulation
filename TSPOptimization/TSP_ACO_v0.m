function [bestOrder, bestCost, history] = TSP_ACO_v0(costMatrix, nPts)
%TSP_ACO_V0 蚁群算法 v0（原版 + MMAS + ASRank）求解 TSP
%   基于原始 TSP_ACO，仅加入两项经典改进:
%     A. MMAS 信息素限幅 — 防止信息素垄断，避免过早收敛
%     B. ASRank 排名沉积 — top nRank 蚂蚁按排名加权，保留次优路径信息
%        (可通过 enableASRank 开关启用/禁用，禁用时回退为迭代最优沉积)
%
%   无 2-opt、无 NN 初始化、无自适应停止 — 便于与原版对比改进效果。
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost, history

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 40;
nIter = 1500;
alpha = 1.0;
beta  = 2.0;
rho   = 0.25;
Q     = 225;
tau0  = 0.1;

% ===== 方案 B：ASRank 排名沉积 =====
enableASRank = 0;          % 1=启用排名沉积, 0=仅迭代最优沉积
nRank = 6;                 % 参与沉积的蚂蚁数（仅 enableASRank=1 时生效）

trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nIter, 1);
    avgCostHistory = zeros(nIter, 1);
    timeHistory = zeros(nIter, 1);
end

if nMid == 0
    bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% 初始化信息素矩阵（均匀）
tau = ones(nMid, nMid) * tau0;

% ---- 方案 A：MMAS 初始信息素限幅 ----
% 初始 tauMax 基于 P(ρ, τ₀) 先保守估计，找到第一个解后动态更新
tauMax = 1 / (rho * tau0 * nMid);  % 保守上界，待首次全局最优后更新
tauMin = tauMax / (5 * nMid);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

% 启发式信息矩阵 η = 1/d
eta = zeros(nMid, nMid);
for i = 1:nMid
    for j = 1:nMid
        if i ~= j
            d = costMatrix(midIdx(i), midIdx(j));
            if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
        end
    end
end

globalBestCost = inf;
globalBestTour = midIdx;  % 占位

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
                next = find(cumsum(prob) >= rand(), 1, 'first');
            end
            tour(step) = next;
            visited(next) = true;
        end

        antTours{a} = tour;
        fullOrder = [1, midIdx(tour), nPts];
        c = 0;
        for k = 1:(nPts - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
        antCosts(a) = c;
    end

    % 更新全局最优
    [iterBestCost, bestAntIdx] = min(antCosts);
    if iterBestCost < globalBestCost
        globalBestCost = iterBestCost;
        globalBestTour = antTours{bestAntIdx};
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % --- 信息素蒸发 ---
    tau = tau * (1 - rho);

    % ---- 方案 B：ASRank 排名沉积 (可开关) ----
    if enableASRank
        % 排名沉积: top nRank 按排名加权
        [sortedCosts, sortIdx] = sort(antCosts);
        for rank = 1:min(nRank, nAnts)
            weight = (nRank - rank + 1) / nRank;  % 排名1→权重1, 排名nRank→权重1/nRank
            deposit = (Q / sortedCosts(rank)) * weight;
            rTour = antTours{sortIdx(rank)};
            for k = 1:(length(rTour) - 1)
                i = rTour(k); j = rTour(k + 1);
                tau(i, j) = tau(i, j) + deposit;
                tau(j, i) = tau(j, i) + deposit;
            end
        end
    else
        % 回退: 仅迭代最优沉积 (标准 AS)
        deposit = Q / iterBestCost;
        rTour = antTours{bestAntIdx};
        for k = 1:(length(rTour) - 1)
            i = rTour(k); j = rTour(k + 1);
            tau(i, j) = tau(i, j) + deposit;
            tau(j, i) = tau(j, i) + deposit;
        end
    end

    % 全局最优精英沉积
    eliteDeposit = Q / globalBestCost;
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

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory;
    history.avgCostHistory = avgCostHistory;
    history.timeHistory = timeHistory;
    history.iterCount = nIter;
    history.elapsedTime = toc(tStart);
end
end
