function [bestOrder, bestCost, history] = TSP_ACO_v1_4(costMatrix, nPts)
%TSP_ACO_V1_4 蚁群算法 + 选择性 2-opt + 排名沉积 + 自适应蒸发 求解 TSP
%   基于 TSP_ACO_v1_3，新增:
%     H. 排名沉积: top nRank 蚂蚁按排名加权沉积，保留次优路径的有用边
%     I. 自适应蒸发率: rho 前期大（加速探索）→ 后期小（精细开发）
%
%   保留:
%     A. MMAS 信息素限幅
%     B. 自适应停止
%     C. NN 启发式初始信息素
%     F. 选择性 2-opt
%     G. 局部信息素更新 (ACS 风格)

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 50;              % 蚂蚁数量
nIter  = 500;            % 最大迭代次数
alpha  = 1.0;            % 信息素权重
beta   = 2.0;            % 启发式信息权重
Q      = 100;            % 信息素沉积常数

% ===== 方案 H：排名沉积 =====
nRank = 6;               % 每代参与沉积的蚂蚁数（前 nRank 名）

% ===== 方案 I：自适应蒸发率 =====
enableAdaptiveRho = 0;   % 1=蒸发率随迭代递减, 0=固定 rho
rho_max = 0.5;           % 蒸发率上限（早期快速探索）
rho_min = 0.1;           % 蒸发率下限（后期精细开发）

% ===== 方案 F：选择性 2-opt =====
enableSelectiveOpt = 1;
optRatio = 0.30;

% ===== 方案 G：局部信息素更新 (ACS 风格) =====
enableLocalUpdate = 0;
xi    = 0.10;

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;
cvThreshold  = 0.001;
stagnationLim = 80;
minIter      = 40;

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

% 启发式信息矩阵
eta = zeros(nMid, nMid);
for i = 1:nMid
    for j = 1:nMid
        if i ~= j
            d = costMatrix(midIdx(i), midIdx(j));
            if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
        end
    end
end

% ---- 方案 C：NN 启发式初始信息素 ----
nnCosts = zeros(nMid, 1);
nnTours = cell(nMid, 1);
for s = 1:nMid
    visited = false(1, nMid); visited(s) = true;
    tour = zeros(1, nMid); tour(1) = s; cur = s;
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
    nnCosts(s) = cost; nnTours{s} = tour;
end
[~, nnBestIdx] = min(nnCosts);
nnBestTour = nnTours{nnBestIdx};
[nnBestTour, nnBestCost] = twoOpt(nnBestTour, midIdx, costMatrix, nPts);

% 初始蒸发率用于计算初始信息素界
rho_init = rho_max;  % 用上界计算保守的初始信息素界
tau0 = Q / nnBestCost;
tau = ones(nMid, nMid) * tau0;

% ---- 方案 A：MMAS 信息素限幅 ----
tauMax = 1 / (rho_init * nnBestCost);
tauMin = tauMax / (2 * nMid);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

globalBestCost = nnBestCost;
globalBestTour = nnBestTour;
stagnationCount = 0;
stopReason = '';

nOptAnts = max(1, round(nAnts * optRatio));

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
            curIdx = tour(step - 1);
            prob = zeros(1, nMid);
            for j = 1:nMid
                if ~visited(j)
                    prob(j) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta);
                end
            end
            totalProb = sum(prob);
            if totalProb == 0
                unv = find(~visited);
                nxt = unv(randi(length(unv)));
            else
                prob = prob / totalProb;
                nxt = find(cumsum(prob) >= rand(), 1, 'first');
            end
            tour(step) = nxt;
            visited(nxt) = true;
        end

        % ---- 方案 G：局部信息素更新 ----
        if enableLocalUpdate
            for k = 1:(length(tour) - 1)
                i = tour(k); j = tour(k + 1);
                tau(i, j) = (1 - xi) * tau(i, j) + xi * tau0;
                tau(j, i) = (1 - xi) * tau(j, i) + xi * tau0;
            end
        end

        fullOrder = [1, midIdx(tour), nPts];
        c = 0;
        for k = 1:(nPts - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
        antTours{a} = tour;
        antCosts(a) = c;
    end

    % ---- 方案 F：选择性 2-opt ----
    if enableSelectiveOpt && nOptAnts < nAnts
        [~, sortIdx] = sort(antCosts);
        for a_idx = 1:nOptAnts
            a = sortIdx(a_idx);
            [antTours{a}, antCosts(a)] = twoOpt(antTours{a}, midIdx, costMatrix, nPts);
        end
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

    % ---- 方案 B：自适应停止 ----
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

    % ---- 方案 I：自适应蒸发率 ----
    if enableAdaptiveRho
        rho = rho_min + (rho_max - rho_min) * (1 - iter / nIter);
    else
        rho = rho_max;
    end

    % --- 全局信息素蒸发 ---
    tau = tau * (1 - rho);

    % ---- 方案 H：排名沉积（top nRank 按排名加权） ----
    [sortedCosts, sortedIdx] = sort(antCosts);
    for rank = 1:min(nRank, nAnts)
        weight = (nRank - rank + 1) / nRank;  % 第1名权重=1, 第nRank名权重=1/nRank
        deposit = (Q / sortedCosts(rank)) * weight;
        rTour = antTours{sortedIdx(rank)};
        for k = 1:(length(rTour) - 1)
            i = rTour(k); j = rTour(k + 1);
            tau(i, j) = tau(i, j) + deposit;
            tau(j, i) = tau(j, i) + deposit;
        end
    end

    % 全局最优额外精英沉积
    eliteDeposit = Q / globalBestCost;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % ---- 方案 A：MMAS 限幅 + 动态更新（rho 变化时同步更新界限） ----
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
    if isempty(stopReason), stopReason = 'maxIter'; end
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
for k = 1:(nPts - 1)
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
