function [bestOrder, bestCost, history] = TSP_ACO_v1_6(costMatrix, nPts)
%TSP_ACO_V1_6 蚁群 + 2-opt + 三大防过早收敛机制 求解 TSP
%   基于 TSP_ACO_v1_3，新增:
%     K. 伪随机比例规则 (ACS 风格): 构造时以概率 q0 贪心选边, 1-q0 随机探索
%     M. Double-bridge 突变: 2-opt 后随机重组片段，跳出 2-opt 不可达区域
%     O. 动态启发式权重: beta 从低到高递增，早期探索 → 后期开发
%
%   保留:
%     A. MMAS 信息素限幅
%     B. 自适应停止
%     C. NN 启发式初始信息素
%     F. 选择性 2-opt
%     G. 局部信息素更新

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 50;
nIter  = 500;
alpha  = 1.0;

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 1;  % 1=开启
q0 = 0.80;               % 贪心选择概率（高值=更贪心，低值=更探索）

% ===== 方案 O：动态启发式权重 =====
enableDynamicBeta = 1;   % 1=beta 从低到高递增
beta_min = 1.0;          % 早期（弱启发式，自由探索）
beta_max = 3.0;          % 后期（强启发式，精准开发）

% ===== 方案 F：选择性 2-opt =====
enableSelectiveOpt = 1;
optRatio = 0.30;

% ===== 方案 M：Double-bridge 突变 =====
enableDoubleBridge = 1;  % 1=底部蚂蚁施加 double-bridge
dbRatio = 0.10;          % 施加突变的底部蚂蚁比例（0.1 = bottom 10%）

rho    = 0.3;
Q      = 150;

% ===== 方案 G：局部信息素更新 =====
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

tau0 = Q / nnBestCost;
tau = ones(nMid, nMid) * tau0;

% ---- 方案 A：MMAS 信息素限幅 ----
tauMax = 1 / (rho * nnBestCost);
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
    % ---- 方案 O：动态 β ----
    if enableDynamicBeta
        beta = beta_min + (beta_max - beta_min) * (iter / nIter);
    else
        beta = beta_max;
    end

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

            % ---- 方案 K：伪随机比例规则 ----
            candidates = find(~visited);
            nCand = length(candidates);

            % 计算各候选点的选择值 τ^α × η^β
            scores = zeros(1, nCand);
            for c = 1:nCand
                j = candidates(c);
                scores(c) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta);
            end

            if enablePseudoRandom && rand() < q0
                % 贪心：选分值最高的
                [~, bestIdx] = max(scores);
                nxt = candidates(bestIdx);
            else
                % 轮盘赌随机选择
                totalScore = sum(scores);
                if totalScore == 0
                    nxt = candidates(randi(nCand));
                else
                    prob = scores / totalScore;
                    nxt = candidates(find(cumsum(prob) >= rand(), 1, 'first'));
                end
            end

            tour(step) = nxt;
            visited(nxt) = true;
        end

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

    % ---- 方案 F：选择性 2-opt（top ants） + 方案 M：Double-bridge（bottom ants） ----
    [~, sortIdx] = sort(antCosts);

    if enableSelectiveOpt
        % Top optRatio → 2-opt 精炼
        for a_idx = 1:nOptAnts
            a = sortIdx(a_idx);
            [antTours{a}, antCosts(a)] = twoOpt(antTours{a}, midIdx, costMatrix, nPts);
        end
    end

    % 方案 M：Bottom dbRatio → double-bridge 突变（注入多样性，不影响精英）
    if enableDoubleBridge && nMid >= 8
        nDB = max(1, round(nAnts * dbRatio));
        for a_idx = (nAnts - nDB + 1) : nAnts
            a = sortIdx(a_idx);
            [antTours{a}, antCosts(a)] = doubleBridge(antTours{a}, midIdx, costMatrix, nPts);
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

    % --- 全局信息素蒸发 ---
    tau = tau * (1 - rho);

    % --- 信息素沉积 ---
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1)
        i = iterBestTour(k); j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end

    eliteDeposit = Q / globalBestCost * 0.5;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % ---- 方案 A：MMAS 限幅 ----
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

% =========================================================================
%  方案 M：Double-bridge 突变
% =========================================================================
function [tour, cost] = doubleBridge(tour, midIdx, costMatrix, nPts)
%DOUBLEBRIDGE 4-opt 双桥重组：2-opt 无法逆转的空间跳跃
%   将路径切为 4 段 → 重组为 A-rev(C)-rev(B)-D
%   构造性地跳出 2-opt 局部最优陷阱

nMid = length(tour);
if nMid < 8, return; end  % 至少需要 8 个点才能有意义地切 4 段

% 选取 3 个切点，保证每段 ≥ 2 个点
minSeg = 2;
i = randi([minSeg, nMid - 3*minSeg]);
j = randi([i + minSeg, nMid - 2*minSeg]);
k = randi([j + minSeg, nMid - minSeg]);

% 4 个片段
A = tour(1 : i);
B = tour(i+1 : j);
C = tour(j+1 : k);
D = tour(k+1 : end);

% Double-bridge: A + rev(C) + rev(B) + D
newTour = [A, C(end:-1:1), B(end:-1:1), D];

% 重新计算代价
fullOrder = [1, midIdx(newTour), nPts];
cost = 0;
for kk = 1:(nPts - 1)
    cost = cost + costMatrix(fullOrder(kk), fullOrder(kk + 1));
end

tour = newTour;
end
