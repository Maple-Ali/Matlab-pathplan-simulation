function [bestOrder, bestCost, history] = TSP_ACO_v2(costMatrix, nPts)
%TSP_ACO_V2 蚁群 + VND 局部搜索 求解 TSP
%   基于 TSP_ACO_v1_8，核心升级：
%     VND (变邻域下降) 局部搜索取代单一 2-opt:
%       2-opt → 节点重定位 → 节点交换 → 循环直到全部局部最优
%     组合邻域覆盖 2-opt/Or-opt-1/swap 结构，大幅降低局部最优陷阱
%
%   保留 v1_8 全部机制：S/J/T/K/M/P/A/B

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法基础参数 =====
nAnts = 50;              % 蚂蚁数量
nIter = 800;             % 最大迭代次数
alpha = 1;               % 信息素权重 (τ^alpha)
beta  = 2.0;             % 启发式信息权重 (η^beta)
rho   = 0.25;            % 信息素蒸发率 (0~1, 越大蒸发越快)
Q     = 150;             % 信息素沉积常数 (deposit = Q / cost)

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 0;  % 1=开启伪随机比例规则
q0 = 0.65;               % 贪心选择概率 (0.65=65%贪心选最优边, 35%轮盘赌探索)

% ===== 方案 S：渐进式局部搜索 =====
optRatio_start = 0.1;    % 迭代初期局部搜索比例 (10% 蚂蚁执行 VND)
optRatio_end   = 0.3;   % 迭代后期局部搜索比例 (45% 蚂蚁执行 VND)
optMidIter     = 100;    % 达到 optRatio_end 的迭代数 (线性递增)

% ===== 方案 P：GRASP 初始解 =====
enableGRASP = 0;         % 1=GRASP 构建初始解 + 2-opt 精炼, 0=均匀 tau0=0.1

% ===== 方案 M：Double-bridge 突变 (底部蚂蚁多样性注入) =====
enableDoubleBridge = 0;  % 1=开启
dbRatio = 0.25;          % 底部蚂蚁比例 (25% 最差蚂蚁施加 double-bridge)

% ===== 方案 J：分层信息素重置 =====
enableSmoothing = 0;     % 1=开启分层重置
resetStagThr = 7;        % 每停滞此代数触发一次重置 (7代)
smoothGamma  = 0.40;     % 平滑强度 (0.4=40%回归均匀tau0, 60%保留)
maxSmoothing = 2;        % 平滑次数上限 (超过后触发完全重置)
maxFullResets = 0;       % 完全重置次数上限 (0=不执行完全重置)

% ===== 方案 T：混合精英扰动 (ILS 式盆地跳跃) =====
enableElitePerturb = 0;  % 1=开启精英扰动
eliteStagThr = 5;        % 每停滞此代数触发一次扰动 (5代)
eliteTriesMax = 20;      % 每次扰动最大重试次数

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;  % 1=开启自适应停止
cvThreshold  = 0.001;    % 种群代价变异系数阈值 (CV < 0.001 → 同质收敛停止)
stagnationLim = 80;      % 最优解连续停滞上限 (80代未改善则停止)
minIter      = 60;       % 自适应停止最少迭代代数 (60代后才开始检测)

trackHistory = (nargout >= 3);
if trackHistory, tStart = tic;
    bestCostHistory = zeros(nIter, 1); avgCostHistory = zeros(nIter, 1);
    timeHistory = zeros(nIter, 1);
end

if nMid == 0
    bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, 'avgCostHistory', bestCost, ...
            'timeHistory', 0, 'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
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

% ---- 方案 P：GRASP 初始解 ----
if enableGRASP
    nStarts = min(25, nMid); startSeeds = randperm(nMid, nStarts);
    graspCosts = zeros(nStarts, 1); graspTours = cell(nStarts, 1);
    for sIdx = 1:nStarts
        s = startSeeds(sIdx); visited = false(1, nMid); visited(s) = true;
        tour = zeros(1, nMid); tour(1) = s; cur = s;
        cost = costMatrix(1, midIdx(s));
        for step = 2:nMid
            unvisited = find(~visited); nUnv = length(unvisited); kCand = min(3, nUnv);
            dists = zeros(1, nUnv);
            for j = 1:nUnv, dists(j) = costMatrix(midIdx(cur), midIdx(unvisited(j))); end
            [~, distOrder] = sort(dists); pick = distOrder(randi(kCand)); nextJ = unvisited(pick);
            cost = cost + costMatrix(midIdx(cur), midIdx(nextJ));
            visited(nextJ) = true; tour(step) = nextJ; cur = nextJ;
        end
        cost = cost + costMatrix(midIdx(cur), nPts);
        graspCosts(sIdx) = cost; graspTours{sIdx} = tour;
    end
    [~, bestIdx] = min(graspCosts); nnBestTour = graspTours{bestIdx};
    [nnBestTour, nnBestCost] = vndSearch(nnBestTour, midIdx, costMatrix, nPts);
    tau0 = Q / nnBestCost; tau = ones(nMid, nMid) * tau0;
    tauMax = 1 / (rho * nnBestCost); tauMin = tauMax / nMid;
    tauMin = max(tauMin, 1e-6); tau = min(tau, tauMax); tau = max(tau, tauMin);
    globalBestCost = nnBestCost; globalBestTour = nnBestTour;
else
    tau0 = 0.1; tau = ones(nMid, nMid) * tau0;
    tauMax = 1 / (rho * tau0 * nMid); tauMin = tauMax / (5 * nMid);
    tauMin = max(tauMin, 1e-6); tau = min(tau, tauMax); tau = max(tau, tauMin);
    globalBestCost = inf; globalBestTour = midIdx;
end

stagnationCount = 0; nSmoothing = 0; nFullResets = 0; stopReason = '';

for iter = 1:nIter
    optRatio = optRatio_start + (optRatio_end - optRatio_start) * min(1, iter / optMidIter);
    nOptAnts = max(1, round(nAnts * optRatio));
    antTours = cell(nAnts, 1); antCosts = zeros(nAnts, 1);

    for a = 1:nAnts
        tour = zeros(1, nMid); visited = false(1, nMid);
        start = randi(nMid); tour(1) = start; visited(start) = true;
        for step = 2:nMid
            curIdx = tour(step - 1); candidates = find(~visited); nCand = length(candidates);
            scores = zeros(1, nCand);
            for c = 1:nCand, j = candidates(c);
                scores(c) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta);
            end
            if enablePseudoRandom && rand() < q0
                [~, bestScIdx] = max(scores); nxt = candidates(bestScIdx);
            else
                totalScore = sum(scores);
                if totalScore == 0, nxt = candidates(randi(nCand));
                else, prob = scores / totalScore;
                    nxt = candidates(find(cumsum(prob) >= rand(), 1, 'first'));
                end
            end
            tour(step) = nxt; visited(nxt) = true;
        end
        c = tourCost(tour, midIdx, costMatrix, nPts);
        antTours{a} = tour; antCosts(a) = c;
    end

    % ---- VND 局部搜索 + Double-bridge ----
    [~, sortIdx] = sort(antCosts);
    for a_idx = 1:nOptAnts
        a = sortIdx(a_idx);
        [antTours{a}, antCosts(a)] = vndSearch(antTours{a}, midIdx, costMatrix, nPts);
    end
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
        globalBestCost = iterBestCost; globalBestTour = iterBestTour; improved = true;
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts); timeHistory(iter) = toc(tStart);
    end

    % ---- 方案 B + J + T ----
    if enableAdaptiveStop && iter >= minIter
        cv = std(antCosts) / mean(antCosts);
        if cv < cvThreshold
            stopReason = sprintf('种群同质(CV=%.4f<%g)', cv, cvThreshold); break;
        end
        if improved, stagnationCount = 0;
        else, stagnationCount = stagnationCount + 1; end

        if enableElitePerturb && stagnationCount > 0 && mod(stagnationCount, eliteStagThr) == 0
            for tryIdx = 1:eliteTriesMax
                if rand < 0.5
                    [pertTour, ~] = doubleBridge(globalBestTour, midIdx, costMatrix, nPts);
                else
                    permIdx = randperm(nMid, min(5, nMid)); pertTour = globalBestTour;
                    pertTour(permIdx) = pertTour(permIdx(randperm(length(permIdx))));
                end
                [pertTour, pertCost] = vndSearch(pertTour, midIdx, costMatrix, nPts);
                if pertCost < globalBestCost - 1e-10
                    globalBestCost = pertCost; globalBestTour = pertTour;
                    stagnationCount = 0; break;
                end
            end
        end

        if enableSmoothing && stagnationCount > 0 && mod(stagnationCount, resetStagThr) == 0
            if nSmoothing < maxSmoothing
                tau = (1 - smoothGamma) * tau + smoothGamma * tau0; nSmoothing = nSmoothing + 1;
            elseif nFullResets < maxFullResets
                tau = ones(nMid, nMid) * tau0; nSmoothing = 0; nFullResets = nFullResets + 1;
                eDep = Q / globalBestCost;
                for k = 1:(length(globalBestTour) - 1), i = globalBestTour(k); j = globalBestTour(k + 1);
                    tau(i, j) = tau(i, j) + eDep; tau(j, i) = tau(j, i) + eDep;
                end
            end
            tau = min(tau, tauMax); tau = max(tau, tauMin);
        end

        if stagnationCount >= stagnationLim
            stopReason = sprintf('最优停滞(%d,平滑%d,重置%d)', stagnationCount, nSmoothing, nFullResets); break;
        end
    end

    tau = tau * (1 - rho);
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1), i = iterBestTour(k); j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit; tau(j, i) = tau(j, i) + deposit;
    end
    eliteDeposit = Q / globalBestCost;
    for k = 1:(length(globalBestTour) - 1), i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit; tau(j, i) = tau(j, i) + eliteDeposit;
    end
    tau = min(tau, tauMax); tau = max(tau, tauMin);
    tauMax = 1 / (rho * globalBestCost); tauMin = tauMax / nMid; tauMin = max(tauMin, 1e-6);
end

bestCost = globalBestCost; bestOrder = [1, midIdx(globalBestTour), nPts]; actualIter = iter;

if trackHistory
    history = struct(); history.bestCostHistory = bestCostHistory(1:actualIter);
    history.avgCostHistory = avgCostHistory(1:actualIter);
    history.timeHistory = timeHistory(1:actualIter);
    history.iterCount = actualIter; history.elapsedTime = toc(tStart);
    if isempty(stopReason), stopReason = 'maxIter'; end
    history.stopReason = stopReason; history.nSmoothing = nSmoothing; history.nFullResets = nFullResets;
end
end

% =========================================================================
%  VND 局部搜索: 2-opt → 重定位 → 交换 → 循环
% =========================================================================
function [tour, cost] = vndSearch(tour, midIdx, costMatrix, nPts)
cost = tourCost(tour, midIdx, costMatrix, nPts);
improved = true;
while improved
    improved = false;
    % 1: 2-opt first-improvement
    [tour, cost, ok] = twoOptFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
    % 2: 节点重定位
    [tour, cost, ok] = relocateFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
    % 3: 节点交换
    [tour, cost, ok] = swapFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
end
end

function c = tourCost(tour, midIdx, costMatrix, nPts)
fullOrder = [1, midIdx(tour), nPts]; c = 0;
for k = 1:(nPts - 1), c = c + costMatrix(fullOrder(k), fullOrder(k + 1)); end
end

% ---- 2-opt (first-improvement) ----
function [tour, cost, improved] = twoOptFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];
improved = false;
for i = 1:(nMid - 1)
    for j = (i + 1):nMid
        fi = i + 1; fj = j + 1;
        old = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + costMatrix(fullOrder(fj), fullOrder(fj + 1));
        nw  = costMatrix(fullOrder(fi), fullOrder(fj)) + costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
        if nw < old - 1e-10
            tour((i + 1):j) = tour(j:-1:(i + 1));
            cost = cost - old + nw; improved = true;
            fullOrder = [1, midIdx(tour), nPts];
        end
    end
end
end

% ---- 节点重定位 (first-improvement) ----
function [tour, cost, improved] = relocateFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour);
improved = false;
% 用实际索引简化计算：Lv/Rv/Lp/Rp 都是 costMatrix 实际索引
for v = 1:nMid
    Cv = midIdx(tour(v));
    if v == 1, Lv = 1; else, Lv = midIdx(tour(v - 1)); end
    if v == nMid, Rv = nPts; else, Rv = midIdx(tour(v + 1)); end

    for p = 0:nMid
        if p == v || p == v - 1, continue; end
        if p == 0, Lp = 1; Rp = midIdx(tour(1));
        elseif p == nMid, Lp = midIdx(tour(nMid)); Rp = nPts;
        else, Lp = midIdx(tour(p)); Rp = midIdx(tour(p + 1));
        end
        oldCost = costMatrix(Lv, Cv) + costMatrix(Cv, Rv) + costMatrix(Lp, Rp);
        newCost = costMatrix(Lv, Rv) + costMatrix(Lp, Cv) + costMatrix(Cv, Rp);
        if newCost < oldCost - 1e-10
            cityVal = tour(v);
            if p < v
                newTour = [tour(1:p), cityVal, tour(p+1:v-1), tour(v+1:end)];
            else
                newTour = [tour(1:v-1), tour(v+1:p), cityVal, tour(p+1:end)];
            end
            tour = newTour; cost = cost - oldCost + newCost; improved = true;
            return;
        end
    end
end
end

% ---- 节点交换 (first-improvement) ----
function [tour, cost, improved] = swapFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];
improved = false;
for i = 1:(nMid - 1)
    for j = (i + 1):nMid
        fi = i + 1; fj = j + 1;
        Li = fullOrder(fi - 1); Ai = fullOrder(fi); Ri = fullOrder(fi + 1);
        Lj = fullOrder(fj - 1); Aj = fullOrder(fj); Rj = fullOrder(fj + 1);
        if j == i + 1  % 相邻
            oldCost = costMatrix(Li, Ai) + costMatrix(Ai, Aj) + costMatrix(Aj, Rj);
            newCost = costMatrix(Li, Aj) + costMatrix(Aj, Ai) + costMatrix(Ai, Rj);
        else
            oldCost = costMatrix(Li, Ai) + costMatrix(Ai, Ri) + costMatrix(Lj, Aj) + costMatrix(Aj, Rj);
            newCost = costMatrix(Li, Aj) + costMatrix(Aj, Ri) + costMatrix(Lj, Ai) + costMatrix(Ai, Rj);
        end
        if newCost < oldCost - 1e-10
            tour([i, j]) = tour([j, i]);
            cost = cost - oldCost + newCost; improved = true;
            fullOrder = [1, midIdx(tour), nPts];
        end
    end
end
end

% =========================================================================
function [tour, cost] = doubleBridge(tour, midIdx, costMatrix, nPts)
nMid = length(tour);
if nMid < 8, return; end
minSeg = 2;
i = randi([minSeg, nMid - 3*minSeg]);
j = randi([i + minSeg, nMid - 2*minSeg]);
k = randi([j + minSeg, nMid - minSeg]);
A = tour(1 : i); B = tour(i+1 : j); C = tour(j+1 : k); D = tour(k+1 : end);
newTour = [A, C(end:-1:1), B(end:-1:1), D];
cost = tourCost(newTour, midIdx, costMatrix, nPts);
tour = newTour;
end
