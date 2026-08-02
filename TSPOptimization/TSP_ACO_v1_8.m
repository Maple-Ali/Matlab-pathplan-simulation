function [bestOrder, bestCost, history] = TSP_ACO_v1_8(costMatrix, nPts)
%TSP_ACO_V1_8 蚁群 + 分层信息素重置 + 混合精英扰动 求解 TSP
%   基于 TSP_ACO_v1_7，改进:
%     J. 分层信息素重置: 平滑重置 → 完全重置→全局最优引导 (三级)
%     T. 混合精英扰动: 50% Double-bridge + 50% 随机5城市打乱 + 2-opt
%
%   保留 v1_7:
%     A. MMAS 信息素限幅
%     B. 自适应停止
%     K. 伪随机比例规则 (ACS 风格)
%     M. Double-bridge 突变 (底部蚂蚁)
%     P. GRASP 初始解
%     S. 渐进式 2-opt

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 50;
nIter  = 800;
alpha  = 1;
beta   = 2.0;
rho    = 0.25;
Q      = 150;

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 1;
q0 = 0.65;

% ===== 方案 S：渐进式 2-opt =====
optRatio_start = 0.1;
optRatio_end   = 0.45;
optMidIter     = 150;

% ===== 方案 P：GRASP 初始解 =====
enableGRASP = 0;         % 1=GRASP 初始解, 0=均匀 tau0（仅靠 ACO 自身）

% ===== 方案 M：Double-bridge 突变 =====
enableDoubleBridge = 0;
dbRatio = 0.25;

% ===== 方案 J：分层信息素重置 =====
enableSmoothing = 0;     % 1=停滞时触发分层重置
resetStagThr = 7;       % 连续停滞此代数时触发一次重置
smoothGamma  = 0.40;     % 平滑强度（平滑阶段用）
maxSmoothing = 2;        % 平滑次数上限（超过后触发完全重置）
maxFullResets = 0;       % 完全重置次数上限

% ===== 方案 T：混合精英扰动 =====
enableElitePerturb = 0;
eliteStagThr = 5;       % 停滞此代数时触发一次扰动
eliteTriesMax = 20;      % 每次扰动最大尝试次数

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;
cvThreshold  = 0.001;
stagnationLim = 80;      % 最优解连续停滞上限（代）
minIter      = 60;

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

% ---- 方案 P：GRASP 初始解 (仅 enableGRASP=1) ----
if enableGRASP
    nStarts = min(25, nMid);
    startSeeds = randperm(nMid, nStarts);
    graspCosts = zeros(nStarts, 1);
    graspTours = cell(nStarts, 1);
    for sIdx = 1:nStarts
        s = startSeeds(sIdx);
        visited = false(1, nMid); visited(s) = true;
        tour = zeros(1, nMid); tour(1) = s; cur = s;
        cost = costMatrix(1, midIdx(s));
        for step = 2:nMid
            unvisited = find(~visited);
            nUnv = length(unvisited);
            kCand = min(3, nUnv);
            dists = zeros(1, nUnv);
            for j = 1:nUnv
                dists(j) = costMatrix(midIdx(cur), midIdx(unvisited(j)));
            end
            [~, distOrder] = sort(dists);
            pick = distOrder(randi(kCand));
            nextJ = unvisited(pick);
            cost = cost + costMatrix(midIdx(cur), midIdx(nextJ));
            visited(nextJ) = true;
            tour(step) = nextJ; cur = nextJ;
        end
        cost = cost + costMatrix(midIdx(cur), nPts);
        graspCosts(sIdx) = cost; graspTours{sIdx} = tour;
    end
    [~, bestIdx] = min(graspCosts);
    nnBestTour = graspTours{bestIdx};
    [nnBestTour, nnBestCost] = twoOpt(nnBestTour, midIdx, costMatrix, nPts);

    tau0 = Q / nnBestCost;
    tau = ones(nMid, nMid) * tau0;

    % ---- 方案 A：MMAS 信息素限幅 ----
    tauMax = 1 / (rho * nnBestCost);
    tauMin = tauMax / (nMid);
    tauMin = max(tauMin, 1e-6);
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);

    globalBestCost = nnBestCost;
    globalBestTour = nnBestTour;
else
    % 纯 ACO：均匀初始信息素，无外部启发式引导
    tau0 = 0.1;
    tau = ones(nMid, nMid) * tau0;

    tauMax = 1 / (rho * tau0 * nMid);  % 保守上界
    tauMin = tauMax / (5 * nMid);
    tauMin = max(tauMin, 1e-6);
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);

    globalBestCost = inf;
    globalBestTour = midIdx;
end
stagnationCount = 0;

% ---- 方案 J：分层重置计数器 ----
nSmoothing  = 0;     % 当前已平滑次数（归零后触发完全重置）
nFullResets = 0;     % 已执行完全重置次数

stopReason = '';

for iter = 1:nIter
    % ---- 方案 S：渐进式 2-opt 比例 ----
    optRatio = optRatio_start + (optRatio_end - optRatio_start) * min(1, iter / optMidIter);
    nOptAnts = max(1, round(nAnts * optRatio));

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
            candidates = find(~visited);
            nCand = length(candidates);

            scores = zeros(1, nCand);
            for c = 1:nCand
                j = candidates(c);
                scores(c) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta);
            end

            if enablePseudoRandom && rand() < q0
                [~, bestScIdx] = max(scores);
                nxt = candidates(bestScIdx);
            else
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

        fullOrder = [1, midIdx(tour), nPts];
        c = 0;
        for k = 1:(nPts - 1)
            c = c + costMatrix(fullOrder(k), fullOrder(k + 1));
        end
        antTours{a} = tour;
        antCosts(a) = c;
    end

    % ---- 方案 S：渐进式 2-opt + 方案 M：Double-bridge ----
    [~, sortIdx] = sort(antCosts);

    for a_idx = 1:nOptAnts
        a = sortIdx(a_idx);
        [antTours{a}, antCosts(a)] = twoOpt(antTours{a}, midIdx, costMatrix, nPts);
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
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
        improved = true;
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % ---- 方案 B + J + T：自适应停止 + 分层重置 + 精英扰动 ----
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
        end

        % ---- 方案 T：混合精英扰动 ----
        if enableElitePerturb && stagnationCount > 0 && mod(stagnationCount, eliteStagThr) == 0
            for tryIdx = 1:eliteTriesMax
                % 混合扰动: 50% double-bridge, 50% 随机5城市打乱
                if rand < 0.5
                    [pertTour, ~] = doubleBridge(globalBestTour, midIdx, costMatrix, nPts);
                else
                    permIdx = randperm(nMid, min(5, nMid));
                    pertTour = globalBestTour;
                    pertTour(permIdx) = pertTour(permIdx(randperm(length(permIdx))));
                end
                [pertTour, pertCost] = twoOpt(pertTour, midIdx, costMatrix, nPts);
                if pertCost < globalBestCost - 1e-10
                    globalBestCost = pertCost;
                    globalBestTour = pertTour;
                    stagnationCount = 0;
                    break;
                end
            end
        end

        % ---- 方案 J：分层信息素重置 (每 resetStagThr 代触发) ----
        if enableSmoothing && stagnationCount > 0 && mod(stagnationCount, resetStagThr) == 0
            if nSmoothing < maxSmoothing
                % 平滑重置（保留部分记忆）
                tau = (1 - smoothGamma) * tau + smoothGamma * tau0;
                nSmoothing = nSmoothing + 1;
            elseif nFullResets < maxFullResets
                % 完全重置：信息素回到初始均匀 + 全局最优引导
                tau = ones(nMid, nMid) * tau0;
                nFullResets = nFullResets + 1;
                % 将全局最优沉积到初始信息素上，加速重新收敛
                eDep = Q / globalBestCost;
                for k = 1:(length(globalBestTour) - 1)
                    i = globalBestTour(k); j = globalBestTour(k + 1);
                    tau(i, j) = tau(i, j) + eDep;
                    tau(j, i) = tau(j, i) + eDep;
                end
            end
            tau = min(tau, tauMax);
            tau = max(tau, tauMin);
        end

        if stagnationCount >= stagnationLim
            stopReason = sprintf('最优停滞(%d代,平滑%d,全重置%d)', stagnationCount, nSmoothing, nFullResets);
            break;
        end
    end

    % --- 全局信息素蒸发 ---
    tau = tau * (1 - rho);

    % ---- 信息素沉积 ----
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1)
        i = iterBestTour(k); j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end

    eliteDeposit = Q / globalBestCost;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % ---- 方案 A：MMAS 限幅 ----
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);
    tauMax = 1 / (rho * globalBestCost);
    tauMin = tauMax / (nMid);
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
    history.nSmoothing  = nSmoothing;
    history.nFullResets = nFullResets;
end
end

% =========================================================================
%  2-opt 局部搜索
% =========================================================================
function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];
cost = 0;
for k = 1:(nPts - 1), cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1)); end
improved = true;
while improved
    improved = false;
    for i = 1:(nMid - 1)
        for j = (i + 1):nMid
            fi = i + 1; fj = j + 1;
            old = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + ...
                  costMatrix(fullOrder(fj), fullOrder(fj + 1));
            nw  = costMatrix(fullOrder(fi), fullOrder(fj)) + ...
                  costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
            if nw < old - 1e-10
                tour((i + 1):j) = tour(j:-1:(i + 1));
                cost = cost - old + nw; improved = true;
                fullOrder = [1, midIdx(tour), nPts];
            end
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
fullOrder = [1, midIdx(newTour), nPts];
cost = 0;
for kk = 1:(nPts - 1), cost = cost + costMatrix(fullOrder(kk), fullOrder(kk + 1)); end
tour = newTour;
end
