function [bestOrder, bestCost, history] = TSP_ACO_v1_7_1(costMatrix, nPts)
%TSP_ACO_V1_7_1 蚁群 + 渐进2-opt + 平滑重置 (无 GRASP 初始解)
%   基于 TSP_ACO_v1_7，移除方案 P (GRASP 初始解)，改为均匀信息素初始化。
%   保留:
%     A. MMAS 信息素限幅
%     B. 自适应停止
%     K. 伪随机比例规则 (ACS 风格)
%     M. Double-bridge 突变 (底部蚂蚁)
%     S. 渐进式 2-opt
%     J. 信息素平滑重置
%     T. 精英扰动
%
%   移除:
%     P. GRASP 初始解 — 纯 ACO 构造，不从其他算法借初始解

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法参数 =====
nAnts = 60;
nIter  = 800;
alpha  = 1.0;
beta   = 2.0;
rho    = 0.55;
Q      = 150;
tau0   = 0.1;            % 均匀初始信息素

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 1;
q0 = 0.50;

% ===== 方案 S：渐进式 2-opt =====
optRatio_start = 0.1;
optRatio_end   = 1;
optMidIter     = 100;

% ===== 方案 M：Double-bridge 突变 =====
enableDoubleBridge = 1;
dbRatio = 0.07;

% ===== 方案 J：信息素平滑重置 =====
enableSmoothing = 1;
resetStagThr = 25;
smoothGamma  = 0.40;
maxResets    = 3;

% ===== 方案 T：精英扰动 =====
enableElitePerturb = 1;
eliteStagThr = 15;
eliteTriesMax = 5;

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;
cvThreshold  = 0.001;
stagnationLim = 210;
minIter      = 200;

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

% ---- 均匀信息素初始化（无 GRASP） ----
tau = ones(nMid, nMid) * tau0;

% MMAS 保守初始界（找到第一个解后动态更新）
tauMax = 1 / (rho * tau0 * nMid);
tauMin = tauMax / (5 * nMid);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

globalBestCost = inf;
globalBestTour = midIdx;
stagnationCount = 0;
nResets = 0;
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

    % ---- 方案 B + J + T：自适应停止 + 平滑 + 精英扰动 ----
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

        if enableElitePerturb && stagnationCount > 0 && mod(stagnationCount, eliteStagThr) == 0
            for tryIdx = 1:eliteTriesMax
                [pertTour, ~] = doubleBridge(globalBestTour, midIdx, costMatrix, nPts);
                [pertTour, pertCost] = twoOpt(pertTour, midIdx, costMatrix, nPts);
                if pertCost < globalBestCost - 1e-10
                    globalBestCost = pertCost;
                    globalBestTour = pertTour;
                    stagnationCount = 0;
                    break;
                end
            end
        end

        if enableSmoothing && stagnationCount == resetStagThr && nResets < maxResets
            tau = (1 - smoothGamma) * tau + smoothGamma * tau0;
            tau = min(tau, tauMax);
            tau = max(tau, tauMin);
            stagnationCount = 0;
            nResets = nResets + 1;
        end

        if stagnationCount >= stagnationLim
            if nResets > 0
                stopReason = sprintf('最优停滞(%d代,平滑%d次)', stagnationCount, nResets);
            else
                stopReason = sprintf('最优停滞(%d代未改善)', stagnationCount);
            end
            break;
        end
    end

    tau = tau * (1 - rho);

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
    history.nResets = nResets;
end
end

% =========================================================================
function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
nMid = length(tour); fullOrder = [1, midIdx(tour), nPts];
cost = 0; for k = 1:(nPts-1), cost = cost + costMatrix(fullOrder(k), fullOrder(k+1)); end
improved = true;
while improved
    improved = false;
    for i = 1:(nMid-1), for j = (i+1):nMid
            fi=i+1; fj=j+1;
            old=costMatrix(fullOrder(fi),fullOrder(fi+1))+costMatrix(fullOrder(fj),fullOrder(fj+1));
            nw=costMatrix(fullOrder(fi),fullOrder(fj))+costMatrix(fullOrder(fi+1),fullOrder(fj+1));
            if nw<old-1e-10
                tour((i+1):j)=tour(j:-1:(i+1)); cost=cost-old+nw; improved=true;
                fullOrder=[1,midIdx(tour),nPts];
            end
    end,end
end
end

function [tour, cost] = doubleBridge(tour, midIdx, costMatrix, nPts)
nMid = length(tour);
if nMid < 8, return; end
minSeg = 2;
i = randi([minSeg, nMid-3*minSeg]);
j = randi([i+minSeg, nMid-2*minSeg]);
k = randi([j+minSeg, nMid-minSeg]);
A=tour(1:i); B=tour(i+1:j); C=tour(j+1:k); D=tour(k+1:end);
newTour=[A, C(end:-1:1), B(end:-1:1), D];
fullOrder=[1,midIdx(newTour),nPts];
cost=0; for kk=1:(nPts-1), cost=cost+costMatrix(fullOrder(kk),fullOrder(kk+1)); end
tour=newTour;
end
