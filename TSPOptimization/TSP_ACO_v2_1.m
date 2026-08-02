function [bestOrder, bestCost, history] = TSP_ACO_v2_1(costMatrix, nPts)
%TSP_ACO_V2_1 蚁群 + VND 局部搜索 (精简版) 求解 TSP
%   基于 TSP_ACO_v2，保留核心机制:
%     VND 局部搜索: 2-opt → 节点重定位 → 节点交换 → 循环
%     K. 伪随机比例规则
%     S. 渐进式局部搜索
%     M. Double-bridge 突变
%     A. MMAS 信息素限幅
%     B. 自适应停止
%
%   移除:
%     P. GRASP 初始解 (始终均匀 tau0=0.1)
%     J. 分层信息素重置
%     T. 混合精英扰动

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法基础参数 =====
nAnts = 50;              % 蚂蚁数量
nIter = 800;             % 最大迭代次数
alpha = 1;               % 信息素权重
beta  = 2.0;             % 启发式信息权重
rho   = 0.25;            % 信息素蒸发率
Q     = 150;             % 信息素沉积常数

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 1;  % 1=开启
q0 = 0.25;               % 贪心选择概率

% ===== 方案 S：渐进式局部搜索 (S 曲线) =====
optRatio_start = 0.00;       % 初期局部搜索比例 (y_min)
optRatio_end   = 0.30;       % 后期局部搜索比例 (y_max)
optTransInterval = [0, 100]; % 过渡区间 [x_L, x_R] (S曲线从 y_min 升到 y_max 的迭代区间)
optCurveA      = 2.0;        % S 曲线陡峭度 (a>1=先缓后陡, a=1=线性, a<1=先陡后缓)

% ===== 方案 M：Double-bridge 突变 =====
enableDoubleBridge = 1;  % 1=开启
dbRatio = 0.25;          % 底部蚂蚁比例

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;  % 1=开启
cvThreshold  = 0.001;    % 种群 CV 阈值
stagnationLim = 50;      % 停滞上限 (代)
minIter      = 30;       % 最少迭代代数

trackHistory = (nargout >= 3);
if trackHistory, tStart = tic;
    bestCostHistory = zeros(nIter, 1);
    avgCostHistory = zeros(nIter, 1);
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

% ---- 均匀初始信息素 (无外部启发式) ----
tau0 = 0.1;
tau = ones(nMid, nMid) * tau0;

% ---- 方案 A：MMAS 初始信息素限幅 (保守) ----
tauMax = 1 / (rho * tau0 * nMid);
tauMin = tauMax / (5 * nMid);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

globalBestCost = inf;
globalBestTour = midIdx;
stagnationCount = 0;
stopReason = '';

for iter = 1:nIter
    % ---- 方案 S：渐进式局部搜索比例 (S 曲线) ----
    xL = optTransInterval(1); xR = optTransInterval(2);
    if iter <= xL
        optRatio = optRatio_start;
    elseif iter >= xR
        optRatio = optRatio_end;
    else
        t = (iter - xL) / (xR - xL);
        optRatio = optRatio_start + (optRatio_end - optRatio_start) * (t^optCurveA / (t^optCurveA + (1-t)^optCurveA));
    end
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

        antTours{a} = tour;
        antCosts(a) = tourCost(tour, midIdx, costMatrix, nPts);
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
            antTours{a} = doubleBridge(antTours{a}, midIdx, costMatrix, nPts);
            antCosts(a) = tourCost(antTours{a}, midIdx, costMatrix, nPts);
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
                stopReason = sprintf('最优停滞(%d代)', stagnationCount);
                break;
            end
        end
    end

    % --- 信息素蒸发 ---
    tau = tau * (1 - rho);

    % --- 信息素沉积 ---
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

    % ---- 方案 A：MMAS 限幅 + 动态更新 ----
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);
    tauMax = 1 / (rho * globalBestCost);
    tauMin = tauMax / nMid;
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
%  VND 局部搜索: 2-opt → 重定位 → 交换 → 循环
% =========================================================================
function [tour, cost] = vndSearch(tour, midIdx, costMatrix, nPts)
cost = tourCost(tour, midIdx, costMatrix, nPts);  % 初始精确成本
improved = true;
while improved
    improved = false;
    [tour, cost, ok] = twoOptFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
    [tour, cost, ok] = relocateFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
    [tour, cost, ok] = swapFI(tour, midIdx, costMatrix, nPts, cost);
    improved = improved || ok;
end
cost = tourCost(tour, midIdx, costMatrix, nPts);  % 结束时全路径校准，消除累积偏差
end

function c = tourCost(tour, midIdx, costMatrix, nPts)
fo = [1, midIdx(tour), nPts]; c = 0;
for k = 1:(nPts - 1), c = c + costMatrix(fo(k), fo(k + 1)); end
end

% ---- 2-opt (first-improvement, 单遍扫描) ----
function [tour, cost, improved] = twoOptFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];
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

% ---- 节点重定位 (first-improvement) ----
function [tour, cost, improved] = relocateFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour); improved = false;
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
            tour = newTour; cost = cost - oldCost + newCost; improved = true; return;
        end
    end
end
end

% ---- 节点交换 (first-improvement) ----
function [tour, cost, improved] = swapFI(tour, midIdx, costMatrix, nPts, cost)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts]; improved = false;
for i = 1:(nMid - 1)
    for j = (i + 1):nMid
        fi = i + 1; fj = j + 1;
        Li = fullOrder(fi - 1); Ai = fullOrder(fi); Ri = fullOrder(fi + 1);
        Lj = fullOrder(fj - 1); Aj = fullOrder(fj); Rj = fullOrder(fj + 1);
        if j == i + 1
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
function tour = doubleBridge(tour, ~, ~, ~)
nMid = length(tour);
if nMid < 8, return; end
minSeg = 2;
i = randi([minSeg, nMid - 3*minSeg]);
j = randi([i + minSeg, nMid - 2*minSeg]);
k = randi([j + minSeg, nMid - minSeg]);
A = tour(1 : i); B = tour(i+1 : j); C = tour(j+1 : k); D = tour(k+1 : end);
tour = [A, C(end:-1:1), B(end:-1:1), D];
end
