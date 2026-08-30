function [bestOrder, bestCost, history] = TSP_SA_v0_1_X(costMatrix, nPts)
%TSP_SA_V0_1_X 自适应模拟退火算法求解 TSP（虚拟节点法：起点终点固定 + 中间点全排列优化）
%   在原 TSP_SA_v0_1 基础上，将起点终点固定问题通过虚拟节点转化为闭合环路 TSP。
%   起点(1)和终点(nPts)固定在首尾，仅排列中间点。
%   虚拟节点 V 与起点(1)、终点(nPts) 距离为 0，与中间点距离 BIG。
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost (开路径成本), history

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% 扩展成本矩阵: 添加虚拟节点 V = nPts+1
Vidx = nPts + 1;
nExt = nPts + 1;
maxFinite = max(costMatrix(costMatrix < inf));
if isempty(maxFinite), maxFinite = 1; end
BIG = (nPts + 1) * maxFinite * 2;

costExt = zeros(nExt);
costExt(1:nPts, 1:nPts) = costMatrix;
costExt(Vidx, 1) = 0;  costExt(1, Vidx) = 0;
costExt(Vidx, nPts) = 0;  costExt(nPts, Vidx) = 0;
costExt(Vidx, midIdx) = BIG;  costExt(midIdx, Vidx) = BIG;

% ===== 可调参数 =====
T0_factor  = 5.0;
alpha_base = 0.998;
alpha_min  = 0.996;
alpha_max  = 0.9995;
alpha_beta = 0.005;
T_min      = 1e-50;

adaptInner  = 1;
nInnerMult  = 10;
nInnerMin   = 55;
% ====================

trackHistory = (nargout >= 3);
if trackHistory, tStart = tic; end

if nMid == 0
    bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% ---- 随机初始解 (仅排列中间点) ----
midOrder = midIdx(randperm(nMid));

curCost = calcCost(midOrder, costExt, nPts, nMid);
bestMid = midOrder; bestCost = curCost;

% ---- SA 参数 (用原 costMatrix 避免 BIG 值影响 T0) ----
if T0_factor > 0
    T0 = T0_factor * max(costMatrix(costMatrix < inf & costMatrix > 0));
else
    T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
end
if isempty(T0) || T0 == 0, T0 = 100; end
T = T0;
nInnerBase = max(nMid * nInnerMult, 50);

nEst = ceil(log(T_min / T0) / log(alpha_min)) + 200;
if trackHistory
    bestCostHistory = zeros(nEst, 1);
    avgCostHistory = zeros(nEst, 1);
    timeHistory = zeros(nEst, 1);
end
nOuter = 0;

% ---- SA 主循环 ----
while T > T_min
    nOuter = nOuter + 1;
    sumCost = 0; nAccepted = 0;
    curMid = midOrder;

    % ---- 自适应内循环 ----
    if adaptInner
        nInner = max(nInnerMin, round(nInnerBase * sqrt(T / T0)));
    else
        nInner = nInnerBase;
    end

    for i = 1:nInner
        % 单一邻域操作: swap (中间点位置, 1~nMid)
        pair = sort(randperm(nMid, 2));
        i1 = pair(1); i2 = pair(2);
        [newCost, delta] = deltaSwap(curMid, i1, i2, curCost, costExt, nMid);

        if delta < 0 || rand < exp(-delta / T)
            curMid([i1, i2]) = curMid([i2, i1]);
            curCost = newCost; nAccepted = nAccepted + 1;
        end
        sumCost = sumCost + curCost;

        if curCost < bestCost
            bestCost = curCost; bestMid = curMid;
        end
    end

    midOrder = curMid;

    if trackHistory
        bestCostHistory(nOuter) = bestCost;
        avgCostHistory(nOuter) = sumCost / nInner;
        timeHistory(nOuter) = toc(tStart);
    end

    % ---- 自适应冷却 ----
    acptRate = nAccepted / nInner;
    alpha = alpha_base - alpha_beta * (acptRate - 0.5);
    alpha = max(alpha_min, min(alpha_max, alpha));
    T = T * alpha;
end

% 输出: bestCost 已是开路径成本 (calcCost 已减去闭合边)
bestOrder = [1, bestMid, nPts];

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory(1:nOuter);
    history.avgCostHistory = avgCostHistory(1:nOuter);
    history.timeHistory = timeHistory(1:nOuter);
    history.iterCount = nOuter;
    history.elapsedTime = toc(tStart);
end
end

% =========================================================================
function c = calcCost(midOrder, costExt, nPts, nMid)
% 开路径成本: start→midOrder(1)→...→midOrder(nMid)→goal
c = costExt(1, midOrder(1));
for k = 1:(nMid - 1)
    c = c + costExt(midOrder(k), midOrder(k + 1));
end
c = c + costExt(midOrder(nMid), nPts);
end

function [nc, d] = deltaSwap(o, i, j, cur, cm, nMid)
% 开路径 swap 邻域的增量成本 (起点/终点固定在首尾之外)
Ai = o(i);  Aj = o(j);
if j == i + 1
    if i == 1, Li = 1;    else, Li = o(i - 1); end
    if j == nMid, Rj = nMid + 2; else, Rj = o(j + 1); end
    old = cm(Li, Ai) + cm(Ai, Aj) + cm(Aj, Rj);
    nw  = cm(Li, Aj) + cm(Aj, Ai) + cm(Ai, Rj);
else
    if i == 1, Li = 1;    else, Li = o(i - 1); end
    Ri = o(i + 1);
    Lj = o(j - 1);
    if j == nMid, Rj = nMid + 2; else, Rj = o(j + 1); end
    old = cm(Li, Ai) + cm(Ai, Ri) + cm(Lj, Aj) + cm(Aj, Rj);
    nw  = cm(Li, Aj) + cm(Aj, Ri) + cm(Lj, Ai) + cm(Ai, Rj);
end
d = nw - old; nc = cur + d;
end
