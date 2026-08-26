function [bestOrder, bestCost, history] = TSP_SA_v0_1_X(costMatrix, nPts)
%TSP_SA_V0_1_X 自适应模拟退火算法求解 TSP（虚拟节点法：起点终点固定 + 中间点全排列优化）
%   在原 TSP_SA_v0_1 基础上，将起点终点固定问题通过虚拟节点转化为闭合环路 TSP。
%   虚拟节点 V 与起点(1)、终点(nPts) 距离为 0，与中间点距离 BIG。
%   求解闭合环路后，去掉虚拟节点并校正方向，得到 起点→中间点→终点 的开路径。
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost, history

nNodes = nPts;  % 闭合环路: 排列 1:nPts, 虚拟节点隐含

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
midIdx = 2:(nPts - 1);
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

if nNodes <= 2
    bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% ---- 随机初始解 (闭合环路排列) ----
midOrder = randperm(nNodes);

curCost = calcCost(midOrder, costExt, nNodes);
bestMid = midOrder; bestCost = curCost;

% ---- SA 参数 (用原 costMatrix 避免 BIG 值影响 T0) ----
if T0_factor > 0
    T0 = T0_factor * max(costMatrix(costMatrix < inf & costMatrix > 0));
else
    T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
end
if isempty(T0) || T0 == 0, T0 = 100; end
T = T0;
nInnerBase = max(nNodes * nInnerMult, 50);

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
        % 单一邻域操作: swap (闭合环路, 交换任意两个位置)
        pair = sort(randperm(nNodes, 2));
        i1 = pair(1); i2 = pair(2);
        [newCost, delta] = deltaSwap(curMid, i1, i2, curCost, costExt, nNodes);

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

% 提取开路径: 去掉虚拟节点, 校正方向, 返回开路径成本
bestOrder = extractPath(bestMid, nPts, Vidx);
bestCost = bestCost - costExt(bestMid(nNodes), bestMid(1));  % 减去闭合边

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
function c = calcCost(order, costExt, n)
% 闭合环路成本
c = 0;
for k = 1:n
    c = c + costExt(order(k), order(mod(k, n) + 1));
end
end

function [nc, d] = deltaSwap(o, i, j, cur, cm, n)
% 闭合环路 swap 邻域的增量成本 (闭合: pos n 与 pos 1 相邻)
Ai = o(i);  Aj = o(j);
Li = o(mod(i - 2, n) + 1);  Ri = o(mod(i, n) + 1);
Lj = o(mod(j - 2, n) + 1);  Rj = o(mod(j, n) + 1);
if i == 1 && j == n
    % 首尾相邻 (闭合环路的跨越边): 仅替换两条边
    old = cm(Aj, Ai) + cm(Ai, Rj);
    nw  = cm(Ai, Aj) + cm(Aj, Rj);
elseif j == i + 1
    old = cm(Li, Ai) + cm(Ai, Aj) + cm(Aj, Rj);
    nw  = cm(Li, Aj) + cm(Aj, Ai) + cm(Ai, Rj);
else
    old = cm(Li, Ai) + cm(Ai, Ri) + cm(Lj, Aj) + cm(Aj, Rj);
    nw  = cm(Li, Aj) + cm(Aj, Ri) + cm(Lj, Ai) + cm(Ai, Rj);
end
d = nw - old; nc = cur + d;
end

function order = extractPath(cycle, nPts, Vidx)
n = length(cycle);
vPos = find(cycle == Vidx, 1);
if isempty(vPos)
    order = cycle;
else
    order = zeros(1, nPts);
    idx = mod(vPos, n) + 1;
    for t = 1:nPts
        order(t) = cycle(idx);
        idx = mod(idx, n) + 1;
    end
end
if order(1) == nPts
    order = fliplr(order);
end
if order(1) ~= 1
    p1 = find(order == 1, 1);
    order = [order(p1:end), order(1:p1-1)];
end
end
