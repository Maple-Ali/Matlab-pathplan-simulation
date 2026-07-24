function [bestOrder, bestCost, history] = TSP_SA_v2(costMatrix, nPts)
%TSP_SA_V2 改进模拟退火算法求解 TSP（增强局部搜索）
%   相对 TSP_SA_v1_1 的新增:
%     F. 增强 2-opt 嵌入: 发现更好解时立即精炼 + 低温阶段周期性触发
%     G. Or-opt 算子: 移动 2~3 点连续片段，O(1) 增量评价
%
%   保留:
%     A. 多点最近邻初始解 + 2-opt
%     B. 初始解 + 最终解 2-opt 精炼
%     C. 增量代价计算 delta O(1)（swap/insert/2-opt-reverse/or-opt）
%     D. 自适应算子权重（4 算子）
%     E. 自适应冷却 + 自适应内循环 + 基于温度的停滞检测
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 关键可调参数 =====================================================
% --- 初始解 ---
nNNRestarts = 0;            % 多点NN数量（0=尝试全部nMid个起点）

% --- 自适应冷却速率 ---
alpha_base   = 0.995;       % 基础冷却系数
adaptAlpha   = 1;           % 1=自适应冷却, 0=固定 alpha=alpha_base
alpha_min    = 0.990;       % 冷却系数下限
alpha_max    = 0.998;       % 冷却系数上限
alpha_beta   = 0.005;       % 自适应强度
targetAcpt   = 0.50;        % 目标接受率

% --- 内循环自适应 ---
adaptInner   = 1;           % 1=内循环随温度自适应, 0=固定
nInnerMult   = 10;          % 内循环基数
nInnerMin    = 30;          % 内循环最小长度
nInnerPower  = 0.5;         % 温度权重指数

% --- 温度 ---
T0_factor    = 1.0;
T_min        = 1e-3;

% --- 邻域算子权重（新增 Or-opt 为第4算子） ---
opWeights    = [1.0, 1.0, 1.0, 0.8];  % [swap, insert, 2optRev, oropt]
opLearnRate  = 0.15;
opDecay      = 0.95;

% --- 方案 F：增强 2-opt 嵌入 ---
enableOptOnImprove = 1;     % 1=发现全局更优时立即 2-opt 精炼
enableOptPeriodic  = 1;     % 1=低温阶段周期性 2-opt
optInterval  = 40;          % 周期性 2-opt 间隔（外循环轮数）

% --- 方案 G：Or-opt 算子 ---
enableOropt  = 1;           % 1=启用 Or-opt 算子
oroptLen     = [2, 3];      % 片段长度随机取 2 或 3

% --- 自适应停止 ---
enableAdaptiveStop = 1;
T_cold_factor = 0.02;
stagnationLim = 250;
minOuter      = 80;
maxOuterIter  = 3000;
% ========================================================================

trackHistory = (nargout >= 3);
if trackHistory, tStart = tic; end

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

% ---- 方案 A：多点最近邻初始解 ----
nStarts = nMid;
if nNNRestarts > 0, nStarts = min(nNNRestarts, nMid); end
startSeeds = randperm(nMid, nStarts);

nnCosts = zeros(nStarts, 1);
nnTours = cell(nStarts, 1);
for sIdx = 1:nStarts
    s = startSeeds(sIdx);
    visited = false(1, nMid); visited(s) = true;
    tour = zeros(1, nMid); tour(1) = midIdx(s);
    cur = midIdx(s);
    cost = costMatrix(1, cur);
    for step = 2:nMid
        bestD = inf; bestJ = -1;
        for j = 1:nMid
            if ~visited(j)
                d = costMatrix(cur, midIdx(j));
                if d < bestD, bestD = d; bestJ = j; end
            end
        end
        cost = cost + bestD; visited(bestJ) = true;
        tour(step) = midIdx(bestJ); cur = midIdx(bestJ);
    end
    cost = cost + costMatrix(cur, nPts);
    nnCosts(sIdx) = cost; nnTours{sIdx} = tour;
end
[~, nnBestIdx] = min(nnCosts);
midOrder = nnTours{nnBestIdx};

% ---- 方案 B：初始解 2-opt 优化 ----
[midOrder, curCost] = twoOptLocal(midOrder, costMatrix, nPts);

bestMid = midOrder;
bestCost = curCost;

% ---- SA 参数 ----
if T0_factor > 0
    T0 = T0_factor * max(costMatrix(costMatrix < inf & costMatrix > 0));
else
    T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
end
if isempty(T0) || T0 == 0, T0 = 100; end
T = T0;
nInnerBase = max(nMid * nInnerMult, 50);
numOps = 4;  % swap, insert, 2opt-reverse, or-opt

nEstOuter = ceil(log(T_min / T0) / log(alpha_min)) + 200;
if trackHistory
    bestCostHistory = zeros(nEstOuter, 1);
    avgCostHistory = zeros(nEstOuter, 1);
    timeHistory = zeros(nEstOuter, 1);
end

% --- 方案 D：自适应算子权重 ---
opW = opWeights(:);
opSuccess = zeros(numOps, 1);
opAttempts = zeros(numOps, 1);

% --- 方案 E：自适应停止 ---
stagnationCount = 0;
stopReason = '';
prevBestCost = bestCost;

nOuter = 0;

% ---- 模拟退火主循环 ----
while T > T_min && nOuter < maxOuterIter
    nOuter = nOuter + 1;
    sumCost = 0;
    nAccepted = 0;
    opSuccess(:) = 0;
    opAttempts(:) = 0;

    % --- 自适应内循环长度 ---
    if adaptInner
        nInner = max(nInnerMin, round(nInnerBase * (T / T0)^nInnerPower));
    else
        nInner = nInnerBase;
    end

    curMid = midOrder;

    for i = 1:nInner
        % --- 依权重选择算子 ---
        rOp = rand() * sum(opW);
        cumW = cumsum(opW);
        op = find(cumW >= rOp, 1, 'first');

        switch op
            case 1  % Swap
                pair = randperm(nMid, 2);
                i1 = min(pair); i2 = max(pair);
                [newCost, delta] = deltaSwap(curMid, i1, i2, curCost, costMatrix, nMid, nPts);
                if delta < 0 || rand < exp(-delta / T)
                    curMid([i1, i2]) = curMid([i2, i1]);
                    curCost = newCost; nAccepted = nAccepted + 1;
                    opSuccess(1) = opSuccess(1) + 1;
                end
                opAttempts(1) = opAttempts(1) + 1;

            case 2  % Insert
                pos = randperm(nMid, 2);
                fromPos = min(pos); toPos = max(pos);
                [newCost, delta] = deltaInsert(curMid, fromPos, toPos, curCost, costMatrix, nMid, nPts);
                if delta < 0 || rand < exp(-delta / T)
                    val = curMid(fromPos);
                    curMid(fromPos:toPos-1) = curMid(fromPos+1:toPos);
                    curMid(toPos) = val;
                    curCost = newCost; nAccepted = nAccepted + 1;
                    opSuccess(2) = opSuccess(2) + 1;
                end
                opAttempts(2) = opAttempts(2) + 1;

            case 3  % 2-opt reverse
                pair = sort(randi(nMid, [1, 2]));
                i1 = pair(1); i2 = pair(2);
                if i1 < i2
                    [newCost, delta] = delta2opt(curMid, i1, i2, curCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        curMid(i1:i2) = curMid(i2:-1:i1);
                        curCost = newCost; nAccepted = nAccepted + 1;
                        opSuccess(3) = opSuccess(3) + 1;
                    end
                    opAttempts(3) = opAttempts(3) + 1;
                end

            case 4  % Or-opt（方案 G）
                if enableOropt && nMid >= 4
                    k = oroptLen(randi(length(oroptLen)));  % 随机取 2 或 3
                    maxFrom = nMid - k + 1;
                    from = randi(maxFrom);
                    % 随机选插入位置，排除 [from-1, from+k] 避免邻接
                    validTo = [];
                    for t = 1:nMid
                        if t < from-1 || t >= from+k+1
                            validTo(end+1) = t; %#ok<AGROW>
                        end
                    end
                    if ~isempty(validTo)
                        to = validTo(randi(length(validTo)));
                        [newCost, delta] = deltaOropt(curMid, from, k, to, curCost, costMatrix, nMid, nPts);
                        if delta < 0 || rand < exp(-delta / T)
                            curMid = applyOropt(curMid, from, k, to);
                            curCost = newCost; nAccepted = nAccepted + 1;
                            opSuccess(4) = opSuccess(4) + 1;
                        end
                        opAttempts(4) = opAttempts(4) + 1;
                    end
                end
        end

        sumCost = sumCost + curCost;

        % --- 方案 F：发现全局更优时立即 2-opt 精炼 ---
        if curCost < bestCost
            bestMid = curMid;
            if enableOptOnImprove
                [bestMid, bestCost] = twoOptLocal(bestMid, costMatrix, nPts);
                curMid = bestMid;
                curCost = bestCost;
            else
                bestCost = curCost;
            end
        end
    end

    midOrder = curMid;

    % 记录收敛历史
    if trackHistory
        bestCostHistory(nOuter) = bestCost;
        avgCostHistory(nOuter) = sumCost / nInner;
        timeHistory(nOuter) = toc(tStart);
    end

    % --- 方案 F：低温阶段周期性 2-opt ---
    if enableOptPeriodic && T <= T_cold_factor * T0 && mod(nOuter, optInterval) == 0
        [bestMid, bestCost] = twoOptLocal(bestMid, costMatrix, nPts);
        midOrder = bestMid;
        curCost = bestCost;
    end

    % --- 方案 E：自适应停止检测 ---
    if enableAdaptiveStop && nOuter >= minOuter
        if bestCost < prevBestCost - 1e-10
            stagnationCount = 0;
        elseif T <= T_cold_factor * T0
            stagnationCount = stagnationCount + 1;
            if stagnationCount >= stagnationLim
                stopReason = sprintf('低温停滞(T/T0=%.3g,%d轮)', T/T0, stagnationCount);
                break;
            end
        end
        prevBestCost = bestCost;
    end

    % --- 方案 D：更新算子权重 ---
    for o = 1:numOps
        if opAttempts(o) > 0
            succRate = opSuccess(o) / opAttempts(o);
            opW(o) = opW(o) * opDecay + succRate * opLearnRate;
            opW(o) = max(opW(o), 0.01);
        end
    end

    % --- 自适应冷却 ---
    if adaptAlpha
        acptRate = nAccepted / nInner;
        alpha = alpha_base - alpha_beta * (acptRate - targetAcpt);
        alpha = max(alpha_min, min(alpha_max, alpha));
    else
        alpha = alpha_base;
    end

    T = T * alpha;
end

% ---- 方案 B：全局最优 2-opt 精炼 ----
[bestMid, bestCost] = twoOptLocal(bestMid, costMatrix, nPts);

bestOrder = [1, bestMid, nPts];

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory(1:nOuter);
    history.avgCostHistory = avgCostHistory(1:nOuter);
    history.timeHistory = timeHistory(1:nOuter);
    history.iterCount = nOuter;
    history.elapsedTime = toc(tStart);
    if isempty(stopReason)
        if nOuter >= maxOuterIter
            stopReason = 'maxOuter硬上限';
        else
            stopReason = sprintf('T_min(%.2g)', T_min);
        end
    end
    history.stopReason = stopReason;
    history.nInner = nInner;
    history.finalT = T;
end
end

% =========================================================================
%  增量代价计算（方案 C：O(1) 邻域评价）
% =========================================================================

function [newCost, delta] = deltaSwap(midOrder, i, j, curCost, costMatrix, nMid, nPts)
Li = getLeft(midOrder, i);
Ri = getRight(midOrder, i, nMid, nPts);
Lj = getLeft(midOrder, j);
Rj = getRight(midOrder, j, nMid, nPts);
Ai = midOrder(i); Aj = midOrder(j);
if j == i + 1
    oldEdges = costMatrix(Li, Ai) + costMatrix(Ai, Aj) + costMatrix(Aj, Rj);
    newEdges = costMatrix(Li, Aj) + costMatrix(Aj, Ai) + costMatrix(Ai, Rj);
else
    oldEdges = costMatrix(Li, Ai) + costMatrix(Ai, Ri) ...
             + costMatrix(Lj, Aj) + costMatrix(Aj, Rj);
    newEdges = costMatrix(Li, Aj) + costMatrix(Aj, Ri) ...
             + costMatrix(Lj, Ai) + costMatrix(Ai, Rj);
end
delta = newEdges - oldEdges;
newCost = curCost + delta;
end

function [newCost, delta] = deltaInsert(midOrder, from, to, curCost, costMatrix, nMid, nPts)
X = midOrder(from);
L = getLeft(midOrder, from);
B = midOrder(from + 1);
D = midOrder(to);
R = getRight(midOrder, to, nMid, nPts);
oldEdges = costMatrix(L, X) + costMatrix(X, B) + costMatrix(D, R);
newEdges = costMatrix(L, B) + costMatrix(D, X) + costMatrix(X, R);
delta = newEdges - oldEdges;
newCost = curCost + delta;
end

function [newCost, delta] = delta2opt(midOrder, i, j, curCost, costMatrix, nMid, nPts)
L = getLeft(midOrder, i);
A = midOrder(i);
B = midOrder(j);
R = getRight(midOrder, j, nMid, nPts);
oldEdges = costMatrix(L, A) + costMatrix(B, R);
newEdges = costMatrix(L, B) + costMatrix(A, R);
delta = newEdges - oldEdges;
newCost = curCost + delta;
end

function [newCost, delta] = deltaOropt(midOrder, from, k, to, curCost, costMatrix, nMid, nPts)
%DELTAOROPT Or-opt 增量代价 O(1)
%   移动片段 midOrder(from:from+k-1) 到位置 to 之后
%   前置条件：to < from-1 或 to >= from+k+1（不重叠不邻接）
%
%   变化仅涉及 3 对边：片段两端 + 插入点两端

% 片段边界
if from == 1, Lf = 1; else, Lf = midOrder(from - 1); end
S1 = midOrder(from);
Sk = midOrder(from + k - 1);
if from + k - 1 == nMid, Rf = nPts; else, Rf = midOrder(from + k); end

% 插入点边界
Lt = midOrder(to);
if to == nMid, Rt = nPts; else, Rt = midOrder(to + 1); end

% 移除旧边 (Lf→S1, Sk→Rf, Lt→Rt) + 加入新边 (Lf→Rf, Lt→S1, Sk→Rt)
oldEdges = costMatrix(Lf, S1) + costMatrix(Sk, Rf) + costMatrix(Lt, Rt);
newEdges = costMatrix(Lf, Rf) + costMatrix(Lt, S1) + costMatrix(Sk, Rt);

delta = newEdges - oldEdges;
newCost = curCost + delta;
end

function newMid = applyOropt(midOrder, from, k, to)
%APPLYOROPT 执行 Or-opt 片段移动
seg = midOrder(from : from + k - 1);
% 移除片段
newMid = [midOrder(1:from-1), midOrder(from+k:end)];
% 确定插入位置（to > from 时需补偿偏移）
if to > from
    toNew = to - k;
else
    toNew = to;
end
% 插入片段
newMid = [newMid(1:toNew), seg, newMid(toNew+1:end)];
end

% =========================================================================
%  辅助函数
% =========================================================================

function L = getLeft(order, pos)
if pos == 1, L = 1; else, L = order(pos - 1); end
end

function R = getRight(order, pos, nMid, nPts)
if pos == nMid, R = nPts; else, R = order(pos + 1); end
end

% =========================================================================
%  2-opt 局部搜索
% =========================================================================
function [order, cost] = twoOptLocal(order, costMatrix, nPts)
nMid = length(order);
fullOrder = [1, order, nPts];

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
            old = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + ...
                  costMatrix(fullOrder(fj), fullOrder(fj + 1));
            new = costMatrix(fullOrder(fi), fullOrder(fj)) + ...
                  costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
            if new < old - 1e-10
                order((i + 1):j) = order(j:-1:(i + 1));
                cost = cost - old + new;
                improved = true;
                fullOrder = [1, order, nPts];
            end
        end
    end
end
end
