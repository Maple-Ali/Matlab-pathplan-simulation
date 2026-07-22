function [bestOrder, bestCost, history] = TSP_SA_v1_1(costMatrix, nPts)
%TSP_SA_V1_1 改进模拟退火算法求解 TSP
%   相对 TSP_SA_v1 的改进:
%     F. 自适应冷却速率：根据接受率动态调节 alpha，保持 ~50% 接受率
%     G. 自适应内循环：高温多迭代充分探索，低温少迭代精细搜索
%     （移除 v1 的重升温功能，表现不稳定）
%
%   保留 v1 的改进:
%     A. 多点最近邻初始解
%     B. 初始解 + 最终解 2-opt 局部搜索精炼
%     C. 增量代价计算 delta O(1)
%     D. 自适应算子权重
%     E. 基于温度的停滞检测
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 关键可调参数 =====================================================
% --- 初始解 ---
nNNRestarts = 0;            % 多点NN数量（0=尝试全部nMid个起点）

% --- 自适应冷却速率（方案F） ---
alpha_base   = 0.995;       % 基础冷却系数（固定模式时的值）
adaptAlpha   = 1;           % 1=自适应冷却, 0=固定 alpha=alpha_base
alpha_min    = 0.990;       % 冷却系数下限（快冷却，防浮点误差累积）
alpha_max    = 0.998;       % 冷却系数上限（慢冷却，接受率偏低时减速）
alpha_beta   = 0.005;       % 自适应强度（alpha += beta*(targetAcpt-acptRate)）
targetAcpt   = 0.50;        % 目标接受率（SA经典值0.5，平衡探索与开发）

% --- 内循环自适应（方案G） ---
adaptInner   = 1;           % 1=内循环随温度自适应, 0=固定
nInnerMult   = 10;          % 内循环基数（高温时 nInner = max(nMid*nInnerMult, 50)）
nInnerMin    = 30;          % 内循环最小长度（低温保底，太少无法采样）
nInnerPower  = 0.5;         % 温度权重指数（0.5=平方根，1=线性；越小低温衰减越缓）

% --- 温度 ---
T0_factor    = 1.0;         % 初始温度因子（>0 时 T0 = factor*最大边长, ≤0 自动）
T_min        = 1e-3;        % 终止温度

% --- 邻域算子权重 ---
opWeights    = [1.0, 1.0, 1.0];  % [swap, insert, 2-opt] 初始权重
opLearnRate  = 0.15;             % 权重学习率
opDecay      = 0.95;             % 每轮权重衰减

% --- 自适应停止（方案E） ---
enableAdaptiveStop = 1;     % 1=开启自适应停止
T_cold_factor = 0.02;       % 仅当 T/T0 < factor 时才计停滞（高温探索期不计数）
stagnationLim = 250;        % 低温阶段连续停滞上限（外循环轮数）
minOuter      = 80;         % 自适应停止最少外循环轮数
maxOuterIter  = 3000;       % 外循环硬上限（安全网）
% ========================================================================

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory, tStart = tic; end

% 边界情况：无中间目标点
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
if nNNRestarts > 0
    nStarts = min(nNNRestarts, nMid);
end
startSeeds = randperm(nMid, nStarts);

nnCosts = zeros(nStarts, 1);
nnTours = cell(nStarts, 1);
for sIdx = 1:nStarts
    s = startSeeds(sIdx);
    visited = false(1, nMid);
    visited(s) = true;
    tour = zeros(1, nMid);
    tour(1) = midIdx(s);
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
        cost = cost + bestD;
        visited(bestJ) = true;
        tour(step) = midIdx(bestJ);
        cur = midIdx(bestJ);
    end
    cost = cost + costMatrix(cur, nPts);
    nnCosts(sIdx) = cost;
    nnTours{sIdx} = tour;
end
[~, nnBestIdx] = min(nnCosts);
midOrder = nnTours{nnBestIdx};

% ---- 方案 B：初始解 2-opt 优化 ----
[midOrder, curCost] = twoOpt(midOrder, costMatrix, nPts);

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

% 内循环基数（高温时使用）
nInnerBase = max(nMid * nInnerMult, 50);

% 预估最大外循环数（用于预分配 history）
nEstOuter = ceil(log(T_min / T0) / log(alpha_min)) + 200;
if trackHistory
    bestCostHistory = zeros(nEstOuter, 1);
    avgCostHistory = zeros(nEstOuter, 1);
    timeHistory = zeros(nEstOuter, 1);
end

% --- 方案 D：自适应算子权重 ---
opW = opWeights(:);
opSuccess = zeros(3, 1);
opAttempts = zeros(3, 1);

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

    % --- 方案 G：内循环长度随温度自适应 ---
    if adaptInner
        nInner = max(nInnerMin, round(nInnerBase * (T / T0)^nInnerPower));
    else
        nInner = nInnerBase;
    end

    curMid = midOrder;

    for i = 1:nInner
        % --- 方案 D：依权重选择算子 ---
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
                    curCost = newCost;
                    nAccepted = nAccepted + 1;
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
                    curCost = newCost;
                    nAccepted = nAccepted + 1;
                    opSuccess(2) = opSuccess(2) + 1;
                end
                opAttempts(2) = opAttempts(2) + 1;

            case 3  % 2-opt (reverse)
                pair = sort(randi(nMid, [1, 2]));
                i1 = pair(1); i2 = pair(2);
                if i1 < i2
                    [newCost, delta] = delta2opt(curMid, i1, i2, curCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        curMid(i1:i2) = curMid(i2:-1:i1);
                        curCost = newCost;
                        nAccepted = nAccepted + 1;
                        opSuccess(3) = opSuccess(3) + 1;
                    end
                    opAttempts(3) = opAttempts(3) + 1;
                end
        end

        sumCost = sumCost + curCost;

        if curCost < bestCost
            bestCost = curCost;
            bestMid = curMid;
        end
    end

    midOrder = curMid;

    % 记录收敛历史
    if trackHistory
        bestCostHistory(nOuter) = bestCost;
        avgCostHistory(nOuter) = sumCost / nInner;
        timeHistory(nOuter) = toc(tStart);
    end

    % --- 方案 E：自适应停止检测（仅低温阶段计数） ---
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
    for o = 1:3
        if opAttempts(o) > 0
            succRate = opSuccess(o) / opAttempts(o);
            opW(o) = opW(o) * opDecay + succRate * opLearnRate;
            opW(o) = max(opW(o), 0.01);
        end
    end

    % --- 方案 F：自适应冷却速率 ---
    if adaptAlpha
        acptRate = nAccepted / nInner;
        % alpha = alpha_base - beta*(acptRate - targetAcpt)
        % 接受率高于目标 → 降温更快（alpha↓）
        % 接受率低于目标 → 降温更慢（alpha↑）
        alpha = alpha_base - alpha_beta * (acptRate - targetAcpt);
        alpha = max(alpha_min, min(alpha_max, alpha));
    else
        alpha = alpha_base;
    end

    % 降温
    T = T * alpha;
end

% ---- 方案 B：全局最优 2-opt 精炼 ----
[bestMid, bestCost] = twoOpt(bestMid, costMatrix, nPts);

% ---- 组装输出 ----
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
    history.nInner = nInner;   % 末轮内循环长度
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
Ai = midOrder(i);
Aj = midOrder(j);

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
%  2-opt 局部搜索（方案 B）
% =========================================================================
function [order, cost] = twoOpt(order, costMatrix, nPts)
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
