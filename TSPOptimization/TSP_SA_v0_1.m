function [bestOrder, bestCost, history] = TSP_SA_v0_1(costMatrix, nPts)
%TSP_SA_V0_1 自适应模拟退火算法求解 TSP
%   在经典 SA (v0) 基础上仅加入两项自适应改进:
%     1. 自适应冷却速率 — 根据接受率动态调节 alpha
%     2. 自适应内循环 — 高温多迭代，低温少迭代
%
%   其余与 v0 一致: 贪心初始解 + swap 单邻域 + T_min 终止。
%   用于对比: 这两项自适应改进能带来多大提升。
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost, history

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 可调参数 =====
% --- 冷却参数 ---
T0_factor  = 5.0;           % 初始温度因子
alpha_base = 0.998;         % 基础冷却系数
alpha_min  = 0.996;         % 冷却系数下限
alpha_max  = 0.9995;         % 冷却系数上限
alpha_beta = 0.005;         % 自适应强度
T_min      = 1e-50;          % 终止温度

% --- 内循环 ---
adaptInner  = 1;            % 1=内循环随温度自适应
nInnerMult  = 10;           % 内循环基数
nInnerMin   = 55;           % 内循环最小长度
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

% ---- 随机初始解 ----
% % ---- 贪心初始解 (单次 NN, 与 v0 一致) ----
% cur = 1;
% visited = false(1, nMid);
% midOrder = zeros(1, nMid);
% for s = 1:nMid
%     bestDist = inf; bestJ = 0;
%     for j = 1:nMid
%         if ~visited(j)
%             d = costMatrix(cur, midIdx(j));
%             if d < bestDist, bestDist = d; bestJ = j; end
%         end
%     end
%     visited(bestJ) = true;
%     midOrder(s) = midIdx(bestJ);
%     cur = midIdx(bestJ);
% end
midOrder = midIdx(randperm(nMid));

curCost = calcCost(midOrder, costMatrix, nPts);
bestMid = midOrder; bestCost = curCost;

% ---- SA 参数 ----
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
        % 单一邻域操作: swap
        pair = randperm(nMid, 2);
        i1 = min(pair); i2 = max(pair);
        [newCost, delta] = deltaSwap(curMid, i1, i2, curCost, costMatrix, nMid, nPts);

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
function c = calcCost(order, costMatrix, nPts)
fullOrder = [1, order, nPts]; c = 0;
for k = 1:(nPts-1), c = c + costMatrix(fullOrder(k), fullOrder(k+1)); end
end

function [nc, d] = deltaSwap(o, i, j, cur, cm, nM, nP)
Li=gL(o,i);Ri=gR(o,i,nM,nP);Lj=gL(o,j);Rj=gR(o,j,nM,nP);Ai=o(i);Aj=o(j);
if j==i+1
    old=cm(Li,Ai)+cm(Ai,Aj)+cm(Aj,Rj); nw=cm(Li,Aj)+cm(Aj,Ai)+cm(Ai,Rj);
else
    old=cm(Li,Ai)+cm(Ai,Ri)+cm(Lj,Aj)+cm(Aj,Rj); nw=cm(Li,Aj)+cm(Aj,Ri)+cm(Lj,Ai)+cm(Ai,Rj);
end
d=nw-old; nc=cur+d;
end

function L=gL(o,p), if p==1,L=1;else,L=o(p-1);end,end
function R=gR(o,p,nM,nP), if p==nM,R=nP;else,R=o(p+1);end,end
