function [bestOrder, bestCost, history] = TSP_SA_v0(costMatrix, nPts)
%TSP_SA_V0 经典模拟退火算法求解 TSP
%   最简 SA 基线: 贪心初始解 + swap 单邻域 + 指数降温 + T_min 终止
%   用于对比: 仅自适应冷却 + 自适应内循环能带来多大提升 (v0_1)。
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost, history

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 可调参数 =====
T0_factor = 3.0;           % 初始温度因子
alpha     = 0.996;         % 降温系数 (固定)
T_min     = 1e-3;          % 终止温度
nInnerMult = 10;           % 内循环倍数
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
% % ---- 贪心初始解 (单次 NN, 已注释) ----
% cur = 1;  % start point index in costMatrix
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
nInner = max(nMid * nInnerMult, 50);

nEst = ceil(log(T_min / T0) / log(alpha)) + 200;
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
