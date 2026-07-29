function [bestOrder, bestCost, history] = TSP_SA_G(costMatrix, nPts)
%TSP_SA_G 遗传模拟退火算法求解 TSP（交叉产子 + SA 精炼 + 准则取舍）
%   架构：每代 = 交叉(GA) → SA局部搜索精炼 → Metropolis 取舍
%   核心: GA 交叉产生候选解，SA 精炼并决定是否接受（差解可存活）
%
%   Inputs: costMatrix, nPts
%   Outputs: bestOrder, bestCost, history

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 关键可调参数 =====================================================
% --- 种群 ---
popSize    = 20;          % 种群大小
nHeuristic = 5;           % NN 启发式初始化个数
crossRate  = 0.80;        % 交叉概率 (OX)

% --- SA 局部搜索（精炼每次交叉产生的子代）---
nInnerMult = 5;           % 内循环倍数（nInner = nMid * nInnerMult）
T0_factor  = 0.5;         % 初始温度因子
alpha      = 0.99;        % 降温系数
T_min      = 0.01;        % 终止温度

% --- 自适应停止 ---
enableAdaptiveStop = 1;
stagnationLim = 80;
maxGen       = 3000;
% ========================================================================

trackHistory = (nargout >= 3);
if trackHistory, tStart = tic; end

if nMid == 0
    bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
    end
    return;
end

% ---- 初始化种群 ----
pop = zeros(popSize, nMid);
for s = 1:min(nHeuristic, nMid)
    visited = false(1, nMid); visited(s) = true;
    tour = zeros(1, nMid); tour(1) = midIdx(s); cur = s;
    for step = 2:nMid
        bestD = inf; bestJ = 0;
        for j = 1:nMid
            if ~visited(j)
                d = costMatrix(midIdx(cur), midIdx(j));
                if d < bestD, bestD = d; bestJ = j; end
            end
        end
        visited(bestJ) = true;
        tour(step) = midIdx(bestJ); cur = bestJ;
    end
    pop(s, :) = tour;
end
for i = (nHeuristic + 1):popSize
    pop(i, :) = midIdx(randperm(nMid));
end

% 适应度
fit = zeros(popSize, 1);
for i = 1:popSize
    fit(i) = calcCost(pop(i, :), costMatrix, nPts);
end
[bestCost, bestIdx] = min(fit);
bestMid = pop(bestIdx, :);

% SA 温度
if T0_factor > 0
    T0 = T0_factor * max(costMatrix(costMatrix < inf & costMatrix > 0));
else
    T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
end
if isempty(T0) || T0 == 0, T0 = 100; end
T = T0;
nInner = max(nMid * nInnerMult, 20);
mutWeights = [1.0, 1.0, 1.0];

stagnationCount = 0;
prevBestCost = bestCost;
stopReason = '';

nEst = ceil(log(T_min / T0) / log(alpha)) + 200;
if trackHistory
    bestCostHistory = zeros(nEst, 1);
    avgCostHistory = zeros(nEst, 1);
    timeHistory = zeros(nEst, 1);
end

gen = 0;

while T > T_min && gen < maxGen
    gen = gen + 1;
    nAccepted = 0;

    % ======== 对每个个体：交叉产子 → SA 精炼 → Metropolis 取舍 ========
    for i = 1:popSize
        % 锦标赛选择配偶
        cand = randi(popSize, [2, 1]); [~, w] = min(fit(cand));
        partner = pop(cand(w), :);

        % OX 交叉产生子代
        if rand < crossRate
            cp = sort(randi(nMid, [1, 2]));
            child = zeros(1, nMid);
            child(cp(1):cp(2)) = pop(i, cp(1):cp(2));
            rem = setdiff(partner, child(cp(1):cp(2)), 'stable');
            idx = 1;
            for j = 1:nMid, if child(j)==0, child(j)=rem(idx); idx=idx+1; end; end
        else
            child = pop(i, :);  % 不交叉 → 子代=父代
        end

        childCost = calcCost(child, costMatrix, nPts);

        % ======== SA 局部搜索精炼子代 ========
        for step = 1:nInner
            rOp = rand() * sum(mutWeights);
            cumW = cumsum(mutWeights);
            op = find(cumW >= rOp, 1, 'first');

            switch op
                case 1  % Swap
                    pair = randperm(nMid, 2);
                    i1 = min(pair); i2 = max(pair);
                    [nc, delta] = deltaSwap(child, i1, i2, childCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        child([i1, i2]) = child([i2, i1]); childCost = nc;
                    end
                case 2  % Insert
                    pos = randperm(nMid, 2);
                    from = min(pos); to = max(pos);
                    [nc, delta] = deltaInsert(child, from, to, childCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        val = child(from);
                        child(from:to-1) = child(from+1:to);
                        child(to) = val; childCost = nc;
                    end
                case 3  % 2-opt reverse
                    pair = sort(randi(nMid, [1, 2]));
                    i1 = pair(1); i2 = pair(2);
                    if i1 < i2
                        [nc, delta] = delta2opt(child, i1, i2, childCost, costMatrix, nMid, nPts);
                        if delta < 0 || rand < exp(-delta / T)
                            child(i1:i2) = child(i2:-1:i1); childCost = nc;
                        end
                    end
            end
        end

        % ======== Metropolis 准则：精炼子代 vs 原始父代 ========
        delta = childCost - fit(i);
        if delta < 0 || rand < exp(-delta / T)
            pop(i, :) = child;
            fit(i) = childCost;
            nAccepted = nAccepted + 1;
        end
    end

    % 更新全局最优
    [genBestCost, genBestIdx] = min(fit);
    if genBestCost < bestCost - 1e-10
        bestCost = genBestCost;
        bestMid = pop(genBestIdx, :);
    end

    % ======== 精英保留 ========
    [~, sortIdx] = sort(fit);
    % 全局最优覆盖最差个体
    pop(sortIdx(end), :) = bestMid;
    fit(sortIdx(end)) = bestCost;

    if trackHistory
        bestCostHistory(gen) = bestCost;
        avgCostHistory(gen) = mean(fit);
        timeHistory(gen) = toc(tStart);
    end

    % 自适应停止
    if enableAdaptiveStop
        if bestCost < prevBestCost - 1e-10
            stagnationCount = 0;
        else
            stagnationCount = stagnationCount + 1;
            if stagnationCount >= stagnationLim
                stopReason = sprintf('最优停滞(%d代)', stagnationCount);
                break;
            end
        end
        prevBestCost = bestCost;
    end

    T = T * alpha;
end

% 最终精炼
[bestMid, bestCost] = twoOptLocal(bestMid, costMatrix, nPts);
bestOrder = [1, bestMid, nPts];

if trackHistory
    % 追加 2-opt 精炼结果到 history
    bestCostHistory(gen + 1) = bestCost;
    avgCostHistory(gen + 1) = bestCost;
    timeHistory(gen + 1) = toc(tStart);
    genFinal = gen + 1;

    history = struct();
    history.bestCostHistory = bestCostHistory(1:genFinal);
    history.avgCostHistory = avgCostHistory(1:genFinal);
    history.timeHistory = timeHistory(1:genFinal);
    history.iterCount = genFinal;
    history.elapsedTime = toc(tStart);
    if isempty(stopReason)
        if T <= T_min, stopReason = sprintf('T_min(%.2g)', T_min);
        else, stopReason = 'maxGen'; end
    end
    history.stopReason = stopReason;
    history.finalT = T;
end
end

% =========================================================================
function c = calcCost(order, costMatrix, nPts)
fullOrder = [1, order, nPts]; c = 0;
for k = 1:(nPts - 1), c = c + costMatrix(fullOrder(k), fullOrder(k + 1)); end
end

% =========================================================================
function [nc, delta] = deltaSwap(o, i, j, cur, cm, nM, nP)
Li=getL(o,i);Ri=getR(o,i,nM,nP);Lj=getL(o,j);Rj=getR(o,j,nM,nP);Ai=o(i);Aj=o(j);
if j==i+1
    old=cm(Li,Ai)+cm(Ai,Aj)+cm(Aj,Rj); nw=cm(Li,Aj)+cm(Aj,Ai)+cm(Ai,Rj);
else
    old=cm(Li,Ai)+cm(Ai,Ri)+cm(Lj,Aj)+cm(Aj,Rj); nw=cm(Li,Aj)+cm(Aj,Ri)+cm(Lj,Ai)+cm(Ai,Rj);
end
delta=nw-old; nc=cur+delta;
end

function [nc, delta] = deltaInsert(o, f, t, cur, cm, nM, nP)
X=o(f);L=getL(o,f);B=o(f+1);D=o(t);R=getR(o,t,nM,nP);
old=cm(L,X)+cm(X,B)+cm(D,R); nw=cm(L,B)+cm(D,X)+cm(X,R);
delta=nw-old; nc=cur+delta;
end

function [nc, delta] = delta2opt(o, i, j, cur, cm, nM, nP)
L=getL(o,i);A=o(i);B=o(j);R=getR(o,j,nM,nP);
old=cm(L,A)+cm(B,R); nw=cm(L,B)+cm(A,R);
delta=nw-old; nc=cur+delta;
end

function L=getL(o,p), if p==1,L=1;else,L=o(p-1);end,end
function R=getR(o,p,nM,nP), if p==nM,R=nP;else,R=o(p+1);end,end

% =========================================================================
function [order, cost] = twoOptLocal(order, costMatrix, nPts)
nMid = length(order); fullOrder = [1, order, nPts];
cost = 0; for k = 1:(nPts-1), cost = cost + costMatrix(fullOrder(k), fullOrder(k+1)); end
improved = true;
while improved
    improved = false;
    for i = 1:(nMid-1)
        for j = (i+1):nMid
            fi=i+1; fj=j+1;
            old=costMatrix(fullOrder(fi),fullOrder(fi+1))+costMatrix(fullOrder(fj),fullOrder(fj+1));
            nw=costMatrix(fullOrder(fi),fullOrder(fj))+costMatrix(fullOrder(fi+1),fullOrder(fj+1));
            if nw<old-1e-10
                order((i+1):j)=order(j:-1:(i+1)); cost=cost-old+nw; improved=true;
                fullOrder=[1,order,nPts];
            end
        end
    end
end
end
