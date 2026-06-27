function [bestOrder, bestCost, history] = TSP_SA(costMatrix, nPts)
%TSP_SA 模拟退火算法求解 TSP
%   采用指数降温 + 多种邻域操作（swap/insert/2-opt）
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体:
%       .bestCostHistory  - 每轮最优成本
%       .avgCostHistory   - 每轮平均成本
%       .timeHistory      - 每轮累计耗时（秒）
%       .iterCount        - 总迭代轮数
%       .elapsedTime      - 纯计算耗时（秒）

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% 是否记录收敛历史
trackHistory = (nargout >= 3);
if trackHistory, tStart = tic; end

% ---- 初始解：最近邻启发式 ----
startIdx = 1;
visited = false(1, nMid);
cur = startIdx;
midOrder = zeros(1, nMid);
for s = 1:nMid
    bestDist = inf;
    bestJ = 0;
    for j = 1:nMid
        if ~visited(j)
            ptIdx = midIdx(j);
            d = costMatrix(cur, ptIdx);
            if d < bestDist
                bestDist = d;
                bestJ = j;
            end
        end
    end
    visited(bestJ) = true;
    midOrder(s) = midIdx(bestJ);
    cur = midIdx(bestJ);
end

% ---- 代价计算函数 ----
    function c = calcCost(order)
        fullOrd = [1, order, nPts];
        c = 0;
        for k = 1:(nPts - 1)
            c = c + costMatrix(fullOrd(k), fullOrd(k + 1));
        end
    end

% ---- 模拟退火参数 ----
curCost = calcCost(midOrder);
bestMid = midOrder;
bestCost = curCost;

T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
if isempty(T0), T0 = 100; end
T = T0;
T_min = 1e-3;
alpha = 0.995;
nInner = max(nMid * 10, 50);

nOuter = 0;

if trackHistory
    maxOuter = ceil(log(T_min / T0) / log(alpha));
    bestCostHistory = zeros(maxOuter, 1);
    avgCostHistory = zeros(maxOuter, 1);
    timeHistory = zeros(maxOuter, 1);
end

% ---- 模拟退火主循环 ----
while T > T_min
    nOuter = nOuter + 1;
    sumCost = 0;
    nAccepted = 0;

    for i = 1:nInner
        % 随机选择邻域操作
        op = randi(3);
        newMid = midOrder;

        switch op
            case 1  % Swap：随机交换两个中间点
                pair = randperm(nMid, 2);
                newMid(pair) = newMid(fliplr(pair));

            case 2  % Insert：随机取出一个点插入到另一位置
                pos = randperm(nMid, 2);
                fromPos = min(pos);
                toPos = max(pos);
                val = newMid(fromPos);
                newMid(fromPos:toPos-1) = newMid(fromPos+1:toPos);
                newMid(toPos) = val;

            case 3  % 2-opt：反转一段子序列
                pair = sort(randi(nMid, [1, 2]));
                if pair(1) < pair(2)
                    newMid(pair(1):pair(2)) = newMid(pair(2):-1:pair(1));
                end
        end

        newCost = calcCost(newMid);
        delta = newCost - curCost;
        sumCost = sumCost + curCost;

        % Metropolis 接受准则
        if delta < 0 || rand < exp(-delta / T)
            midOrder = newMid;
            curCost = newCost;
            nAccepted = nAccepted + 1;

            if curCost < bestCost
                bestCost = curCost;
                bestMid = midOrder;
            end
        end
    end

    % 记录收敛历史
    if trackHistory
        bestCostHistory(nOuter) = bestCost;
        avgCostHistory(nOuter) = sumCost / nInner;
        timeHistory(nOuter) = toc(tStart);
    end

    % 降温
    T = T * alpha;
end

% ---- 组装输出 ----
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
