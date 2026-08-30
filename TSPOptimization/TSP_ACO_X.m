function [bestOrder, bestCost, history] = TSP_ACO_X(costMatrix, nPts)
%TSP_ACO_X 蚁群算法求解 TSP（虚拟节点法：起点终点固定 + 中间点全排列优化）
%   在原 TSP_ACO 基础上，将起点终点固定问题通过虚拟节点转化为闭合环路 TSP。
%   虚拟节点 V 与起点(1)、终点(nPts) 距离为 0，与中间点距离 BIG。
%   求解闭合环路后，去掉虚拟节点并校正方向，得到 起点→中间点→终点 的开路径。
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵 (1=起点, nPts=终点)
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量, 1 在首、nPts 在尾）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% 扩展成本矩阵: 添加虚拟节点 V = nPts+1
nNodes = nPts + 1;
Vidx = nPts + 1;
maxFinite = max(costMatrix(costMatrix < inf));
if isempty(maxFinite), maxFinite = 1; end
BIG = (nPts + 1) * maxFinite * 2;

costExt = zeros(nNodes);
costExt(1:nPts, 1:nPts) = costMatrix;
costExt(Vidx, 1) = 0;  costExt(1, Vidx) = 0;
costExt(Vidx, nPts) = 0;  costExt(nPts, Vidx) = 0;
costExt(Vidx, midIdx) = BIG;  costExt(midIdx, Vidx) = BIG;

% 算法参数
nAnts = 40;
nIter = 500;
alpha = 1.0;
beta = 2.0;
rho = 0.25;
Q = 225;
tau0 = 0.1;

trackHistory = (nargout >= 3);
if trackHistory
    tStart = tic;
    bestCostHistory = zeros(nIter, 1);
    avgCostHistory = zeros(nIter, 1);
    timeHistory = zeros(nIter, 1);
end

% 边界情况：无中间目标点
if nMid == 0
    bestOrder = [1, nPts];
    bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, ...
            'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

% 初始化信息素矩阵 (全节点)
tau = ones(nNodes) * tau0;

% 启发式信息矩阵 η = 1/d (全节点, 虚拟节点边 cap)
eta = zeros(nNodes);
for i = 1:nNodes
    for j = 1:nNodes
        if i ~= j && costExt(i,j) > 0 && ~isinf(costExt(i,j))
            eta(i,j) = 1.0 / costExt(i,j);
        end
    end
end
% V 与起点/终点的 0 距离边: 1/0 无意义，用极大值表示"必连"
eta(Vidx, 1) = BIG;    eta(1, Vidx) = BIG;
eta(Vidx, nPts) = BIG;  eta(nPts, Vidx) = BIG;

globalBestCost = inf;
globalBestTour = [];

for iter = 1:nIter
    antTours = cell(nAnts, 1);
    antCosts = zeros(nAnts, 1);

    for a = 1:nAnts
        tour = zeros(1, nNodes);
        visited = false(1, nNodes);
        % 蚂蚁从中间点随机起始（V/起点/终点位置固定，不应作为起始点）
        start = midIdx(randi(nMid));
        tour(1) = start;
        visited(start) = true;

        for step = 2:nNodes
            cur = tour(step - 1);
            prob = zeros(1, nNodes);
            for j = 1:nNodes
                if ~visited(j)
                    prob(j) = (tau(cur, j) ^ alpha) * (eta(cur, j) ^ beta);
                end
            end
            totalProb = sum(prob);
            if totalProb == 0
                unvisited = find(~visited);
                next = unvisited(randi(length(unvisited)));
            else
                prob = prob / totalProb;
                cumProb = cumsum(prob);
                r = rand();
                next = find(cumProb >= r, 1, 'first');
            end
            tour(step) = next;
            visited(next) = true;
        end

        antTours{a} = tour;
        antCosts(a) = cycleCost(tour, costExt, nNodes);
    end

    [iterBestCost, bestAntIdx] = min(antCosts);
    iterBestTour = antTours{bestAntIdx};

    if iterBestCost < globalBestCost
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % 信息素蒸发
    tau = tau * (1 - rho);

    % 信息素沉积（本代最优, 跳过 V 边）
    deposit = Q / iterBestCost;
    for k = 1:nNodes
        i = iterBestTour(k);
        j = iterBestTour(mod(k, nNodes) + 1);
        if costExt(i,j) > 0 && ~isinf(costExt(i,j))
            tau(i, j) = tau(i, j) + deposit;
            tau(j, i) = tau(j, i) + deposit;
        end
    end

    % 精英沉积（全局最优, 跳过 V 边）
    eliteDeposit = Q / globalBestCost * 0.3;
    for k = 1:nNodes
        i = globalBestTour(k);
        j = globalBestTour(mod(k, nNodes) + 1);
        if costExt(i,j) > 0 && ~isinf(costExt(i,j))
            tau(i, j) = tau(i, j) + eliteDeposit;
            tau(j, i) = tau(j, i) + eliteDeposit;
        end
    end
end

bestCost = globalBestCost;
bestOrder = extractPath(globalBestTour, nPts, Vidx);

if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory;
    history.avgCostHistory = avgCostHistory;
    history.timeHistory = timeHistory;
    history.iterCount = nIter;
    history.elapsedTime = toc(tStart);
end
end

% =========================================================================
function c = cycleCost(cycle, costExt, n)
c = 0;
for k = 1:n
    c = c + costExt(cycle(k), cycle(mod(k, n) + 1));
end
end

function order = extractPath(cycle, nPts, Vidx)
n = length(cycle);
vPos = find(cycle == Vidx, 1);
order = zeros(1, nPts);
idx = mod(vPos, n) + 1;
for t = 1:nPts
    order(t) = cycle(idx);
    idx = mod(idx, n) + 1;
end
if order(1) == nPts
    order = fliplr(order);
end
if order(1) ~= 1
    p1 = find(order == 1, 1);
    order = [order(p1:end), order(1:p1-1)];
end
end
