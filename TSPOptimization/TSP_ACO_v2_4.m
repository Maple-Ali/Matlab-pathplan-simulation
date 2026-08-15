function [bestOrder, bestCost, history] = TSP_ACO_v2_4(costMatrix, nPts)
%TSP_ACO_V2_4 蚁群 + 虚拟点(起点终点合并) + 候选列表加速 VND 求解 TSP
%   基于 TSP_ACO_v2_3，核心升级:
%     V. 虚拟点法: 将起点(节点1)与终点(节点nPts)合并为一个虚拟点 V,
%        把"固定起点终点的路径 TSP"转化为"闭合回路 TSP"。
%        虚拟点 V 到起点/终点的距离为 0 (起点=终点=同一货栈时等价于 V 位于货栈处);
%        虚拟点 V 到各中间点的距离 = 货栈到该中间点的真实距离
%        (起点=终点场景下 V 与中间点并非"无穷远", 而是货栈真实距离——
%         这是虚拟点法可计算的关键; 若按字面设无穷远, 回路成本恒为无穷)。
%
%   与 v2_3 的区别:
%       v2_3 将起点1/终点nPts 固定为路径两端, 信息素仅覆盖中间点;
%       v2_4 将起点终点合并为虚拟点 V, 作为闭合回路中的一个普通节点,
%            信息素覆盖 V 及全部中间点, 2-opt/重定位/交换在完整回路上进行。
%
%   约化节点映射: 约化节点 1 = 虚拟点 V; 约化节点 k (k>=2) = 原始节点 k。
%   回路表示 [1, tour, 1] (V 固定为回路首), 还原为路径 [1, tour, nPts]。
%
%   保留 v2_3 全部机制: MMAS / C(候选列表) / K(伪随机) / S(S曲线) / B(自适应停止)

nMid = nPts - 2;
nNodes = nMid + 1;          % 虚拟点(1) + 中间点(2..nPts-1) = nPts-1 个节点

% ===== 算法基础参数 =====
nAnts = 40;              % 蚂蚁数量
nIter = 800;             % 最大迭代次数
alpha = 1;               % 信息素权重
beta  = 2.0;             % 启发式信息权重
rho   = 0.25;            % 信息素蒸发率
Q     = 225;             % 信息素沉积常数

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 1;  % 1=开启
q0 = 0.45;               % 贪心选择概率

% ===== 方案 S：渐进式局部搜索 (S 曲线) =====
optRatio_start = 0.00;       % 初期局部搜索比例 (y_min)
optRatio_end   = 0.3;        % 后期局部搜索比例 (y_max)
optTransInterval = [0, 100]; % 过渡区间 [x_L, x_R]
optCurveA      = 2.0;        % S 曲线陡峭度
optEliteRatio  = 0.6;        % 局部搜索预算中精英蚂蚁占比
optRandomCap   = 0.15;       % 随机抽取比例上限

% ===== 方案 C：候选列表加速 VND =====
kCand = 9;               % 每个节点的候选邻居数

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;  % 1=开启
cvThreshold  = 0.001;    % 种群 CV 阈值 (CV < 阈值 → 种群同质停止)
stagnationLim = 50;      % 最优解连续停滞上限 (代)
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
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, 'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
    end
    return;
end

% ---- 方案 V：构建虚拟点约化闭合回路成本矩阵 (nNodes×nNodes) ----
cmRed = zeros(nNodes);
cmRed(2:nNodes, 2:nNodes) = costMatrix(2:(nPts-1), 2:(nPts-1));   % 中间点之间
cmRed(1, 2:nNodes)        = costMatrix(1, 2:(nPts-1));            % V → 中间点 (货栈真实距离)
cmRed(2:nNodes, 1)        = costMatrix(2:(nPts-1), 1);            % 中间点 → V

% ---- 方案 C：预计算候选列表 ----
% isCand(i,j) = true 若 j 是 i 的 k 个最近邻居之一
isCand = false(nNodes, nNodes);
for i = 1:nNodes
    dists = cmRed(i, :);
    dists(i) = inf;                    % 排除自身
    [~, idx] = sort(dists);
    isCand(i, idx(1:min(kCand, nNodes-1))) = true;
end

% 启发式信息矩阵
eta = zeros(nNodes, nNodes);
for i = 1:nNodes
    for j = 1:nNodes
        if i ~= j
            d = cmRed(i, j);
            if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
        end
    end
end

% ---- 均匀初始信息素 (无外部启发式) ----
tau0 = 0.1;
tau = ones(nNodes, nNodes) * tau0;

% ---- 方案 A：MMAS 初始信息素限幅 (保守) ----
tauMax = 1 / (rho * tau0 * nNodes);
tauMin = tauMax / (5 * nNodes);
tauMin = max(tauMin, 1e-6);
tau = min(tau, tauMax);
tau = max(tau, tauMin);

globalBestCost = inf;
globalBestTour = 2:nNodes;
costTol = 1e-9;           % 成本比较容差 (消除浮点累加噪声, 100边×eps≈3.6e-10)
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
        tRel = (iter - xL) / (xR - xL);
        optRatio = optRatio_start + (optRatio_end - optRatio_start) * ...
            (tRel^optCurveA / (tRel^optCurveA + (1-tRel)^optCurveA));
    end
    nOptAnts = max(1, round(nAnts * optRatio));

    antTours = cell(nAnts, 1);
    antCosts = zeros(nAnts, 1);

    for a = 1:nAnts
        tour = zeros(1, nMid);
        visited = false(1, nNodes);
        visited(1) = true;              % 虚拟点 V 固定为回路首
        start = randi([2, nNodes]);
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
        antCosts(a) = tourCostV(tour, cmRed, nNodes);
    end

    % ---- 方案 S：局部搜索蚂蚁选择 (精英 + 随机探索) ----
    [~, sortIdx] = sort(antCosts);

    % 拆分: 精英 (前 optEliteRatio) + 随机探索 (剩余, 上限 optRandomCap)
    nElite = round(nOptAnts * optEliteRatio);
    nRandom = min(nOptAnts - nElite, round(nAnts * optRandomCap));
    randPool = nElite + randperm(nAnts - nElite, nRandom);
    lsAnts = [sortIdx(1:nElite); sortIdx(randPool)];   % 全部局部搜索蚂蚁

    % VND 搜索 (所有 lsAnts, 候选列表加速)
    for idx = 1:length(lsAnts)
        a = lsAnts(idx);
        [antTours{a}, antCosts(a)] = vndSearchV(antTours{a}, cmRed, nNodes, isCand);
    end

    [iterBestCost, bestAntIdx] = min(antCosts);
    iterBestTour = antTours{bestAntIdx};
    improved = false;
    if iterBestCost < globalBestCost - costTol
        globalBestCost = iterBestCost;
        globalBestTour = iterBestTour;
        improved = true;
    end

    if trackHistory
        bestCostHistory(iter) = globalBestCost;
        avgCostHistory(iter) = mean(antCosts);
        timeHistory(iter) = toc(tStart);
    end

    % ---- 方案 B：自适应停止 (CV 同质 + 停滞计数) ----
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

    % --- 信息素沉积 (闭合回路全部边, 含虚拟点两侧边) ---
    deposit = Q / iterBestCost;
    cyc = [1, iterBestTour, 1];
    for k = 1:nNodes
        i = cyc(k); j = cyc(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end
    eliteDeposit = Q / globalBestCost;
    cyc = [1, globalBestTour, 1];
    for k = 1:nNodes
        i = cyc(k); j = cyc(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

    % ---- 方案 A：MMAS 限幅 + 动态更新 ----
    tau = min(tau, tauMax);
    tau = max(tau, tauMin);
    tauMax = 1 / (rho * globalBestCost);
    tauMin = tauMax / nNodes;
    tauMin = max(tauMin, 1e-6);
end

bestCost = globalBestCost;
bestOrder = [1, globalBestTour, nPts];
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
%  VND 局部搜索 (闭合回路): 2-opt → 重定位 → 交换 → 循环 (候选列表加速)
% =========================================================================
function [tour, cost] = vndSearchV(tour, cmRed, nNodes, isCand)
cost = tourCostV(tour, cmRed, nNodes);
improved = true;
while improved
    improved = false;
    [tour, cost, ok] = twoOptV(tour, cmRed, cost, isCand);
    improved = improved || ok;
    [tour, cost, ok] = relocateV(tour, cmRed, cost, isCand);
    improved = improved || ok;
    [tour, cost, ok] = swapV(tour, cmRed, cost, isCand);
    improved = improved || ok;
end
end

function c = tourCostV(tour, cmRed, nNodes)
fo = [1, tour, 1]; c = 0;
for k = 1:nNodes, c = c + cmRed(fo(k), fo(k + 1)); end
end

% ---- 2-opt (候选加速: 仅当新边涉及候选边对时评估) ----
function [tour, cost, improved] = twoOptV(tour, cmRed, cost, isCand)
nMid = length(tour);
fullOrder = [1, tour, 1];
improved = false;
for i = 1:(nMid - 1)
    for j = (i + 1):nMid
        fi = i + 1; fj = j + 1;
        u = fullOrder(fi); v = fullOrder(fj);
        up1 = fullOrder(fi + 1); vp1 = fullOrder(fj + 1);
        % 仅当新边 (u,v) 或 (up1,vp1) 至少一对是候选边时才评估
        if ~isCand(u, v) && ~isCand(up1, vp1)
            continue;
        end
        old = cmRed(u, up1) + cmRed(v, vp1);
        nw  = cmRed(u, v) + cmRed(up1, vp1);
        if nw < old - 1e-10
            tour((i + 1):j) = tour(j:-1:(i + 1));
            cost = cost - old + nw; improved = true;
            fullOrder = [1, tour, 1];
        end
    end
end
end

% ---- 节点重定位 (候选加速: 仅当插入城市是新邻居的候选邻居时评估) ----
function [tour, cost, improved] = relocateV(tour, cmRed, cost, isCand)
nMid = length(tour); improved = false;
for v = 1:nMid
    Cv = tour(v);
    if v == 1, Lv = 1; else, Lv = tour(v - 1); end
    if v == nMid, Rv = 1; else, Rv = tour(v + 1); end
    for p = 0:nMid
        if p == v || p == v - 1, continue; end
        if p == 0, Lp = 1; Rp = tour(1);
        elseif p == nMid, Lp = tour(nMid); Rp = 1;
        else, Lp = tour(p); Rp = tour(p + 1);
        end
        % 仅当 Cv 是 Lp 或 Rp 的候选邻居时才评估
        if ~isCand(Lp, Cv) && ~isCand(Rp, Cv)
            continue;
        end
        oldCost = cmRed(Lv, Cv) + cmRed(Cv, Rv) + cmRed(Lp, Rp);
        newCost = cmRed(Lv, Rv) + cmRed(Lp, Cv) + cmRed(Cv, Rp);
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

% ---- 节点交换 (候选加速: 仅当两个城市互为候选邻居时评估) ----
function [tour, cost, improved] = swapV(tour, cmRed, cost, isCand)
nMid = length(tour);
fullOrder = [1, tour, 1]; improved = false;
for i = 1:(nMid - 1)
    for j = (i + 1):nMid
        fi = i + 1; fj = j + 1;
        Ai = fullOrder(fi); Aj = fullOrder(fj);
        % 仅当两个被交换城市互为候选邻居时才评估
        if ~isCand(Ai, Aj)
            continue;
        end
        Li = fullOrder(fi - 1); Ri = fullOrder(fi + 1);
        Lj = fullOrder(fj - 1); Rj = fullOrder(fj + 1);
        if j == i + 1
            oldCost = cmRed(Li, Ai) + cmRed(Ai, Aj) + cmRed(Aj, Rj);
            newCost = cmRed(Li, Aj) + cmRed(Aj, Ai) + cmRed(Ai, Rj);
        else
            oldCost = cmRed(Li, Ai) + cmRed(Ai, Ri) + cmRed(Lj, Aj) + cmRed(Aj, Rj);
            newCost = cmRed(Li, Aj) + cmRed(Aj, Ri) + cmRed(Lj, Ai) + cmRed(Ai, Rj);
        end
        if newCost < oldCost - 1e-10
            tour([i, j]) = tour([j, i]);
            cost = cost - oldCost + newCost; improved = true;
            fullOrder = [1, tour, 1];
        end
    end
end
end
