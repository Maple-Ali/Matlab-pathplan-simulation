function [bestOrder, bestCost, history] = TSP_ACO_v2_4(costMatrix, nPts)
%TSP_ACO_V2_4 蚁群 + 虚拟节点(哑节点) + 候选列表加速 VND 求解 TSP
%   基于 TSP_ACO_v2_3，核心升级:
%     V. 虚拟节点法(标准哑节点): 新增虚拟节点 V = nPts+1, 将其与起点(节点1)
%        和终点(节点nPts)分别用 0 距离连接, 与所有中间点用极大惩罚 BIG 连接,
%        把"固定起点终点的路径 TSP"转化为"闭环 TSP"。
%
%   步骤:
%     1. 构造扩展距离矩阵 costExt (nPts+1 个节点):
%          costExt(V,1)=costExt(1,V)=0            (虚拟点→起点)
%          costExt(V,nPts)=costExt(nPts,V)=0      (虚拟点→终点)
%          costExt(V,midIdx)=costExt(midIdx,V)=BIG (虚拟点→中间点, 极大惩罚)
%          其他节点间距离不变
%     2. 在扩展矩阵上运行闭环 TSP: 蚂蚁构造、VND 局部搜索、信息素更新
%        均基于 costExt, 求解包含所有节点的最短回路
%     3. 从最优回路提取开环路径: 定位虚拟点, 去掉后按方向整理出
%        起点→终点路径
%
%   与 v2_3 的区别: v2_3 将起点/终点固定为路径两端、信息素只覆盖中间点;
%                   v2_4 引入虚拟点后, 起点/终点/中间点均为闭合回路中的普通节点,
%                   信息素覆盖全部 nPts+1 个节点, 局部搜索在完整回路上进行。
%
%   保留 v2_3 全部机制: MMAS / C(候选列表) / K(伪随机) / S(S曲线) / B(自适应停止)

nMid = nPts - 2;
midIdx = 2:(nPts - 1);
nNodes = nPts + 1;          % 节点数 (含虚拟点)
Vidx  = nPts + 1;           % 虚拟节点索引

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
optRatio_end   = 0.6;        % 后期局部搜索比例 (y_max)
optTransInterval = [0, 50]; % 过渡区间 [x_L, x_R]
optCurveA      = 2.0;        % S 曲线陡峭度
optEliteRatio  = 0.7;        % 局部搜索预算中精英蚂蚁占比
% 随机抽取 = 局部搜索预算 - 精英数量, 不再单独设上限

% ===== 方案 C：候选列表加速 VND =====
kCand = 9;               % 每个节点的候选邻居数

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;  % 1=开启
cvThreshold  = 0.001;    % 种群 CV 阈值 (CV < 阈值 → 种群同质停止)
stagnationLim = 100;      % 最优解连续停滞上限 (代)
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

% =========================================================================
%  步骤 1：构造扩展距离矩阵 costExt (nPts+1 个节点)
% =========================================================================
finitePos = costMatrix(costMatrix > 0 & isfinite(costMatrix));
if isempty(finitePos), maxFinite = 1; else, maxFinite = max(finitePos); end
BIG = (nPts + 1) * maxFinite * 2;   % 极大惩罚: 严格大于任何可行回路成本

costExt = zeros(nNodes);
costExt(1:nPts, 1:nPts) = costMatrix;         % 其他节点间距离不变
costExt(Vidx, 1) = 0;  costExt(1, Vidx) = 0;  % 虚拟点 → 起点
costExt(Vidx, nPts) = 0; costExt(nPts, Vidx) = 0;  % 虚拟点 → 终点
costExt(Vidx, midIdx) = BIG; costExt(midIdx, Vidx) = BIG;  % 虚拟点 → 中间点

% ---- 方案 C：预计算候选列表 (基于 costExt) ----
isCand = false(nNodes, nNodes);
for i = 1:nNodes
    dists = costExt(i, :);
    dists(i) = inf;                    % 排除自身
    [~, idx] = sort(dists);
    isCand(i, idx(1:min(kCand, nNodes-1))) = true;
end

% 启发式信息矩阵 eta = 1/d
% 虚拟点到起点/终点的 0 距离边: 1/0 → 用极大值 BIG 表示"必连"
eta = zeros(nNodes, nNodes);
for i = 1:nNodes
    for j = 1:nNodes
        if i ~= j
            d = costExt(i, j);
            if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
        end
    end
end
eta(Vidx, 1) = BIG; eta(1, Vidx) = BIG;   % 虚拟点 ↔ 起点 (0 距离 → 极大吸引)
eta(Vidx, nPts) = BIG; eta(nPts, Vidx) = BIG;  % 虚拟点 ↔ 终点

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
globalBestTour = 1:nNodes;
costTol = 1e-9;           % 成本比较容差 (消除浮点累加噪声)
stagnationCount = 0;
stopReason = '';

% =========================================================================
%  步骤 2：在扩展矩阵上运行闭环 TSP
% =========================================================================
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

    % ---- 蚂蚁构造: 包含所有 nNodes 节点的闭合回路 ----
    for a = 1:nAnts
        cycle = zeros(1, nNodes);
        visited = false(1, nNodes);
        start = randi(nNodes);
        cycle(1) = start;
        visited(start) = true;

        for step = 2:nNodes
            curIdx = cycle(step - 1);
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
            cycle(step) = nxt;
            visited(nxt) = true;
        end

        antTours{a} = cycle;
        antCosts(a) = cycleCost(cycle, costExt, nNodes);
    end

    % ---- 方案 S：局部搜索蚂蚁选择 (精英 + 随机探索) ----
    [~, sortIdx] = sort(antCosts);

    % 拆分: 精英 (前 optEliteRatio) + 随机探索 (剩余 = nOptAnts - nElite)
    nElite = round(nOptAnts * optEliteRatio);
    nRandom = nOptAnts - nElite;
    randPool = nElite + randperm(nAnts - nElite, nRandom);
    lsAnts = [sortIdx(1:nElite); sortIdx(randPool)];   % 全部局部搜索蚂蚁

    % VND 搜索 (闭合回路, 候选列表加速)
    for idx = 1:length(lsAnts)
        a = lsAnts(idx);
        [antTours{a}, antCosts(a)] = vndSearchClosed(antTours{a}, costExt, nNodes, isCand);
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
    for k = 1:nNodes
        i = iterBestTour(k);
        j = iterBestTour(mod(k, nNodes) + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end
    eliteDeposit = Q / globalBestCost;
    for k = 1:nNodes
        i = globalBestTour(k);
        j = globalBestTour(mod(k, nNodes) + 1);
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

% =========================================================================
%  步骤 3：从最优闭合回路提取开环路径
% =========================================================================
bestCost = globalBestCost;
bestOrder = extractPath(globalBestTour, nPts, Vidx);
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
%  提取开环路径: 定位虚拟点, 去掉后按方向整理出 起点→终点 路径
% =========================================================================
function order = extractPath(cycle, nPts, Vidx)
n = length(cycle);
vPos = find(cycle == Vidx, 1);
% 从虚拟点顺时针下一节点开始走一圈, 收集去掉虚拟点后的 nPts 个节点
order = zeros(1, nPts);
idx = mod(vPos, n) + 1;
for t = 1:nPts
    order(t) = cycle(idx);
    idx = mod(idx, n) + 1;
end
% 方向校正: 若终点 nPts 在首位则反转 (使起点 1 在前、终点 nPts 在后)
if order(1) == nPts
    order = fliplr(order);
end
% 起点校正: 若 1 不在首位 (罕见不可行情形, 保险), 旋转使 1 在首位
if order(1) ~= 1
    p1 = find(order == 1, 1);
    order = [order(p1:end), order(1:p1-1)];
end
end

% =========================================================================
%  VND 局部搜索 (闭合回路): 2-opt → 重定位 → 交换 → 循环 (候选列表加速)
% =========================================================================
function [cycle, cost] = vndSearchClosed(cycle, costExt, nNodes, isCand)
cost = cycleCost(cycle, costExt, nNodes);
improved = true;
while improved
    improved = false;
    [cycle, cost, ok] = twoOptClosed(cycle, costExt, cost, isCand);
    improved = improved || ok;
    [cycle, cost, ok] = relocateClosed(cycle, costExt, cost, isCand);
    improved = improved || ok;
    [cycle, cost, ok] = swapClosed(cycle, costExt, cost, isCand);
    improved = improved || ok;
end
% 重新精确计算成本, 避免回路含 Inf 边被移除时 cost=Inf-Inf+finite=NaN 污染
cost = cycleCost(cycle, costExt, nNodes);
end

function c = cycleCost(cycle, costExt, nNodes)
c = 0;
for k = 1:nNodes
    c = c + costExt(cycle(k), cycle(mod(k, nNodes) + 1));
end
end

% ---- 2-opt (闭合回路, 候选加速) ----
function [cycle, cost, improved] = twoOptClosed(cycle, costExt, cost, isCand)
n = length(cycle);
full = [cycle, cycle(1)];
improved = false;
for i = 1:(n - 1)
    for j = (i + 2):n
        u = full(i); v = full(i + 1);
        x = full(j); y = full(j + 1);
        % 仅当新边 (u,x) 或 (v,y) 至少一对是候选边时才评估
        if ~isCand(u, x) && ~isCand(v, y)
            continue;
        end
        old = costExt(u, v) + costExt(x, y);
        nw  = costExt(u, x) + costExt(v, y);
        if nw < old - 1e-10
            full((i + 1):j) = full(j:-1:(i + 1));
            cycle = full(1:n);
            cost = cost - old + nw; improved = true;
            return;
        end
    end
end
end

% ---- 节点重定位 (闭合回路, 候选加速) ----
function [cycle, cost, improved] = relocateClosed(cycle, costExt, cost, isCand)
n = length(cycle); improved = false;
for v = 1:n
    Cv = cycle(v);
    Lv = cycle(mod(v - 2, n) + 1);
    Rv = cycle(mod(v, n) + 1);
    for p = 1:n
        % 跳过退化插入点: p==v (插到自身前) 或 p==v-1 循环意义下 (插到自身后, Rp==Cv)
        if p == v || p == mod(v - 2, n) + 1, continue; end
        Lp = cycle(p);
        Rp = cycle(mod(p, n) + 1);
        % 仅当 Cv 是 Lp 或 Rp 的候选邻居时才评估
        if ~isCand(Lp, Cv) && ~isCand(Rp, Cv)
            continue;
        end
        oldCost = costExt(Lv, Cv) + costExt(Cv, Rv) + costExt(Lp, Rp);
        newCost = costExt(Lv, Rv) + costExt(Lp, Cv) + costExt(Cv, Rp);
        if newCost < oldCost - 1e-10
            cityVal = cycle(v);
            if p < v
                newCycle = [cycle(1:p), cityVal, cycle(p+1:v-1), cycle(v+1:end)];
            else
                newCycle = [cycle(1:v-1), cycle(v+1:p), cityVal, cycle(p+1:end)];
            end
            cycle = newCycle; cost = cost - oldCost + newCost; improved = true; return;
        end
    end
end
end

% ---- 节点交换 (闭合回路, 候选加速) ----
function [cycle, cost, improved] = swapClosed(cycle, costExt, cost, isCand)
n = length(cycle);
full = [cycle, cycle(1)]; improved = false;
for i = 1:(n - 1)
    for j = (i + 1):n
        Ai = full(i); Aj = full(j);
        % 仅当两个被交换节点互为候选邻居时才评估
        if ~isCand(Ai, Aj)
            continue;
        end
        if j == i + 1
            % 相邻交换 (线性相邻)
            if i == 1, Li = full(n); else, Li = full(i - 1); end
            Rj = full(j + 1);
            oldCost = costExt(Li, Ai) + costExt(Ai, Aj) + costExt(Aj, Rj);
            newCost = costExt(Li, Aj) + costExt(Aj, Ai) + costExt(Ai, Rj);
        elseif i == 1 && j == n
            % 相邻交换 (跨闭合边: cycle(n)-cycle(1) 相邻, 对序为 (cycle(n), cycle(1)))
            Li = full(n - 1);
            Rj = full(2);
            oldCost = costExt(Li, Aj) + costExt(Aj, Ai) + costExt(Ai, Rj);
            newCost = costExt(Li, Ai) + costExt(Ai, Aj) + costExt(Aj, Rj);
        else
            % 非相邻交换
            if i == 1, Li = full(n); else, Li = full(i - 1); end
            Ri = full(i + 1);
            Lj = full(j - 1);
            Rj = full(j + 1);
            oldCost = costExt(Li, Ai) + costExt(Ai, Ri) + costExt(Lj, Aj) + costExt(Aj, Rj);
            newCost = costExt(Li, Aj) + costExt(Aj, Ri) + costExt(Lj, Ai) + costExt(Ai, Rj);
        end
        if newCost < oldCost - 1e-10
            cycle([i, j]) = cycle([j, i]);
            cost = cost - oldCost + newCost; improved = true;
            full = [cycle, cycle(1)];
        end
    end
end
end
