function [bestOrder, bestCost, history] = TSP_ACO_v2_3AS(costMatrix, nPts)
%TSP_ACO_V2_3AS 蚁群 + 候选列表加速 VND (消融版: 无 MMAS) 求解 TSP
%   基于 TSP_ACO_v2_3，移除:
%     A. MMAS 信息素限幅 — 删除, 用于消融实验对比 MMAS 效益
%
%   保留 v2_3 其余全部机制: VND / C / K / S / B

nMid = nPts - 2;
midIdx = 2:(nPts - 1);

% ===== 算法基础参数 =====
nAnts = 40;              % 蚂蚁数量
nIter = 800;             % 最大迭代次数
alpha = 1;               % 信息素权重
beta  = 2.0;             % 启发式信息权重
rho   = 0.25;            % 信息素蒸发率
Q     = 225;             % 信息素沉积常数

% ===== 方案 K：伪随机比例规则 (ACS 风格) =====
enablePseudoRandom = 0;  % 1=开启
q0 = 0.45;               % 贪心选择概率

% ===== 方案 S：渐进式局部搜索 (S 曲线) =====
optRatio_start = 0.00;       % 初期局部搜索比例 (y_min)
optRatio_end   = 0.3;       % 后期局部搜索比例 (y_max)
optTransInterval = [0, 100]; % 过渡区间 [x_L, x_R]
optCurveA      = 2.0;        % S 曲线陡峭度
optEliteRatio  = 0.6;       % 局部搜索预算中精英蚂蚁占比
optRandomCap   = 0.15;       % 随机抽取比例上限

% ===== 方案 C：候选列表加速 VND =====
kCand = 50;               % 每个城市的候选邻居数 (10~15)

% ===== 方案 B：自适应停止 =====
enableAdaptiveStop = 1;  % 1=开启
cvThreshold  = 0.001;    % 种群 CV 阈值 (CV < 阈值 → 种群同质停止)
stagnationLim = 200;      % 最优解连续停滞上限 (代)
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

% ---- 方案 C：预计算候选列表 ----
% isCand(i,j) = true 若 j 是 i 的 k 个最近邻居之一
isCand = false(nPts, nPts);
for i = 1:nPts
    dists = costMatrix(i, :);
    dists(i) = inf;                    % 排除自身
    [~, idx] = sort(dists);
    isCand(i, idx(1:min(kCand, nPts-1))) = true;
end

% 启发式信息矩阵
eta = zeros(nMid, nMid);
for i = 1:nMid
    for j = 1:nMid
        if i ~= j
            d = costMatrix(midIdx(i), midIdx(j));
            if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
        end
    end
end

% ---- 均匀初始信息素 (无外部启发式) ----
tau0 = 0.1;
tau = ones(nMid, nMid) * tau0;

globalBestCost = inf;
globalBestTour = midIdx;
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
        visited = false(1, nMid);
        start = randi(nMid);
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
        antCosts(a) = tourCostLocal(tour, midIdx, costMatrix, nPts);
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
        [antTours{a}, antCosts(a)] = vndSearchLocal(antTours{a}, midIdx, costMatrix, nPts, isCand);
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

    % --- 信息素沉积 ---
    deposit = Q / iterBestCost;
    for k = 1:(length(iterBestTour) - 1)
        i = iterBestTour(k); j = iterBestTour(k + 1);
        tau(i, j) = tau(i, j) + deposit;
        tau(j, i) = tau(j, i) + deposit;
    end
    eliteDeposit = Q / globalBestCost;
    for k = 1:(length(globalBestTour) - 1)
        i = globalBestTour(k); j = globalBestTour(k + 1);
        tau(i, j) = tau(i, j) + eliteDeposit;
        tau(j, i) = tau(j, i) + eliteDeposit;
    end

end

bestCost = globalBestCost;
bestOrder = [1, midIdx(globalBestTour), nPts];
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
%  VND 局部搜索: 2-opt → 重定位 → 交换 → 循环 (候选列表加速)
% =========================================================================
function [tour, cost] = vndSearchLocal(tour, midIdx, costMatrix, nPts, isCand)
cost = tourCostLocal(tour, midIdx, costMatrix, nPts);
improved = true;
while improved
    improved = false;
    [tour, cost, ok] = twoOptFI(tour, midIdx, costMatrix, nPts, cost, isCand);
    improved = improved || ok;
    [tour, cost, ok] = relocateFI(tour, midIdx, costMatrix, nPts, cost, isCand);
    improved = improved || ok;
    [tour, cost, ok] = swapFI(tour, midIdx, costMatrix, nPts, cost, isCand);
    improved = improved || ok;
end
end

function c = tourCostLocal(tour, midIdx, costMatrix, nPts)
fo = [1, midIdx(tour), nPts]; c = 0;
for k = 1:(nPts - 1), c = c + costMatrix(fo(k), fo(k + 1)); end
end

% ---- 2-opt (候选加速: 仅当新边涉及候选边对时评估) ----
function [tour, cost, improved] = twoOptFI(tour, midIdx, costMatrix, nPts, cost, isCand)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts];
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
        old = costMatrix(u, up1) + costMatrix(v, vp1);
        nw  = costMatrix(u, v) + costMatrix(up1, vp1);
        if nw < old - 1e-10
            tour((i + 1):j) = tour(j:-1:(i + 1));
            cost = cost - old + nw; improved = true;
            fullOrder = [1, midIdx(tour), nPts];
        end
    end
end
end

% ---- 节点重定位 (候选加速: 仅当插入城市是新邻居的候选邻居时评估) ----
function [tour, cost, improved] = relocateFI(tour, midIdx, costMatrix, nPts, cost, isCand)
nMid = length(tour); improved = false;
for v = 1:nMid
    Cv = midIdx(tour(v));
    if v == 1, Lv = 1; else, Lv = midIdx(tour(v - 1)); end
    if v == nMid, Rv = nPts; else, Rv = midIdx(tour(v + 1)); end
    for p = 0:nMid
        if p == v || p == v - 1, continue; end
        if p == 0, Lp = 1; Rp = midIdx(tour(1));
        elseif p == nMid, Lp = midIdx(tour(nMid)); Rp = nPts;
        else, Lp = midIdx(tour(p)); Rp = midIdx(tour(p + 1));
        end
        % 仅当 Cv 是 Lp 或 Rp 的候选邻居时才评估
        if ~isCand(Lp, Cv) && ~isCand(Rp, Cv)
            continue;
        end
        oldCost = costMatrix(Lv, Cv) + costMatrix(Cv, Rv) + costMatrix(Lp, Rp);
        newCost = costMatrix(Lv, Rv) + costMatrix(Lp, Cv) + costMatrix(Cv, Rp);
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
function [tour, cost, improved] = swapFI(tour, midIdx, costMatrix, nPts, cost, isCand)
nMid = length(tour);
fullOrder = [1, midIdx(tour), nPts]; improved = false;
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
            oldCost = costMatrix(Li, Ai) + costMatrix(Ai, Aj) + costMatrix(Aj, Rj);
            newCost = costMatrix(Li, Aj) + costMatrix(Aj, Ai) + costMatrix(Ai, Rj);
        else
            oldCost = costMatrix(Li, Ai) + costMatrix(Ai, Ri) + costMatrix(Lj, Aj) + costMatrix(Aj, Rj);
            newCost = costMatrix(Li, Aj) + costMatrix(Aj, Ri) + costMatrix(Lj, Ai) + costMatrix(Ai, Rj);
        end
        if newCost < oldCost - 1e-10
            tour([i, j]) = tour([j, i]);
            cost = cost - oldCost + newCost; improved = true;
            fullOrder = [1, midIdx(tour), nPts];
        end
    end
end
end
