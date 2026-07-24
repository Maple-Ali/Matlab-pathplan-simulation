%% exp_sa_v1_1_ablation — TSP_SA_v1_1 参数消融实验（bayg29）
%  方法: 一次一个参数（OPAT），其他参数固定在默认值
%  参数: alpha_beta, targetAcpt, nInnerMult, nInnerMin, nInnerPower
%  每组合: 10次独立运行，自适应停止

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

%% ===== Config =====
nRunsPerCombo = 10;
tspFile = fullfile(fileparts(mfilename('fullpath')), '..', 'bayg29.tsp');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse bayg29 =====
[fullDist29, cityCoords] = parseBayg29(tspFile);

% Build 30x30 cost matrix (start=goal=city1)
costMatrix30 = zeros(30);
costMatrix30(1, 2:29)    = fullDist29(1, 2:29);
costMatrix30(2:29, 1)    = fullDist29(2:29, 1);
costMatrix30(2:29, 30)   = fullDist29(2:29, 1);
costMatrix30(30, 2:29)   = fullDist29(1, 2:29);
costMatrix30(2:29, 2:29) = fullDist29(2:29, 2:29);
costMatrix30(30, 1)      = 0;
costMatrix30(1, 30)      = 0;

%% ===== Parameter Definitions =====
% Default values (matching TSP_SA_v1_1)
defaults = struct('alpha_beta', 0.006, 'targetAcpt', 0.32, ...
    'nInnerMult', 13, 'nInnerMin', 35, 'nInnerPower', 0.2);

paramDefs = {
    'alpha_beta',  [0.005, 0.0055, 0.006, 0.0065, 0.007, 0.0075];
    'targetAcpt',  [0.28, 0.30, 0.32, 0.34, 0.36];
    'nInnerMult',  [11, 12, 13, 14, 15];
    'nInnerMin',   [31, 33, 35, 37, 39];
    'nInnerPower', [0.16, 0.18, 0.2, 0.22, 0.24];
};
nParams = size(paramDefs, 1);

%% ===== Run Ablation =====
fprintf('======== TSP_SA_v1_1 Ablation Study: bayg29 ========\n');
fprintf('Method: One-Parameter-At-A-Time | Runs per combo: %d\n\n', nRunsPerCombo);

paramResults = cell(nParams, 1);
totalCombos = sum(cellfun(@length, paramDefs(:,2)));
comboCount = 0;

for pi = 1:nParams
    pName  = paramDefs{pi, 1};
    pVals  = paramDefs{pi, 2};
    nVals  = length(pVals);

    statsArr = struct();
    statsArr(nVals).value = [];
    statsArr(nVals).allCosts = [];
    statsArr(nVals).allConvergeIters = [];
    statsArr(nVals).allTimes = [];
    statsArr(nVals).meanCost = [];
    statsArr(nVals).stdCost = [];
    statsArr(nVals).bestCost = [];
    statsArr(nVals).meanConvIter = [];
    statsArr(nVals).meanTime = [];
    statsArr(nVals).bestOrder = [];

    for vi = 1:nVals
        comboCount = comboCount + 1;
        pv = pVals(vi);

        overrides = defaults;
        overrides.(pName) = pv;

        costs = zeros(nRunsPerCombo, 1);
        convergeIters = zeros(nRunsPerCombo, 1);
        times = zeros(nRunsPerCombo, 1);
        bestCostLocal = inf;
        bestOrderLocal = [];

        fprintf('[%2d/%2d] %s = %-8g  running %d trials...', ...
            comboCount, totalCombos, pName, pv, nRunsPerCombo);

        for r = 1:nRunsPerCombo
            [order, cost, history] = TSP_SA_v1_1_ablated(costMatrix30, 30, overrides);
            costs(r) = cost;
            convergeIters(r) = findConvergeIter(history.bestCostHistory, cost);
            times(r) = history.elapsedTime;

            if cost < bestCostLocal
                bestCostLocal = cost;
                bestOrderLocal = order;
            end
        end

        statsArr(vi).value       = pv;
        statsArr(vi).allCosts    = costs;
        statsArr(vi).allConvergeIters = convergeIters;
        statsArr(vi).allTimes    = times;
        statsArr(vi).meanCost    = mean(costs);
        statsArr(vi).stdCost     = std(costs);
        statsArr(vi).bestCost    = min(costs);
        statsArr(vi).meanConvIter = mean(convergeIters);
        statsArr(vi).meanTime    = mean(times);
        statsArr(vi).bestOrder   = bestOrderLocal;

        fprintf('  best=%.1f | avg=%.1f±%.1f | convIter=%.0f | time=%.2fs\n', ...
            min(costs), mean(costs), std(costs), mean(convergeIters), mean(times));
    end

    paramResults{pi} = struct('paramName', pName, 'values', pVals, 'stats', statsArr);
end

%% ===== Identify Best Parameters =====
fprintf('\n===== Best Parameters (by lowest mean cost) =====\n');
bestParams = struct();
for pi = 1:nParams
    pr = paramResults{pi};
    [~, bestIdx] = min([pr.stats.meanCost]);
    bestParams.(pr.paramName) = pr.values(bestIdx);
    fprintf('  %-12s = %-8g  (mean cost: %.1f)\n', pr.paramName, ...
        bestParams.(pr.paramName), pr.stats(bestIdx).meanCost);
end

%% ===== Export Data =====
resultsFile = fullfile(resultsDir, 'exp_sa_v1_1_ablation_results.mat');
save(resultsFile, 'paramResults', 'paramDefs', 'bestParams', 'defaults', ...
    'fullDist29', 'cityCoords', 'costMatrix30', 'nRunsPerCombo');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Cost Summary (5 subplots in 2x3 grid) =====
figure('Position', [50, 50, 1400, 800], 'Color', 'w');
colors = lines(nParams);

for pi = 1:nParams
    subplot(2, 3, pi);
    pr = paramResults{pi};
    pName = pr.paramName;
    vals  = pr.values(:)';
    means = [pr.stats.meanCost];
    stds  = [pr.stats.stdCost];

    errorbar(vals, means, stds, 'o-', 'Color', colors(pi,:), ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    hold on;
    [~, bestIdx] = min(means);
    plot(vals(bestIdx), means(bestIdx), 'p', 'MarkerSize', 15, ...
        'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1);

    xlabel(pName); ylabel('Best Cost');
    title(sprintf('%s  (best: %.4g @ %s=%.4g)', pName, means(bestIdx), pName, vals(bestIdx)));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_SA\_v1\_1 Ablation — Cost vs Parameter (Mean ± Std, 10 runs)', 'FontSize', 14);

%% ===== Figure 2: Convergence Speed (5 subplots) =====
figure('Position', [50, 50, 1400, 800], 'Color', 'w');

for pi = 1:nParams
    subplot(2, 3, pi);
    pr = paramResults{pi};
    pName = pr.paramName;
    vals  = pr.values(:)';
    convMeans = [pr.stats.meanConvIter];
    convStds  = zeros(1, length(pr.stats));
    for vi = 1:length(pr.stats)
        convStds(vi) = std(pr.stats(vi).allConvergeIters);
    end

    errorbar(vals, convMeans, convStds, 's-', 'Color', colors(pi,:), ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    hold on;
    [~, fastestIdx] = min(convMeans);
    plot(vals(fastestIdx), convMeans(fastestIdx), 'p', 'MarkerSize', 15, ...
        'MarkerFaceColor', [0.2, 0.7, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1);

    xlabel(pName); ylabel('Converge Iteration');
    title(sprintf('%s  (fastest: iter=%.0f @ %s=%.4g)', pName, convMeans(fastestIdx), pName, vals(fastestIdx)));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_SA\_v1\_1 Ablation — Convergence Speed vs Parameter', 'FontSize', 14);

%% ===== Figure 3: Runtime Summary (5 subplots) =====
figure('Position', [50, 50, 1400, 800], 'Color', 'w');

for pi = 1:nParams
    subplot(2, 3, pi);
    pr = paramResults{pi};
    pName = pr.paramName;
    vals  = pr.values(:)';
    timeMeans = [pr.stats.meanTime];
    timeStds  = zeros(1, length(pr.stats));
    for vi = 1:length(pr.stats)
        timeStds(vi) = std(pr.stats(vi).allTimes);
    end

    errorbar(vals, timeMeans, timeStds, 'd-', 'Color', colors(pi,:), ...
        'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    xlabel(pName); ylabel('Runtime (s)');
    title(sprintf('%s — Runtime', pName));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_SA\_v1\_1 Ablation — Runtime vs Parameter', 'FontSize', 14);

%% ===== Figure 4: Best-Parameter TSP Tour =====
fprintf('\nRunning TSP_SA_v1_1 with best parameters for tour visualization...\n');
[bestOrder, bestCost, ~] = TSP_SA_v1_1_ablated(costMatrix30, 30, bestParams);
fprintf('  Best-param cost: %.2f\n', bestCost);

figure('Position', [150, 150, 750, 680], 'Color', 'w');
hold on;
scatter(cityCoords(:,1), cityCoords(:,2), 50, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
for i = 1:29
    text(cityCoords(i,1) + 30, cityCoords(i,2) + 30, num2str(i), ...
        'FontSize', 7, 'Color', [0.2, 0.2, 0.2]);
end

cityIdx = zeros(1, 30);
for i = 1:30
    if bestOrder(i) == 1 || bestOrder(i) == 30, cityIdx(i) = 1;
    else, cityIdx(i) = bestOrder(i); end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.5);
plot(cityCoords(1,1), cityCoords(1,2), 'o', 'MarkerSize', 14, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1), cityCoords(1,2) - 130, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

bpText = sprintf('Best: \\alpha\\_\\beta=%.4g, targetAcpt=%.4g, nInnerMult=%.4g, nInnerMin=%.4g, nInnerPower=%.4g', ...
    bestParams.alpha_beta, bestParams.targetAcpt, bestParams.nInnerMult, bestParams.nInnerMin, bestParams.nInnerPower);
xlabel('X'); ylabel('Y');
title(sprintf('Best-Param TSP Tour (Cost: %.2f)\n%s', bestCost, bpText), 'FontSize', 10);
axis equal tight; grid on; set(gca, 'GridAlpha', 0.15);
hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ===== Local Functions =====

function convIter = findConvergeIter(bestCostHistory, finalCost)
    threshold = finalCost * 1.01;
    idx = find(bestCostHistory <= threshold, 1, 'first');
    if isempty(idx), convIter = length(bestCostHistory); else, convIter = idx; end
end

function [bestOrder, bestCost, history] = TSP_SA_v1_1_ablated(costMatrix, nPts, params)
%TSP_SA_V1_1_ABLATED 参数化版TSP_SA_v1_1（用于消融实验，不修改原算法）
%   params可选字段: alpha_beta, targetAcpt, nInnerMult, nInnerMin, nInnerPower

    % ===== Default parameters (matching TSP_SA_v1_1) =====
    alpha_beta  = 0.005;
    targetAcpt  = 0.50;
    nInnerMult  = 10;
    nInnerMin   = 30;
    nInnerPower = 0.5;

    % ===== Override with params =====
    if nargin >= 3 && ~isempty(params)
        if isfield(params, 'alpha_beta'),  alpha_beta  = params.alpha_beta;  end
        if isfield(params, 'targetAcpt'),  targetAcpt  = params.targetAcpt;  end
        if isfield(params, 'nInnerMult'),  nInnerMult  = params.nInnerMult;  end
        if isfield(params, 'nInnerMin'),   nInnerMin   = params.nInnerMin;   end
        if isfield(params, 'nInnerPower'), nInnerPower = params.nInnerPower; end
    end

    % ===== Start of TSP_SA_v1_1 logic (parameterized) =====
    nMid = nPts - 2;
    midIdx = 2:(nPts - 1);

    nNNRestarts = 0;
    alpha_base  = 0.995;
    adaptAlpha  = 1;
    alpha_min   = 0.990;
    alpha_max   = 0.998;
    adaptInner  = 1;
    T0_factor   = 1.0;
    T_min       = 1e-3;
    opWeights   = [1.0, 1.0, 1.0];
    opLearnRate = 0.15;
    opDecay     = 0.95;
    enableAdaptiveStop = 1;
    T_cold_factor = 0.02;
    stagnationLim = 250;
    minOuter      = 80;
    maxOuterIter  = 3000;

    trackHistory = (nargout >= 3);
    if trackHistory, tStart = tic; end

    if nMid == 0
        bestOrder = [1, nPts];
        bestCost = costMatrix(1, nPts);
        if trackHistory
            history = struct('bestCostHistory', bestCost, 'avgCostHistory', bestCost, ...
                'timeHistory', 0, 'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
        end
        return;
    end

    nStarts = nMid;
    if nNNRestarts > 0, nStarts = min(nNNRestarts, nMid); end
    startSeeds = randperm(nMid, nStarts);
    nnCosts = zeros(nStarts, 1);
    nnTours = cell(nStarts, 1);
    for sIdx = 1:nStarts
        s = startSeeds(sIdx);
        visited = false(1, nMid); visited(s) = true;
        tour = zeros(1, nMid); tour(1) = midIdx(s); cur = midIdx(s);
        for step = 2:nMid
            bestD = inf; bestJ = -1;
            for j = 1:nMid
                if ~visited(j), d = costMatrix(cur, midIdx(j));
                    if d < bestD, bestD = d; bestJ = j; end
                end
            end
            visited(bestJ) = true; tour(step) = midIdx(bestJ); cur = midIdx(bestJ);
        end
        nnCosts(sIdx) = costMatrix(1, tour(1)) + ...
            sum(arrayfun(@(k) costMatrix(tour(k), tour(k+1)), 1:(nMid-1))) + costMatrix(cur, nPts);
        nnTours{sIdx} = tour;
    end
    [~, nnBestIdx] = min(nnCosts);
    midOrder = nnTours{nnBestIdx};
    [midOrder, curCost] = twoOpt(midOrder, costMatrix, nPts);
    bestMid = midOrder; bestCost = curCost;

    if T0_factor > 0
        T0 = T0_factor * max(costMatrix(costMatrix < inf & costMatrix > 0));
    else
        T0 = max(costMatrix(costMatrix < inf & costMatrix > 0));
    end
    if isempty(T0) || T0 == 0, T0 = 100; end
    T = T0;
    nInnerBase = max(nMid * nInnerMult, 50);
    nEstOuter = ceil(log(T_min / T0) / log(alpha_min)) + 200;

    if trackHistory
        bestCostHistory = zeros(nEstOuter, 1);
        avgCostHistory = zeros(nEstOuter, 1);
        timeHistory = zeros(nEstOuter, 1);
    end

    opW = opWeights(:);
    opSuccess = zeros(3, 1); opAttempts = zeros(3, 1);
    stagnationCount = 0; stopReason = ''; prevBestCost = bestCost;
    nOuter = 0;

    while T > T_min && nOuter < maxOuterIter
        nOuter = nOuter + 1;
        sumCost = 0; nAccepted = 0;
        opSuccess(:) = 0; opAttempts(:) = 0;

        if adaptInner
            nInner = max(nInnerMin, round(nInnerBase * (T / T0)^nInnerPower));
        else
            nInner = nInnerBase;
        end

        curMid = midOrder;
        for i = 1:nInner
            rOp = rand() * sum(opW);
            cumW = cumsum(opW);
            op = find(cumW >= rOp, 1, 'first');

            switch op
                case 1  % Swap
                    pair = randperm(nMid, 2); i1 = min(pair); i2 = max(pair);
                    [newCost, delta] = deltaSwap(curMid, i1, i2, curCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        curMid([i1, i2]) = curMid([i2, i1]); curCost = newCost;
                        nAccepted = nAccepted + 1; opSuccess(1) = opSuccess(1) + 1;
                    end
                    opAttempts(1) = opAttempts(1) + 1;
                case 2  % Insert
                    pos = randperm(nMid, 2); fromPos = min(pos); toPos = max(pos);
                    [newCost, delta] = deltaInsert(curMid, fromPos, toPos, curCost, costMatrix, nMid, nPts);
                    if delta < 0 || rand < exp(-delta / T)
                        val = curMid(fromPos);
                        curMid(fromPos:toPos-1) = curMid(fromPos+1:toPos);
                        curMid(toPos) = val; curCost = newCost;
                        nAccepted = nAccepted + 1; opSuccess(2) = opSuccess(2) + 1;
                    end
                    opAttempts(2) = opAttempts(2) + 1;
                case 3  % 2-opt
                    pair = sort(randi(nMid, [1, 2])); i1 = pair(1); i2 = pair(2);
                    if i1 < i2
                        [newCost, delta] = delta2opt(curMid, i1, i2, curCost, costMatrix, nMid, nPts);
                        if delta < 0 || rand < exp(-delta / T)
                            curMid(i1:i2) = curMid(i2:-1:i1); curCost = newCost;
                            nAccepted = nAccepted + 1; opSuccess(3) = opSuccess(3) + 1;
                        end
                        opAttempts(3) = opAttempts(3) + 1;
                    end
            end
            sumCost = sumCost + curCost;
            if curCost < bestCost, bestCost = curCost; bestMid = curMid; end
        end

        midOrder = curMid;

        if trackHistory
            bestCostHistory(nOuter) = bestCost;
            avgCostHistory(nOuter) = sumCost / nInner;
            timeHistory(nOuter) = toc(tStart);
        end

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

        for o = 1:3
            if opAttempts(o) > 0
                succRate = opSuccess(o) / opAttempts(o);
                opW(o) = opW(o) * opDecay + succRate * opLearnRate;
                opW(o) = max(opW(o), 0.01);
            end
        end

        if adaptAlpha
            acptRate = nAccepted / nInner;
            alpha = alpha_base - alpha_beta * (acptRate - targetAcpt);
            alpha = max(alpha_min, min(alpha_max, alpha));
        else
            alpha = alpha_base;
        end

        T = T * alpha;
    end

    [bestMid, bestCost] = twoOpt(bestMid, costMatrix, nPts);
    bestOrder = [1, bestMid, nPts];

    if trackHistory
        history = struct();
        history.bestCostHistory = bestCostHistory(1:nOuter);
        history.avgCostHistory = avgCostHistory(1:nOuter);
        history.timeHistory = timeHistory(1:nOuter);
        history.iterCount = nOuter;
        history.elapsedTime = toc(tStart);
        if isempty(stopReason)
            if nOuter >= maxOuterIter, stopReason = 'maxOuter硬上限';
            else, stopReason = sprintf('T_min(%.2g)', T_min); end
        end
        history.stopReason = stopReason;
    end
end

% ===== SA Delta Functions (inline with TSP_SA_v1_1_ablated) =====

function [newCost, delta] = deltaSwap(midOrder, i, j, curCost, costMatrix, nMid, nPts)
    Li = getLeft(midOrder, i); Ri = getRight(midOrder, i, nMid, nPts);
    Lj = getLeft(midOrder, j); Rj = getRight(midOrder, j, nMid, nPts);
    Ai = midOrder(i); Aj = midOrder(j);
    if j == i + 1
        oldEdges = costMatrix(Li, Ai) + costMatrix(Ai, Aj) + costMatrix(Aj, Rj);
        newEdges = costMatrix(Li, Aj) + costMatrix(Aj, Ai) + costMatrix(Ai, Rj);
    else
        oldEdges = costMatrix(Li, Ai) + costMatrix(Ai, Ri) + costMatrix(Lj, Aj) + costMatrix(Aj, Rj);
        newEdges = costMatrix(Li, Aj) + costMatrix(Aj, Ri) + costMatrix(Lj, Ai) + costMatrix(Ai, Rj);
    end
    delta = newEdges - oldEdges; newCost = curCost + delta;
end

function [newCost, delta] = deltaInsert(midOrder, from, to, curCost, costMatrix, nMid, nPts)
    X = midOrder(from); L = getLeft(midOrder, from);
    B = midOrder(from + 1); D = midOrder(to);
    R = getRight(midOrder, to, nMid, nPts);
    oldEdges = costMatrix(L, X) + costMatrix(X, B) + costMatrix(D, R);
    newEdges = costMatrix(L, B) + costMatrix(D, X) + costMatrix(X, R);
    delta = newEdges - oldEdges; newCost = curCost + delta;
end

function [newCost, delta] = delta2opt(midOrder, i, j, curCost, costMatrix, nMid, nPts)
    L = getLeft(midOrder, i); A = midOrder(i);
    B = midOrder(j); R = getRight(midOrder, j, nMid, nPts);
    oldEdges = costMatrix(L, A) + costMatrix(B, R);
    newEdges = costMatrix(L, B) + costMatrix(A, R);
    delta = newEdges - oldEdges; newCost = curCost + delta;
end

function L = getLeft(order, pos)
    if pos == 1, L = 1; else, L = order(pos - 1); end
end

function R = getRight(order, pos, nMid, nPts)
    if pos == nMid, R = nPts; else, R = order(pos + 1); end
end

function [order, cost] = twoOpt(order, costMatrix, nPts)
    nMid = length(order); fullOrder = [1, order, nPts];
    cost = 0;
    for k = 1:(nPts - 1), cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1)); end
    improved = true;
    while improved
        improved = false;
        for i = 1:(nMid - 1)
            for j = (i + 1):nMid
                fi = i + 1; fj = j + 1;
                old = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + costMatrix(fullOrder(fj), fullOrder(fj + 1));
                new = costMatrix(fullOrder(fi), fullOrder(fj)) + costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
                if new < old - 1e-10
                    order((i + 1):j) = order(j:-1:(i + 1));
                    cost = cost - old + new; improved = true;
                    fullOrder = [1, order, nPts];
                end
            end
        end
    end
end

function [distMatrix, coords] = parseBayg29(filepath)
    fid = fopen(filepath, 'r');
    if fid < 0, error('Cannot open file: %s', filepath); end
    cleanup = onCleanup(@() fclose(fid));
    nCities = 29; distMatrix = zeros(nCities); coords = zeros(nCities, 2);
    inEdgeSection = false; inDisplaySection = false;
    rowVals = cell(nCities - 1, 1); currentRow = 1;
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if startsWith(line, 'EDGE_WEIGHT_SECTION'), inEdgeSection = true; continue;
        elseif startsWith(line, 'DISPLAY_DATA_SECTION'), inEdgeSection = false; inDisplaySection = true; continue;
        elseif strcmp(line, 'EOF'), break; end
        if inEdgeSection && ~isempty(line) && ~startsWith(line, 'DISPLAY')
            nums = sscanf(line, '%f');
            if ~isempty(nums), rowVals{currentRow} = nums; currentRow = currentRow + 1; end
        elseif inDisplaySection && ~isempty(line) && ~startsWith(line, 'EOF')
            parts = sscanf(line, '%f');
            if length(parts) >= 3, coords(parts(1), :) = [parts(2), parts(3)]; end
        end
    end
    for i = 1:(nCities - 1)
        vals = rowVals{i};
        for k = 1:length(vals), j = i + k; distMatrix(i, j) = vals(k); distMatrix(j, i) = vals(k); end
    end
end
