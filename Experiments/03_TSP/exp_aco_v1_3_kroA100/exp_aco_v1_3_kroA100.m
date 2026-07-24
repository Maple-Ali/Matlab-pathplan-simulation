%% exp_aco_v1_3_kroA100 — TSP_ACO_v1_3 参数敏感性分析（kroA100）
%  方法: 一次一个参数（OPAT），其他参数固定在默认值
%  参数: nAnts, alpha, beta, rho, Q, optRatio
%  每组合: 10次独立运行，自适应停止

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

%% ===== Config =====
nRunsPerCombo = 10;
tspFile  = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA100.tsp');
distFile = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA100_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse kroA100 dataset =====
[fullDist100, cityCoords] = parseKroA100(distFile, tspFile);
nCities = 100;

nPts = nCities + 1;  % 101
costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist100(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist100(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist100(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist100(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist100(2:nCities, 2:nCities);

%% ===== Parameter Definitions =====
% Default values (matching TSP_ACO_v1_3)
defaults = struct('nAnts', 60, 'alpha', 1.0, 'beta', 2.0, ...
    'rho', 0.5, 'Q', 150, 'optRatio', 0.30);

paramDefs = {
    'nAnts',    [20, 30, 40, 50, 60, 70];
    'alpha',    [0.5, 1.0, 1.5, 2.0, 3.0, 4.0];
    'beta',     [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    'rho',      [0.1, 0.3, 0.5, 0.7, 0.9];
    'Q',        [50, 100, 150, 200, 250];
    'optRatio', [0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0];
};
nParams = size(paramDefs, 1);

%% ===== Run Ablation =====
fprintf('======== TSP_ACO_v1_3 Sensitivity: kroA100 ========\n');
fprintf('Method: One-Parameter-At-A-Time | Runs per combo: %d\n', nRunsPerCombo);
fprintf('Known optimal: 21282\n\n');

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
            [order, cost, history] = TSP_ACO_v1_3_ablated(costMatrix, nPts, overrides);
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
    fprintf('  %-10s = %-8g  (mean cost: %.1f, gap: %.1f%%)\n', pr.paramName, ...
        bestParams.(pr.paramName), pr.stats(bestIdx).meanCost, ...
        (pr.stats(bestIdx).meanCost - 21282) / 21282 * 100);
end

%% ===== Export Data =====
resultsFile = fullfile(resultsDir, 'exp_aco_v1_3_kroA100_results.mat');
save(resultsFile, 'paramResults', 'paramDefs', 'bestParams', 'defaults', ...
    'fullDist100', 'cityCoords', 'costMatrix', 'nRunsPerCombo');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Cost Summary (2x3 grid, 6 params) =====
figure('Position', [50, 50, 1400, 900], 'Color', 'w');
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
sgtitle('TSP\_ACO\_v1\_3 Sensitivity (kroA100) — Cost vs Parameter', 'FontSize', 14);

%% ===== Figure 2: Convergence Speed (2x3 grid) =====
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

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
sgtitle('TSP\_ACO\_v1\_3 Sensitivity (kroA100) — Convergence Speed', 'FontSize', 14);

%% ===== Figure 3: Runtime Summary (2x3 grid) =====
figure('Position', [50, 50, 1400, 900], 'Color', 'w');

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
sgtitle('TSP\_ACO\_v1\_3 Sensitivity (kroA100) — Runtime', 'FontSize', 14);

%% ===== Figure 4: Best-Parameter TSP Tour =====
fprintf('\nRunning TSP_ACO_v1_3 with best parameters for tour visualization...\n');
[bestOrder, bestCost, ~] = TSP_ACO_v1_3_ablated(costMatrix, nPts, bestParams);
fprintf('  Best-param cost: %.2f (gap: %.1f%%)\n', bestCost, (bestCost - 21282) / 21282 * 100);

figure('Position', [150, 150, 750, 680], 'Color', 'w');
hold on;
scatter(cityCoords(:,1), cityCoords(:,2), 30, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'none');
for i = 1:nCities
    text(cityCoords(i,1) + 20, cityCoords(i,2) + 20, num2str(i), ...
        'FontSize', 5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end

cityIdx = zeros(1, nPts);
for i = 1:nPts
    if bestOrder(i) == 1 || bestOrder(i) == nPts, cityIdx(i) = 1;
    else, cityIdx(i) = bestOrder(i); end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);
plot(cityCoords(1,1), cityCoords(1,2), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1) + 40, cityCoords(1,2) - 60, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

bpText = sprintf('Best: nAnts=%.4g, \\alpha=%.4g, \\beta=%.4g, \\rho=%.4g, Q=%.4g, optRatio=%.4g', ...
    bestParams.nAnts, bestParams.alpha, bestParams.beta, ...
    bestParams.rho, bestParams.Q, bestParams.optRatio);
xlabel('X'); ylabel('Y');
title(sprintf('Best-Param TSP Tour (Cost: %.2f, gap: %.1f%%)\n%s', ...
    bestCost, (bestCost - 21282) / 21282 * 100, bpText), 'FontSize', 9);
axis equal tight; grid on; set(gca, 'GridAlpha', 0.1);
hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ===== Local Functions =====

function convIter = findConvergeIter(bestCostHistory, finalCost)
    threshold = finalCost * 1.01;
    idx = find(bestCostHistory <= threshold, 1, 'first');
    if isempty(idx), convIter = length(bestCostHistory); else, convIter = idx; end
end

function [bestOrder, bestCost, history] = TSP_ACO_v1_3_ablated(costMatrix, nPts, params)
    % Default parameters (matching TSP_ACO_v1_3)
    nAnts = 50; nIter = 500; alpha = 1.0; beta = 2.0;
    rho = 0.5; Q = 100; optRatio = 0.30;

    if nargin >= 3 && ~isempty(params)
        if isfield(params, 'nAnts'),    nAnts    = params.nAnts;    end
        if isfield(params, 'nIter'),    nIter    = params.nIter;    end
        if isfield(params, 'alpha'),    alpha    = params.alpha;    end
        if isfield(params, 'beta'),     beta     = params.beta;     end
        if isfield(params, 'rho'),      rho      = params.rho;      end
        if isfield(params, 'Q'),        Q        = params.Q;        end
        if isfield(params, 'optRatio'), optRatio = params.optRatio; end
    end

    nMid = nPts - 2; midIdx = 2:(nPts - 1);
    enableSelectiveOpt = 1; enableLocalUpdate = 0; xi = 0.10;
    enableAdaptiveStop = 1; cvThreshold = 0.001; stagnationLim = 80; minIter = 40;

    trackHistory = (nargout >= 3);
    if trackHistory, tStart = tic;
        bestCostHistory = zeros(nIter, 1);
        avgCostHistory = zeros(nIter, 1); timeHistory = zeros(nIter, 1);
    end

    if nMid == 0
        bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
        if trackHistory
            history = struct('bestCostHistory', bestCost, 'avgCostHistory', bestCost, ...
                'timeHistory', 0, 'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial');
        end
        return;
    end

    eta = zeros(nMid, nMid);
    for i = 1:nMid
        for j = 1:nMid
            if i ~= j, d = costMatrix(midIdx(i), midIdx(j));
                if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end
            end
        end
    end

    nnCosts = zeros(nMid, 1); nnTours = cell(nMid, 1);
    for s = 1:nMid
        visited = false(1, nMid); visited(s) = true;
        tour = zeros(1, nMid); tour(1) = s; cur = s;
        for step = 2:nMid
            bestD = inf; bestJ = -1;
            for j = 1:nMid
                if ~visited(j), d = costMatrix(midIdx(cur), midIdx(j));
                    if d < bestD, bestD = d; bestJ = j; end
                end
            end
            visited(bestJ) = true; tour(step) = bestJ; cur = bestJ;
        end
        nnCosts(s) = costMatrix(1, midIdx(s)) + ...
            sum(arrayfun(@(k) costMatrix(midIdx(tour(k)), midIdx(tour(k+1))), 1:(nMid-1))) + ...
            costMatrix(midIdx(cur), nPts);
        nnTours{s} = tour;
    end
    [~, nnBestIdx] = min(nnCosts); nnBestTour = nnTours{nnBestIdx};
    [nnBestTour, nnBestCost] = twoOpt(nnBestTour, midIdx, costMatrix, nPts);

    tau0 = Q / nnBestCost; tau = ones(nMid, nMid) * tau0;
    tauMax = 1 / (rho * nnBestCost); tauMin = tauMax / (2 * nMid);
    tauMin = max(tauMin, 1e-6); tau = min(tau, tauMax); tau = max(tau, tauMin);
    globalBestCost = nnBestCost; globalBestTour = nnBestTour;
    stagnationCount = 0; stopReason = '';
    nOptAnts = max(1, round(nAnts * optRatio));

    for iter = 1:nIter
        antTours = cell(nAnts, 1); antCosts = zeros(nAnts, 1);
        for a = 1:nAnts
            tour = zeros(1, nMid); visited = false(1, nMid);
            start = randi(nMid); tour(1) = start; visited(start) = true;
            for step = 2:nMid
                curIdx = tour(step - 1); prob = zeros(1, nMid);
                for j = 1:nMid
                    if ~visited(j)
                        prob(j) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta);
                    end
                end
                totalProb = sum(prob);
                if totalProb == 0, unv = find(~visited); nxt = unv(randi(length(unv)));
                else, prob = prob / totalProb; r = rand(); nxt = find(cumsum(prob) >= r, 1, 'first'); end
                tour(step) = nxt; visited(nxt) = true;
            end
            if enableLocalUpdate
                for k = 1:(length(tour) - 1)
                    i = tour(k); j = tour(k + 1);
                    tau(i, j) = (1 - xi) * tau(i, j) + xi * tau0;
                    tau(j, i) = (1 - xi) * tau(j, i) + xi * tau0;
                end
            end
            fullOrder = [1, midIdx(tour), nPts]; c = 0;
            for k = 1:(nPts - 1), c = c + costMatrix(fullOrder(k), fullOrder(k + 1)); end
            antTours{a} = tour; antCosts(a) = c;
        end
        if enableSelectiveOpt && nOptAnts < nAnts
            [~, sortIdx] = sort(antCosts);
            for a_idx = 1:nOptAnts
                a = sortIdx(a_idx);
                [antTours{a}, antCosts(a)] = twoOpt(antTours{a}, midIdx, costMatrix, nPts);
            end
        end
        [iterBestCost, bestAntIdx] = min(antCosts); iterBestTour = antTours{bestAntIdx};
        improved = false;
        if iterBestCost < globalBestCost
            globalBestCost = iterBestCost; globalBestTour = iterBestTour; improved = true;
        end
        if trackHistory
            bestCostHistory(iter) = globalBestCost;
            avgCostHistory(iter) = mean(antCosts);
            timeHistory(iter) = toc(tStart);
        end
        if enableAdaptiveStop && iter >= minIter
            cv = std(antCosts) / mean(antCosts);
            if cv < cvThreshold, stopReason = sprintf('种群同质(CV=%.4f<%g)', cv, cvThreshold); break; end
            if improved, stagnationCount = 0;
            else
                stagnationCount = stagnationCount + 1;
                if stagnationCount >= stagnationLim
                    stopReason = sprintf('最优停滞(%d代未改善)', stagnationCount); break;
                end
            end
        end
        tau = tau * (1 - rho);
        deposit = Q / iterBestCost;
        for k = 1:(length(iterBestTour) - 1)
            i = iterBestTour(k); j = iterBestTour(k + 1);
            tau(i, j) = tau(i, j) + deposit; tau(j, i) = tau(j, i) + deposit;
        end
        eliteDeposit = Q / globalBestCost * 0.5;
        for k = 1:(length(globalBestTour) - 1)
            i = globalBestTour(k); j = globalBestTour(k + 1);
            tau(i, j) = tau(i, j) + eliteDeposit; tau(j, i) = tau(j, i) + eliteDeposit;
        end
        tau = min(tau, tauMax); tau = max(tau, tauMin);
        tauMax = 1 / (rho * globalBestCost); tauMin = tauMax / (2 * nMid); tauMin = max(tauMin, 1e-6);
    end
    bestCost = globalBestCost; bestOrder = [1, midIdx(globalBestTour), nPts]; actualIter = iter;
    if trackHistory
        history = struct();
        history.bestCostHistory = bestCostHistory(1:actualIter);
        history.avgCostHistory = avgCostHistory(1:actualIter);
        history.timeHistory = timeHistory(1:actualIter);
        history.iterCount = actualIter; history.elapsedTime = toc(tStart);
        if isempty(stopReason), stopReason = 'maxIter'; end
        history.stopReason = stopReason;
    end
end

function [tour, cost] = twoOpt(tour, midIdx, costMatrix, nPts)
    nMid = length(tour); fullOrder = [1, midIdx(tour), nPts];
    cost = 0;
    for k = 1:(nPts - 1), cost = cost + costMatrix(fullOrder(k), fullOrder(k + 1)); end
    improved = true;
    while improved
        improved = false;
        for i = 1:(nMid - 1)
            for j = (i + 1):nMid
                fi = i + 1; fj = j + 1;
                oldCost = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + ...
                          costMatrix(fullOrder(fj), fullOrder(fj + 1));
                newCost = costMatrix(fullOrder(fi), fullOrder(fj)) + ...
                          costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
                if newCost < oldCost - 1e-10
                    tour((i + 1):j) = tour(j:-1:(i + 1));
                    cost = cost - oldCost + newCost; improved = true;
                    fullOrder = [1, midIdx(tour), nPts];
                end
            end
        end
    end
end

function [distMatrix, coords] = parseKroA100(distFile, tspFile)
    nCities = 100; distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r');
    if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));
    for h = 1:4, fgetl(fid); end
    for i = 1:(nCities - 1)
        line = strtrim(fgetl(fid));
        colonPos = strfind(line, ':');
        if ~isempty(colonPos), line = strtrim(line(colonPos+1:end)); end
        vals = sscanf(line, '%f');
        for p = 1:length(vals), j = nCities - p + 1;
            distMatrix(i, j) = vals(p); distMatrix(j, i) = vals(p); end
    end
    coords = zeros(nCities, 2);
    fid2 = fopen(tspFile, 'r');
    if fid2 < 0, error('Cannot open: %s', tspFile); end
    cleanup2 = onCleanup(@() fclose(fid2));
    inCoordSection = false;
    while ~feof(fid2)
        line = strtrim(fgetl(fid2));
        if startsWith(line, 'NODE_COORD_SECTION'), inCoordSection = true; continue;
        elseif strcmp(line, 'EOF'), break; end
        if inCoordSection && ~isempty(line)
            parts = sscanf(line, '%f');
            if length(parts) >= 3 && parts(1) >= 1 && parts(1) <= nCities
                coords(parts(1), :) = [parts(2), parts(3)]; end
        end
    end
end
