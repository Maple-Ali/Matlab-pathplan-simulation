%% exp_aco_v2_1_kroA150 — TSP_ACO_v2_1 参数敏感性分析（kroA150）
%  方法: 一次一个参数（OPAT），其他参数固定在默认值
%  参数: nAnts, alpha, beta, rho, Q, q0, dbRatio
%  每组合: 10次独立运行，自适应停止

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

%% ===== Config =====
nRunsPerCombo = 10;
tspFile  = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA150.tsp');
distFile = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA150_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse kroA150 dataset =====
[fullDist150, cityCoords] = parseKroA150(distFile, tspFile);
nCities = 150; nPts = nCities + 1;
costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist150(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist150(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist150(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist150(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist150(2:nCities, 2:nCities);

%% ===== Parameter Definitions =====
% Default values (matching TSP_ACO_v2_1)
defaults = struct('nAnts', 50, 'alpha', 1.0, 'beta', 2.0, ...
    'rho', 0.25, 'Q', 150, 'q0', 0.25, 'dbRatio', 0.25);

paramDefs = {
    'nAnts',    [20, 30, 40, 50, 60, 70];
    'alpha',    [0.5, 1.0, 1.5, 2.0, 3.0, 4.0];
    'beta',     [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    'rho',      [0.05, 0.15, 0.25, 0.40, 0.60];
    'Q',        [10, 50, 100, 150, 200, 500];
    'q0',       [0.0, 0.15, 0.25, 0.40, 0.60, 0.80];
    'dbRatio',  [0.0, 0.10, 0.20, 0.25, 0.35, 0.50];
};
nParams = size(paramDefs, 1);

%% ===== Run Ablation =====
fprintf('======== TSP_ACO_v2_1 Sensitivity: kroA150 ========\n');
fprintf('Method: One-Parameter-At-A-Time | Runs per combo: %d\n', nRunsPerCombo);
fprintf('Known optimal: 26524\n\n');

paramResults = cell(nParams, 1);
totalCombos = sum(cellfun(@length, paramDefs(:,2)));
comboCount = 0;

for pi = 1:nParams
    pName = paramDefs{pi, 1}; pVals = paramDefs{pi, 2}; nVals = length(pVals);
    statsArr = struct();
    statsArr(nVals).value = []; statsArr(nVals).allCosts = [];
    statsArr(nVals).allConvergeIters = []; statsArr(nVals).allTimes = [];
    statsArr(nVals).meanCost = []; statsArr(nVals).stdCost = [];
    statsArr(nVals).bestCost = []; statsArr(nVals).meanConvIter = [];
    statsArr(nVals).meanTime = []; statsArr(nVals).bestOrder = [];

    for vi = 1:nVals
        comboCount = comboCount + 1; pv = pVals(vi);
        overrides = defaults; overrides.(pName) = pv;
        costs = zeros(nRunsPerCombo, 1); convergeIters = zeros(nRunsPerCombo, 1);
        times = zeros(nRunsPerCombo, 1); bestCostLocal = inf; bestOrderLocal = [];

        fprintf('[%2d/%2d] %s = %-8g  running %d trials...', ...
            comboCount, totalCombos, pName, pv, nRunsPerCombo);

        for r = 1:nRunsPerCombo
            [order, cost, history] = TSP_ACO_v2_1_ablated(costMatrix, nPts, overrides);
            costs(r) = cost; convergeIters(r) = findConvergeIter(history.bestCostHistory, cost);
            times(r) = history.elapsedTime;
            if cost < bestCostLocal, bestCostLocal = cost; bestOrderLocal = order; end
        end

        statsArr(vi).value = pv; statsArr(vi).allCosts = costs;
        statsArr(vi).allConvergeIters = convergeIters; statsArr(vi).allTimes = times;
        statsArr(vi).meanCost = mean(costs); statsArr(vi).stdCost = std(costs);
        statsArr(vi).bestCost = min(costs); statsArr(vi).meanConvIter = mean(convergeIters);
        statsArr(vi).meanTime = mean(times); statsArr(vi).bestOrder = bestOrderLocal;
        fprintf('  best=%.1f | avg=%.1f±%.1f | convIter=%.0f | time=%.2fs\n', ...
            min(costs), mean(costs), std(costs), mean(convergeIters), mean(times));
    end
    paramResults{pi} = struct('paramName', pName, 'values', pVals, 'stats', statsArr);
end

%% ===== Identify Best Parameters =====
fprintf('\n===== Best Parameters (by lowest mean cost) =====\n');
bestParams = struct();
for pi = 1:nParams
    pr = paramResults{pi}; [~, bestIdx] = min([pr.stats.meanCost]);
    bestParams.(pr.paramName) = pr.values(bestIdx);
    fprintf('  %-10s = %-8g  (mean cost: %.1f, gap: %.1f%%)\n', pr.paramName, ...
        bestParams.(pr.paramName), pr.stats(bestIdx).meanCost, ...
        (pr.stats(bestIdx).meanCost - 26524) / 26524 * 100);
end

save(fullfile(resultsDir, 'exp_aco_v2_1_kroA150_results.mat'), ...
    'paramResults', 'paramDefs', 'bestParams', 'defaults', ...
    'fullDist150', 'cityCoords', 'costMatrix', 'nRunsPerCombo');

%% ===== Figure 1: Cost Summary (3x3 grid) =====
figure('Position', [50, 50, 1500, 1000], 'Color', 'w'); colors = lines(nParams);
for pi = 1:nParams
    subplot(3, 3, pi); pr = paramResults{pi}; vals = pr.values(:)';
    means = [pr.stats.meanCost]; stds = [pr.stats.stdCost];
    errorbar(vals, means, stds, 'o-', 'Color', colors(pi,:), 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    hold on; [~, bi] = min(means);
    plot(vals(bi), means(bi), 'p', 'MarkerSize', 15, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    xlabel(pr.paramName); ylabel('Best Cost');
    title(sprintf('%s (best: %.4g @ %.4g)', pr.paramName, means(bi), vals(bi)));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_ACO\_v2\_1 Sensitivity (kroA150) — Cost', 'FontSize', 14);

%% ===== Figure 2: Convergence Speed =====
figure('Position', [50, 50, 1500, 1000], 'Color', 'w');
for pi = 1:nParams
    subplot(3, 3, pi); pr = paramResults{pi}; vals = pr.values(:)';
    cv = [pr.stats.meanConvIter]; cs = zeros(1, length(pr.stats));
    for vi = 1:length(pr.stats), cs(vi) = std(pr.stats(vi).allConvergeIters); end
    errorbar(vals, cv, cs, 's-', 'Color', colors(pi,:), 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    hold on; [~, fi] = min(cv);
    plot(vals(fi), cv(fi), 'p', 'MarkerSize', 15, 'MarkerFaceColor', [0.2, 0.7, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    xlabel(pr.paramName); ylabel('Converge Iteration');
    title(sprintf('%s (fastest: %.0f @ %.4g)', pr.paramName, cv(fi), vals(fi)));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_ACO\_v2\_1 Sensitivity (kroA150) — Conv Speed', 'FontSize', 14);

%% ===== Figure 3: Runtime =====
figure('Position', [50, 50, 1500, 1000], 'Color', 'w');
for pi = 1:nParams
    subplot(3, 3, pi); pr = paramResults{pi}; vals = pr.values(:)';
    tv = [pr.stats.meanTime]; ts = zeros(1, length(pr.stats));
    for vi = 1:length(pr.stats), ts(vi) = std(pr.stats(vi).allTimes); end
    errorbar(vals, tv, ts, 'd-', 'Color', colors(pi,:), 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', colors(pi,:));
    xlabel(pr.paramName); ylabel('Runtime (s)'); title(sprintf('%s — Runtime', pr.paramName));
    grid on; set(gca, 'GridAlpha', 0.15);
end
sgtitle('TSP\_ACO\_v2\_1 Sensitivity (kroA150) — Runtime', 'FontSize', 14);

%% ===== Figure 4: Best-Param Tour =====
fprintf('\nRunning TSP_ACO_v2_1 with best parameters for tour visualization...\n');
[bestOrder, bestCost, ~] = TSP_ACO_v2_1_ablated(costMatrix, nPts, bestParams);
fprintf('  Best-param cost: %.2f (gap: %.1f%%)\n', bestCost, (bestCost-26524)/26524*100);
figure('Position', [150, 150, 750, 680], 'Color', 'w'); hold on;
scatter(cityCoords(:,1), cityCoords(:,2), 25, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
for i = 1:15:nCities
    text(cityCoords(i,1)+30, cityCoords(i,2)+30, num2str(i), ...
        'FontSize', 6, 'Color', [0.2, 0.2, 0.2]);
end
cityIdx = zeros(1, nPts);
for i = 1:nPts
    if bestOrder(i) == 1 || bestOrder(i) == nPts, cityIdx(i) = 1;
    else, cityIdx(i) = bestOrder(i); end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);
plot(cityCoords(1,1), cityCoords(1,2), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1)+60, cityCoords(1,2)-80, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
bpText = sprintf('Best: nAnts=%.4g, alpha=%.4g, beta=%.4g, rho=%.4g, Q=%.4g, q0=%.4g, dbRatio=%.4g', ...
    bestParams.nAnts, bestParams.alpha, bestParams.beta, bestParams.rho, bestParams.Q, bestParams.q0, bestParams.dbRatio);
xlabel('X'); ylabel('Y');
title(sprintf('Best-Param TSP Tour (Cost: %.2f, gap: %.1f%%)\n%s', bestCost, (bestCost-26524)/26524*100, bpText), 'FontSize', 9);
axis equal tight; grid on; set(gca, 'GridAlpha', 0.1); hold off;
fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ===== Local Functions =====

function convIter = findConvergeIter(bestCostHistory, finalCost)
    threshold = finalCost * 1.01;
    idx = find(bestCostHistory <= threshold, 1, 'first');
    if isempty(idx), convIter = length(bestCostHistory); else, convIter = idx; end
end

function [bestOrder, bestCost, history] = TSP_ACO_v2_1_ablated(costMatrix, nPts, params)
    nAnts = 50; alpha = 1.0; beta = 2.0; rho = 0.25; Q = 150; q0 = 0.25; dbRatio = 0.25;
    if nargin >= 3 && ~isempty(params)
        if isfield(params, 'nAnts'),    nAnts    = params.nAnts;    end
        if isfield(params, 'alpha'),    alpha    = params.alpha;    end
        if isfield(params, 'beta'),     beta     = params.beta;     end
        if isfield(params, 'rho'),      rho      = params.rho;      end
        if isfield(params, 'Q'),        Q        = params.Q;        end
        if isfield(params, 'q0'),       q0       = params.q0;       end
        if isfield(params, 'dbRatio'),  dbRatio  = params.dbRatio;  end
    end
    nMid = nPts - 2; midIdx = 2:(nPts - 1); nIter = 800;
    enablePseudoRandom = 1; optRatio_start = 0.00; optRatio_end = 0.45;
    optTransInterval = [0, 120]; optCurveA = 2.0; enableDoubleBridge = 1;
    enableAdaptiveStop = 1; cvThreshold = 0.001; stagnationLim = 50; minIter = 30;

    trackHistory = (nargout >= 3);
    if trackHistory, tStart = tic;
        bestCostHistory = zeros(nIter, 1); avgCostHistory = zeros(nIter, 1);
        timeHistory = zeros(nIter, 1);
    end
    if nMid == 0
        bestOrder = [1, nPts]; bestCost = costMatrix(1, nPts);
        if trackHistory, history = struct('bestCostHistory', bestCost, 'avgCostHistory', bestCost, ...
                'timeHistory', 0, 'iterCount', 1, 'elapsedTime', 0, 'stopReason', 'trivial'); end
        return;
    end
    eta = zeros(nMid, nMid);
    for i = 1:nMid, for j = 1:nMid, if i ~= j, d = costMatrix(midIdx(i), midIdx(j));
                if d > 0 && ~isinf(d), eta(i, j) = 1.0 / d; end; end; end; end
    tau0 = 0.1; tau = ones(nMid, nMid) * tau0;
    tauMax = 1 / (rho * tau0 * nMid); tauMin = tauMax / (5 * nMid); tauMin = max(tauMin, 1e-6);
    tau = min(tau, tauMax); tau = max(tau, tauMin);
    globalBestCost = inf; globalBestTour = midIdx; stagnationCount = 0; stopReason = '';

    for iter = 1:nIter
        xL = optTransInterval(1); xR = optTransInterval(2);
        if iter <= xL, optRatio = optRatio_start;
        elseif iter >= xR, optRatio = optRatio_end;
        else
            t = (iter - xL) / (xR - xL);
            optRatio = optRatio_start + (optRatio_end - optRatio_start) * (t^optCurveA / (t^optCurveA + (1-t)^optCurveA));
        end
        nOptAnts = max(1, round(nAnts * optRatio));
        antTours = cell(nAnts, 1); antCosts = zeros(nAnts, 1);
        for a = 1:nAnts
            tour = zeros(1, nMid); visited = false(1, nMid); start = randi(nMid);
            tour(1) = start; visited(start) = true;
            for step = 2:nMid
                curIdx = tour(step - 1); candidates = find(~visited); nCand = length(candidates);
                scores = zeros(1, nCand);
                for c = 1:nCand, j = candidates(c); scores(c) = (tau(curIdx, j) ^ alpha) * (eta(curIdx, j) ^ beta); end
                if enablePseudoRandom && rand() < q0
                    [~, bestScIdx] = max(scores); nxt = candidates(bestScIdx);
                else
                    totalScore = sum(scores);
                    if totalScore == 0, nxt = candidates(randi(nCand));
                    else, prob = scores / totalScore; nxt = candidates(find(cumsum(prob) >= rand(), 1, 'first')); end
                end
                tour(step) = nxt; visited(nxt) = true;
            end
            antTours{a} = tour; antCosts(a) = tourCost(tour, midIdx, costMatrix, nPts);
        end
        [~, sortIdx] = sort(antCosts);
        for a_idx = 1:nOptAnts, a = sortIdx(a_idx);
            [antTours{a}, antCosts(a)] = vndSearch(antTours{a}, midIdx, costMatrix, nPts); end
        if enableDoubleBridge && nMid >= 8
            nDB = max(1, round(nAnts * dbRatio));
            for a_idx = (nAnts - nDB + 1) : nAnts, a = sortIdx(a_idx);
                antTours{a} = doubleBridge(antTours{a}); antCosts(a) = tourCost(antTours{a}, midIdx, costMatrix, nPts); end
        end
        [iterBestCost, bestAntIdx] = min(antCosts); iterBestTour = antTours{bestAntIdx};
        improved = false;
        if iterBestCost < globalBestCost, globalBestCost = iterBestCost; globalBestTour = iterBestTour; improved = true; end
        if trackHistory, bestCostHistory(iter) = globalBestCost; avgCostHistory(iter) = mean(antCosts); timeHistory(iter) = toc(tStart); end
        if enableAdaptiveStop && iter >= minIter
            cv = std(antCosts) / mean(antCosts);
            if cv < cvThreshold, stopReason = sprintf('种群同质(CV=%.4f<%g)', cv, cvThreshold); break; end
            if improved, stagnationCount = 0; else, stagnationCount = stagnationCount + 1;
                if stagnationCount >= stagnationLim, stopReason = sprintf('最优停滞(%d代)', stagnationCount); break; end
            end
        end
        tau = tau * (1 - rho);
        deposit = Q / iterBestCost;
        for k = 1:(length(iterBestTour)-1), i = iterBestTour(k); j = iterBestTour(k+1);
            tau(i, j) = tau(i, j) + deposit; tau(j, i) = tau(j, i) + deposit; end
        eliteDeposit = Q / globalBestCost;
        for k = 1:(length(globalBestTour)-1), i = globalBestTour(k); j = globalBestTour(k+1);
            tau(i, j) = tau(i, j) + eliteDeposit; tau(j, i) = tau(j, i) + eliteDeposit; end
        tau = min(tau, tauMax); tau = max(tau, tauMin);
        tauMax = 1 / (rho * globalBestCost); tauMin = tauMax / nMid; tauMin = max(tauMin, 1e-6);
    end
    bestCost = globalBestCost; bestOrder = [1, midIdx(globalBestTour), nPts];
    if trackHistory
        history = struct(); history.bestCostHistory = bestCostHistory(1:iter);
        history.avgCostHistory = avgCostHistory(1:iter); history.timeHistory = timeHistory(1:iter);
        history.iterCount = iter; history.elapsedTime = toc(tStart);
        if isempty(stopReason), stopReason = 'maxIter'; end
        history.stopReason = stopReason;
    end
end

% ---- VND + sub-operators ----
function [tour, cost] = vndSearch(tour, midIdx, costMatrix, nPts)
    cost = tourCost(tour, midIdx, costMatrix, nPts); improved = true;
    while improved
        improved = false;
        [tour, cost, ok] = twoOptFI(tour, midIdx, costMatrix, nPts, cost); improved = improved || ok;
        [tour, cost, ok] = relocateFI(tour, midIdx, costMatrix, nPts, cost); improved = improved || ok;
        [tour, cost, ok] = swapFI(tour, midIdx, costMatrix, nPts, cost); improved = improved || ok;
    end
    cost = tourCost(tour, midIdx, costMatrix, nPts);
end

function c = tourCost(tour, midIdx, costMatrix, nPts)
    fo = [1, midIdx(tour), nPts]; c = 0;
    for k = 1:(nPts - 1), c = c + costMatrix(fo(k), fo(k + 1)); end
end

function [tour, cost, improved] = twoOptFI(tour, midIdx, costMatrix, nPts, cost)
    nMid = length(tour); fullOrder = [1, midIdx(tour), nPts]; improved = false;
    for i = 1:(nMid - 1), for j = (i + 1):nMid
            fi = i + 1; fj = j + 1;
            old = costMatrix(fullOrder(fi), fullOrder(fi + 1)) + costMatrix(fullOrder(fj), fullOrder(fj + 1));
            nw = costMatrix(fullOrder(fi), fullOrder(fj)) + costMatrix(fullOrder(fi + 1), fullOrder(fj + 1));
            if nw < old - 1e-10, tour((i + 1):j) = tour(j:-1:(i + 1));
                cost = cost - old + nw; improved = true; fullOrder = [1, midIdx(tour), nPts]; end
    end; end
end

function [tour, cost, improved] = relocateFI(tour, midIdx, costMatrix, nPts, cost)
    nMid = length(tour); improved = false;
    for v = 1:nMid
        Cv = midIdx(tour(v)); if v == 1, Lv = 1; else, Lv = midIdx(tour(v - 1)); end
        if v == nMid, Rv = nPts; else, Rv = midIdx(tour(v + 1)); end
        for p = 0:nMid
            if p == v || p == v - 1, continue; end
            if p == 0, Lp = 1; Rp = midIdx(tour(1));
            elseif p == nMid, Lp = midIdx(tour(nMid)); Rp = nPts;
            else, Lp = midIdx(tour(p)); Rp = midIdx(tour(p + 1)); end
            old = costMatrix(Lv, Cv) + costMatrix(Cv, Rv) + costMatrix(Lp, Rp);
            nw  = costMatrix(Lv, Rv) + costMatrix(Lp, Cv) + costMatrix(Cv, Rp);
            if nw < old - 1e-10
                cityVal = tour(v);
                if p < v, tour = [tour(1:p), cityVal, tour(p+1:v-1), tour(v+1:end)];
                else, tour = [tour(1:v-1), tour(v+1:p), cityVal, tour(p+1:end)]; end
                cost = cost - old + nw; improved = true; return;
            end
        end
    end
end

function [tour, cost, improved] = swapFI(tour, midIdx, costMatrix, nPts, cost)
    nMid = length(tour); fullOrder = [1, midIdx(tour), nPts]; improved = false;
    for i = 1:(nMid - 1), for j = (i + 1):nMid
            fi = i + 1; fj = j + 1;
            Li = fullOrder(fi - 1); Ai = fullOrder(fi); Ri = fullOrder(fi + 1);
            Lj = fullOrder(fj - 1); Aj = fullOrder(fj); Rj = fullOrder(fj + 1);
            if j == i + 1, old = costMatrix(Li, Ai) + costMatrix(Ai, Aj) + costMatrix(Aj, Rj);
                nw = costMatrix(Li, Aj) + costMatrix(Aj, Ai) + costMatrix(Ai, Rj);
            else, old = costMatrix(Li, Ai) + costMatrix(Ai, Ri) + costMatrix(Lj, Aj) + costMatrix(Aj, Rj);
                nw = costMatrix(Li, Aj) + costMatrix(Aj, Ri) + costMatrix(Lj, Ai) + costMatrix(Ai, Rj); end
            if nw < old - 1e-10, tour([i, j]) = tour([j, i]);
                cost = cost - old + nw; improved = true; fullOrder = [1, midIdx(tour), nPts]; end
    end; end
end

function tour = doubleBridge(tour)
    nMid = length(tour); if nMid < 8, return; end
    minSeg = 2; i = randi([minSeg, nMid - 3*minSeg]);
    j = randi([i + minSeg, nMid - 2*minSeg]); k = randi([j + minSeg, nMid - minSeg]);
    tour = [tour(1:i), tour(k:-1:j+1), tour(j:-1:i+1), tour(k+1:end)];
end

function [distMatrix, coords] = parseKroA150(distFile, tspFile)
    nCities = 150; distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r'); if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));
    while ~feof(fid), line = strtrim(fgetl(fid));
        if ~isempty(line) && ~isnan(str2double(strtok(line, ':'))), break; end
    end
    for i = 1:(nCities - 1)
        colonPos = strfind(line, ':'); if ~isempty(colonPos), line = strtrim(line(colonPos+1:end)); end
        vals = sscanf(line, '%f');
        for p = 1:length(vals), j = nCities - p + 1; distMatrix(i, j) = vals(p); distMatrix(j, i) = vals(p); end
        if i < nCities - 1, line = strtrim(fgetl(fid)); end
    end
    coords = zeros(nCities, 2);
    fid2 = fopen(tspFile, 'r'); if fid2 < 0, error('Cannot open: %s', tspFile); end
    cleanup2 = onCleanup(@() fclose(fid2)); inCoord = false;
    while ~feof(fid2)
        line = strtrim(fgetl(fid2));
        if startsWith(line, 'NODE_COORD_SECTION'), inCoord = true; continue;
        elseif strcmp(line, 'EOF'), break; end
        if inCoord && ~isempty(line), parts = sscanf(line, '%f');
            if length(parts) >= 3 && parts(1) >= 1 && parts(1) <= nCities
                coords(parts(1), :) = [parts(2), parts(3)]; end
        end
    end
end
