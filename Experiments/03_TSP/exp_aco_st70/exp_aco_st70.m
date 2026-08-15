%% exp_aco_st70 — TSP算法在st70数据集上的性能测试
%  数据集: st70 (70城市, EUC_2D坐标, 预计算距离矩阵)
%  起点/终点: 城市1 (同一点)
%  运行次数: 30次独立运行

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% tspSolver = @(costMatrix, nPts) TSP_ACO_v2(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_ACO_v2_3(costMatrix, nPts);

nRuns = 15;
tspFile  = fullfile(fileparts(mfilename('fullpath')), '..', 'st70.tsp');
distFile = fullfile(fileparts(mfilename('fullpath')), '..', 'st70_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse st70 =====
[fullDist70, cityCoords] = parseSt70(distFile, tspFile);
nCities = 70;
fprintf('======== TSP Experiment: st70 ========\n');
fprintf('Algorithm: %s | Cities: %d | Runs: %d\n', func2str(tspSolver), nCities, nRuns);
fprintf('Start/Goal: City 1 (%.0f, %.0f) | Known optimal: 675\n\n', cityCoords(1,:));

%% ===== Build cost matrix =====
nPts = nCities + 1; costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist70(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist70(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist70(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist70(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist70(2:nCities, 2:nCities);

%% ===== 30 Runs =====
allOrders = zeros(nRuns, nPts); allCosts = zeros(nRuns, 1); allHistories = cell(1, nRuns);
fprintf('Running %d trials...\n', nRuns); ticTotal = tic;
for run = 1:nRuns
    [bestOrder, bestCost, history] = tspSolver(costMatrix, nPts);
    allOrders(run, :) = bestOrder; allCosts(run) = bestCost; allHistories{run} = history;
    stopInfo = 'N/A'; if isfield(history, 'stopReason'), stopInfo = history.stopReason; end
    fprintf('  Run %2d: cost=%.2f | iter=%d | time=%.2fs | %s\n', ...
        run, bestCost, history.iterCount, history.elapsedTime, stopInfo);
end
totalWallTime = toc(ticTotal);

[stats.bestCost, bestRunIdx] = min(allCosts); stats.worstCost = max(allCosts);
stats.avgCost = mean(allCosts); stats.stdCost = std(allCosts); stats.medianCost = median(allCosts);
fprintf('\n===== Results =====\nBest: %.2f (Run #%d) | Worst: %.2f\nAvg: %.2f ± %.2f | Median: %.2f\n', ...
    stats.bestCost, bestRunIdx, stats.worstCost, stats.avgCost, stats.stdCost, stats.medianCost);
fprintf('Gap to optimal (675): %.1f%% | Wall time: %.1fs\n', (stats.bestCost-675)/675*100, totalWallTime);

save(fullfile(resultsDir, 'exp_aco_st70_results.mat'), ...
    'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', 'fullDist70', 'cityCoords', 'costMatrix');

%% ===== Figure 1: Optimal Path Map =====
figure('Position', [50, 50, 750, 680], 'Color', 'w'); hold on;
scatter(cityCoords(:,1), cityCoords(:,2), 40, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
for i = 1:nCities
    text(cityCoords(i,1) + 2, cityCoords(i,2) + 2, num2str(i), ...
        'FontSize', 5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end
bestOrderAll = allOrders(bestRunIdx, :); cityIdx = zeros(1, nPts);
for i = 1:nPts
    if bestOrderAll(i) == 1 || bestOrderAll(i) == nPts, cityIdx(i) = 1;
    else, cityIdx(i) = bestOrderAll(i); end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.0);
plot(cityCoords(1,1), cityCoords(1,2), 'o', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1) + 5, cityCoords(1,2) - 5, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — st70 (Cost: %.2f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 13);
axis equal tight; grid on; set(gca, 'GridAlpha', 0.1); hold off;

%% ===== Figures 2-4 =====
figure('Position', [820, 50, 700, 500], 'Color', 'w');
maxIter = max(cellfun(@(h) h.iterCount, allHistories)); costMatrix_iter = nan(nRuns, maxIter);
for run = 1:nRuns, h = allHistories{run}; n = h.iterCount;
    costMatrix_iter(run, 1:n) = h.bestCostHistory(1:n); costMatrix_iter(run, n+1:end) = h.bestCostHistory(n);
end
medIter = median(costMatrix_iter, 1, 'omitnan');
loIter = prctile(costMatrix_iter, 2.5, 1); hiIter = prctile(costMatrix_iter, 97.5, 1);
fill([1:maxIter, maxIter:-1:1], [loIter, fliplr(hiIter)], [0.2, 0.5, 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none'); hold on;
plot(1:maxIter, medIter, '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 2.0);
yline(675, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost'); title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (675)', 'Location', 'northeast'); grid on; hold off;

figure('Position', [820, 580, 700, 500], 'Color', 'w');
nTimePts = 500; maxTime = max(cellfun(@(h) h.elapsedTime, allHistories));
tCommon = linspace(0, maxTime, nTimePts); costMatrix_time = nan(nRuns, nTimePts);
for run = 1:nRuns, h = allHistories{run};
    tRaw = h.timeHistory(1:h.iterCount); cRaw = h.bestCostHistory(1:h.iterCount);
    tRaw = [0; tRaw(:)]; cRaw = [cRaw(1); cRaw(:)]; [tUnique, ia] = unique(tRaw); cUnique = cRaw(ia);
    if length(tUnique) >= 2, costMatrix_time(run, :) = interp1(tUnique, cUnique, tCommon, 'linear', cUnique(end));
    else, costMatrix_time(run, :) = cUnique(end); end
end
medTime = median(costMatrix_time, 1, 'omitnan'); loTime = prctile(costMatrix_time, 2.5, 1); hiTime = prctile(costMatrix_time, 97.5, 1);
fill([tCommon, fliplr(tCommon)], [loTime, fliplr(hiTime)], [0.8, 0.4, 0.2], 'FaceAlpha', 0.15, 'EdgeColor', 'none'); hold on;
plot(tCommon, medTime, '-', 'Color', [0.8, 0.4, 0.2], 'LineWidth', 2.0); yline(675, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost'); title('Convergence — Time (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (675)', 'Location', 'northeast'); grid on; hold off;

figure('Position', [50, 760, 750, 520], 'Color', 'w');
subplot(2,1,1); histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5);
hold on; xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency'); title(sprintf('Cost Distribution (N=%d)', nRuns));
legend(sprintf('Best=%.1f', stats.bestCost), sprintf('Median=%.1f', stats.medianCost), 'Location', 'best'); grid on; hold off;
subplot(2,1,2); boxplot(allCosts, 'Orientation', 'horizontal', 'Widths', 0.5);
hold on; scatter(stats.bestCost, 1, 60, [0.9, 0.2, 0.2], 'filled', 'MarkerEdgeColor', 'k');
xlabel('Best Cost'); title('Cost Box Plot'); set(gca, 'YTickLabel', []); grid on; hold off;
fprintf('\nAll figures ready.\n');

function [distMatrix, coords] = parseSt70(distFile, tspFile)
    nCities = 70; distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r'); if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));
    while ~feof(fid)
        line = strtrim(fgetl(fid));
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
        if inCoord && ~isempty(line)
            parts = sscanf(line, '%f');
            if length(parts) >= 3 && parts(1) >= 1 && parts(1) <= nCities
                coords(parts(1), :) = [parts(2), parts(3)]; end
        end
    end
end
