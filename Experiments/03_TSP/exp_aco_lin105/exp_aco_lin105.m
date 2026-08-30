%% exp_aco_lin105 — TSP算法在lin105数据集上的性能测试
%  数据集: lin105 (105城市, EUC_2D坐标, 预计算距离矩阵)
%  起点/终点: 城市1 (同一点)
%  运行次数: 10次独立运行
%  导出: 城市顺序、收敛曲线、最优/最差成本等

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== TSP Algorithm Selector (change here to test different solvers) =====
% tspSolver = @(costMatrix, nPts) TSP_ACO(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v0(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v2_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v2_2(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_ACO_v2_4(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA_v1_1(costMatrix, nPts);

% ===== Experiment Config =====
nRuns = 10;
coordFile = fullfile(fileparts(mfilename('fullpath')), 'lin105_coords.mat');
distFile  = fullfile(fileparts(mfilename('fullpath')), 'lin105_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Load lin105 dataset =====
tmp = load(coordFile);  % → c105 (105×2)
cityCoords = tmp.c105;
fullDist105 = parseLin105Dist(distFile);
nCities = 105;

fprintf('======== TSP Experiment: lin105 ========\n');
fprintf('Algorithm: %s\n', func2str(tspSolver));
fprintf('Cities: %d | Runs: %d\n', nCities, nRuns);
fprintf('Start/Goal: City 1 (%.0f, %.0f)\n', cityCoords(1,1), cityCoords(1,2));
fprintf('Known optimal: 14379\n\n');

%% ===== Build 106x106 cost matrix (start=city1, goal=city1) =====
nPts = nCities + 1;  % 106: [1=start, 2..105=cities2..105, 106=goal]
costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist105(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist105(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist105(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist105(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist105(2:nCities, 2:nCities);

%% ===== Independent Runs =====
allOrders   = zeros(nRuns, nPts);
allCosts    = zeros(nRuns, 1);
allHistories = cell(1, nRuns);

fprintf('Running %d trials...\n', nRuns);
ticTotal = tic;

for run = 1:nRuns
    [bestOrder, bestCost, history] = tspSolver(costMatrix, nPts);
    allOrders(run, :) = bestOrder;
    allCosts(run)     = bestCost;
    allHistories{run} = history;
    if isfield(history, 'stopReason')
        stopInfo = history.stopReason;
    else
        stopInfo = 'N/A';
    end
    fprintf('  Run %2d: cost=%.2f | iter=%d | time=%.2fs | %s\n', ...
        run, bestCost, history.iterCount, history.elapsedTime, stopInfo);
end

totalWallTime = toc(ticTotal);

%% ===== Statistics =====
[stats.bestCost, bestRunIdx] = min(allCosts);
stats.worstCost  = max(allCosts);
stats.avgCost    = mean(allCosts);
stats.stdCost    = std(allCosts);
stats.medianCost = median(allCosts);

fprintf('\n===== Results =====\n');
fprintf('Best:   %.2f  (Run #%d)\n', stats.bestCost, bestRunIdx);
fprintf('Worst:  %.2f\n', stats.worstCost);
fprintf('Avg:    %.2f ± %.2f\n', stats.avgCost, stats.stdCost);
fprintf('Median: %.2f\n', stats.medianCost);
fprintf('Gap to optimal (14379): %.1f%%\n', (stats.bestCost - 14379) / 14379 * 100);
fprintf('Total wall time: %.1fs\n', totalWallTime);

%% ===== Export Data =====
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
resultsFile = fullfile(resultsDir, 'exp_aco_lin105_results.mat');
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'fullDist105', 'cityCoords', 'costMatrix');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Optimal Path Map =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

scatter(cityCoords(:,1), cityCoords(:,2), 30, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'none');

% Label every 10th city
labelStep = 10;
for i = 1:labelStep:nCities
    text(cityCoords(i,1) + 35, cityCoords(i,2) + 35, num2str(i), ...
        'FontSize', 6, 'Color', [0.2, 0.2, 0.2]);
end

% Best tour path
bestOrderAll = allOrders(bestRunIdx, :);
cityIdx = zeros(1, nPts);
for i = 1:nPts
    if bestOrderAll(i) == 1 || bestOrderAll(i) == nPts
        cityIdx(i) = 1;
    else
        cityIdx(i) = bestOrderAll(i);
    end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);

% Start marker
plot(cityCoords(1,1), cityCoords(1,2), 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1) + 80, cityCoords(1,2) - 80, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — lin105 (Cost: %.2f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 13);
axis equal tight;
grid on; set(gca, 'GridAlpha', 0.1);
hold off;

%% ===== Figure 2: Convergence Curves (Iteration) =====
figure('Position', [820, 50, 700, 500], 'Color', 'w');

maxIter = max(cellfun(@(h) h.iterCount, allHistories));
costMatrix_iter = nan(nRuns, maxIter);

for run = 1:nRuns
    h = allHistories{run};
    n = h.iterCount;
    costMatrix_iter(run, 1:n) = h.bestCostHistory(1:n);
    costMatrix_iter(run, n+1:end) = h.bestCostHistory(n);
end

medIter = median(costMatrix_iter, 1, 'omitnan');
loIter  = prctile(costMatrix_iter, 2.5, 1);
hiIter  = prctile(costMatrix_iter, 97.5, 1);
xIter = 1:maxIter;

fill([xIter, fliplr(xIter)], [loIter, fliplr(hiIter)], ...
    [0.2, 0.5, 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on;
plot(xIter, medIter, '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 2.0);
yline(14379, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (14379)', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 3: Convergence Curves (Time) =====
figure('Position', [820, 580, 700, 500], 'Color', 'w');

nTimePts = 500;
maxTime = max(cellfun(@(h) h.elapsedTime, allHistories));
tCommon = linspace(0, maxTime, nTimePts);
costMatrix_time = nan(nRuns, nTimePts);

for run = 1:nRuns
    h = allHistories{run};
    tRaw = h.timeHistory(1:h.iterCount);
    cRaw = h.bestCostHistory(1:h.iterCount);
    tRaw = [0; tRaw(:)];
    cRaw = [cRaw(1); cRaw(:)];
    [tUnique, ia] = unique(tRaw);
    cUnique = cRaw(ia);
    if length(tUnique) >= 2
        costMatrix_time(run, :) = interp1(tUnique, cUnique, tCommon, 'linear', cUnique(end));
    else
        costMatrix_time(run, :) = cUnique(end);
    end
end

medTime = median(costMatrix_time, 1, 'omitnan');
loTime  = prctile(costMatrix_time, 2.5, 1);
hiTime  = prctile(costMatrix_time, 97.5, 1);

fill([tCommon, fliplr(tCommon)], [loTime, fliplr(hiTime)], ...
    [0.8, 0.4, 0.2], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on;
plot(tCommon, medTime, '-', 'Color', [0.8, 0.4, 0.2], 'LineWidth', 2.0);
yline(14379, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost');
title('Convergence — Time (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (14379)', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 4: Cost Distribution =====
figure('Position', [50, 760, 750, 520], 'Color', 'w');

subplot(2,1,1);
histogram(allCosts, 10, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5);
hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('Cost Distribution (N=%d)', nRuns));
legend(sprintf('Best=%.1f', stats.bestCost), sprintf('Median=%.1f', stats.medianCost), 'Location', 'best');
grid on; hold off;

subplot(2,1,2);
boxplot(allCosts, 'Orientation', 'horizontal', 'Widths', 0.5);
hold on;
scatter(stats.bestCost, 1, 60, [0.9, 0.2, 0.2], 'filled', 'MarkerEdgeColor', 'k');
xlabel('Best Cost');
title('Cost Box Plot');
set(gca, 'YTickLabel', []);
grid on; hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ===== Local Functions =====

function distMatrix = parseLin105Dist(distFile)
%PARSELIN105DIST Parse lin105 distance matrix
%   distMatrix - 105x105 symmetric distance matrix
%   Format: upper triangular, cols in descending order, no header

    nCities = 105;

    % --- Parse distance matrix (upper triangular, cols in descending order) ---
    distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r');
    if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));

    for i = 1:(nCities - 1)
        line = strtrim(fgetl(fid));
        vals = sscanf(line, '%f');
        for p = 1:length(vals)
            j = nCities - p + 1;
            distMatrix(i, j) = vals(p);
            distMatrix(j, i) = vals(p);
        end
    end
end
