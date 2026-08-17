%% exp_aco_kroA100 — TSP算法在kroA100数据集上的性能测试
%  数据集: kroA100 (100城市, EUC_2D坐标, 预计算距离矩阵)
%  起点/终点: 城市1 (同一点)
%  运行次数: 30次独立运行
%  导出: 城市顺序、收敛曲线、最优/最差成本等

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== TSP Algorithm Selector (change here to test different solvers) =====
% tspSolver = @(costMatrix, nPts) TSP_ACO(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v0(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v1_7(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA_v1_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v1_7_1(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_ACO_v2_4(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v2_2(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v2_1(costMatrix, nPts);

% ===== Experiment Config =====
nRuns = 10;
coordFile = fullfile(fileparts(mfilename('fullpath')), 'kroA100_coords.mat');
distFile  = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA100_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Load kroA100 dataset =====
tmp = load(coordFile);  % → c100 (100×2)
cityCoords = tmp.c100;
fullDist100 = parseKroA100Dist(distFile);
nCities = 100;

fprintf('======== TSP Experiment: kroA100 ========\n');
fprintf('Algorithm: %s\n', func2str(tspSolver));
fprintf('Cities: %d | Runs: %d\n', nCities, nRuns);
fprintf('Start/Goal: City 1 (%.0f, %.0f)\n', cityCoords(1,1), cityCoords(1,2));
fprintf('Known optimal: 21282\n\n');

%% ===== Build 101x101 cost matrix (start=city1, goal=city1) =====
nPts = nCities + 1;  % 101: [1=start, 2..100=cities2..100, 101=goal]
costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist100(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist100(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist100(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist100(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist100(2:nCities, 2:nCities);

%% ===== 30 Independent Runs =====
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
fprintf('Gap to optimal (21282): %.1f%%\n', (stats.bestCost - 21282) / 21282 * 100);
fprintf('Total wall time: %.1fs\n', totalWallTime);

%% ===== Export Data =====
resultsFile = fullfile(resultsDir, 'exp_aco_kroA100_results.mat');
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'fullDist100', 'cityCoords', 'costMatrix');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Optimal Path Map =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

scatter(cityCoords(:,1), cityCoords(:,2), 30, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'none');

for i = 1:nCities
    text(cityCoords(i,1) + 20, cityCoords(i,2) + 20, num2str(i), ...
        'FontSize', 5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end

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

plot(cityCoords(1,1), cityCoords(1,2), 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1) + 60, cityCoords(1,2) - 80, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — kroA100 (Cost: %.2f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 13);
axis equal tight;
grid on; set(gca, 'GridAlpha', 0.1);
hold off;

%% ===== Figure 2: 收敛（迭代） =====
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
yline(21282, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (21282)', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 3: Convergence (Time) =====
figure('Position', [820, 400, 700, 500], 'Color', 'w');

nTimePts = 500;
maxTime = max(cellfun(@(h) h.elapsedTime, allHistories));
tCommon = linspace(0, maxTime, nTimePts);
costMatrix_time = nan(nRuns, nTimePts);

for run = 1:nRuns
    h = allHistories{run};
    tRaw = h.timeHistory(1:h.iterCount);
    cRaw = h.bestCostHistory(1:h.iterCount);
    tRaw = [0; tRaw(:)]; cRaw = [cRaw(1); cRaw(:)];
    [tUnique, ia] = unique(tRaw); cUnique = cRaw(ia);
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
yline(21282, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost');
title('Convergence — Time (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (21282)', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 4: Cost Distribution =====
figure('Position', [50, 200, 750, 520], 'Color', 'w');

subplot(2,1,1);
histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5);
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

function distMatrix = parseKroA100Dist(distFile)
    nCities = 100;
    distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r');
    if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));
    for h = 1:4, fgetl(fid); end
    for i = 1:(nCities - 1)
        line = strtrim(fgetl(fid));
        colonPos = strfind(line, ':');
        if ~isempty(colonPos), line = strtrim(line(colonPos+1:end)); end
        vals = sscanf(line, '%f');
        for p = 1:length(vals)
            j = nCities - p + 1;
            distMatrix(i, j) = vals(p); distMatrix(j, i) = vals(p);
        end
    end
end
