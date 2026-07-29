%% exp_aco_kroA150 — TSP算法在kroA150数据集上的性能测试
%  数据集: kroA150 (150城市, EUC_2D坐标, 预计算距离矩阵)
%  起点/终点: 城市1 (同一点)
%  运行次数: 30次独立运行
%  导出: 城市顺序、收敛曲线、最优/最差成本等

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== TSP Algorithm Selector (change here to test different solvers) =====
% tspSolver = @(costMatrix, nPts) TSP_ACO_v2(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_ACO_v1_7(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v1_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA_v2(costMatrix, nPts);

% ===== Experiment Config =====
nRuns = 10;
tspFile  = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA150.tsp');
distFile = fullfile(fileparts(mfilename('fullpath')), '..', 'kroA150_distance_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse kroA150 dataset =====
[fullDist150, cityCoords] = parseKroA150(distFile, tspFile);
nCities = 150;

fprintf('======== TSP Experiment: kroA150 ========\n');
fprintf('Algorithm: %s\n', func2str(tspSolver));
fprintf('Cities: %d | Runs: %d\n', nCities, nRuns);
fprintf('Start/Goal: City 1 (%.0f, %.0f)\n', cityCoords(1,1), cityCoords(1,2));
fprintf('Known optimal: 26524\n\n');

%% ===== Build 151x151 cost matrix (start=city1, goal=city1) =====
nPts = nCities + 1;  % 151: [1=start, 2..150=cities2..150, 151=goal]
costMatrix = zeros(nPts);
costMatrix(1, 2:nCities)         = fullDist150(1, 2:nCities);
costMatrix(2:nCities, 1)         = fullDist150(2:nCities, 1);
costMatrix(2:nCities, nPts)      = fullDist150(2:nCities, 1);
costMatrix(nPts, 2:nCities)      = fullDist150(1, 2:nCities);
costMatrix(2:nCities, 2:nCities) = fullDist150(2:nCities, 2:nCities);

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
fprintf('Gap to optimal (26524): %.1f%%\n', (stats.bestCost - 26524) / 26524 * 100);
fprintf('Total wall time: %.1fs\n', totalWallTime);

%% ===== Export Data =====
resultsFile = fullfile(resultsDir, 'exp_aco_kroA150_results.mat');
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'fullDist150', 'cityCoords', 'costMatrix');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Optimal Path Map =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

scatter(cityCoords(:,1), cityCoords(:,2), 25, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'none');

% Label every 15th city
labelStep = 15;
for i = 1:labelStep:nCities
    text(cityCoords(i,1) + 30, cityCoords(i,2) + 30, num2str(i), ...
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
text(cityCoords(1,1) + 60, cityCoords(1,2) - 80, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — kroA150 (Cost: %.2f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 13);
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
yline(26524, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (26524)', 'Location', 'northeast'); grid on;
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
yline(26524, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost');
title('Convergence — Time (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (26524)', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 4: Cost Distribution =====
figure('Position', [50, 760, 750, 520], 'Color', 'w');

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

function [distMatrix, coords] = parseKroA150(distFile, tspFile)
%PARSEKROA150 Parse kroA150 distance matrix and coordinates
%   distMatrix - 150x150 symmetric distance matrix
%   coords     - 150x2 [x, y] coordinates

    nCities = 150;

    % --- Parse distance matrix (upper triangular, cols in descending order) ---
    distMatrix = zeros(nCities);
    fid = fopen(distFile, 'r');
    if fid < 0, error('Cannot open: %s', distFile); end
    cleanup1 = onCleanup(@() fclose(fid));

    for h = 1:4, fgetl(fid); end  % skip header

    for i = 1:(nCities - 1)
        line = strtrim(fgetl(fid));
        colonPos = strfind(line, ':');
        if ~isempty(colonPos)
            line = strtrim(line(colonPos+1:end));
        end
        vals = sscanf(line, '%f');
        nVals = length(vals);
        for p = 1:nVals
            j = nCities - p + 1;
            distMatrix(i, j) = vals(p);
            distMatrix(j, i) = vals(p);
        end
    end

    % --- Parse coordinates from kroA150.tsp ---
    coords = zeros(nCities, 2);
    fid2 = fopen(tspFile, 'r');
    if fid2 < 0, error('Cannot open: %s', tspFile); end
    cleanup2 = onCleanup(@() fclose(fid2));

    inCoordSection = false;
    while ~feof(fid2)
        line = strtrim(fgetl(fid2));
        if startsWith(line, 'NODE_COORD_SECTION')
            inCoordSection = true; continue;
        elseif strcmp(line, 'EOF'), break; end
        if inCoordSection && ~isempty(line)
            parts = sscanf(line, '%f');
            if length(parts) >= 3
                cityIdx = parts(1);
                if cityIdx >= 1 && cityIdx <= nCities
                    coords(cityIdx, :) = [parts(2), parts(3)];
                end
            end
        end
    end
end
