%% exp_aco_bayg29 — TSP算法在bayg29数据集上的性能测试
%  数据集: bayg29 (29城市, 显式距离矩阵, UPPER_ROW格式)
%  起点/终点: 城市1 (同一点)
%  运行次数: 30次独立运行
%  导出: 城市顺序、收敛曲线、最优/最差成本等

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== TSP算法选择器（在此处切换以测试不同求解器） =====
tspSolver = @(costMatrix, nPts) TSP_ACO_v1_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA_v1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v1_1(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v2(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_v1(costMatrix, nPts);

% ===== Experiment Config =====
nRuns = 30;
tspFile = fullfile(fileparts(mfilename('fullpath')), '..', 'bayg29.tsp');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Parse bayg29 dataset =====
[fullDist29, cityCoords] = parseBayg29(tspFile);
nCities = 29;

fprintf('======== TSP Experiment: bayg29 ========\n');
fprintf('Algorithm: %s\n', func2str(tspSolver));
fprintf('Cities: %d | Runs: %d\n', nCities, nRuns);
fprintf('Start/Goal: City 1 (%.0f, %.0f)\n\n', cityCoords(1,1), cityCoords(1,2));

%% ===== 构建30×30成本矩阵（起点=城市1，终点=城市1） =====
% Layout: idx 1=city1(start), idx 2..29=city2..city28, idx 30=city1(goal)
costMatrix30 = zeros(30);
costMatrix30(1, 2:29)   = fullDist29(1, 2:29);      % start → cities 2..29
costMatrix30(2:29, 1)   = fullDist29(2:29, 1);      % cities 2..29 → start (sym)
costMatrix30(2:29, 30)  = fullDist29(2:29, 1);      % cities 2..29 → goal (=city1)
costMatrix30(30, 2:29)  = fullDist29(1, 2:29);      % goal → cities 2..29 (sym)
costMatrix30(2:29, 2:29)= fullDist29(2:29, 2:29);   % inter-city distances
costMatrix30(30, 1)     = 0;
costMatrix30(1, 30)     = 0;

%% ===== 30 Independent Runs =====
allOrders   = zeros(nRuns, 30);
allCosts    = zeros(nRuns, 1);
allHistories = cell(1, nRuns);

fprintf('Running %d trials...\n', nRuns);
ticTotal = tic;

for run = 1:nRuns
    [bestOrder, bestCost, history] = tspSolver(costMatrix30, 30);
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
fprintf('Total wall time: %.1fs\n', totalWallTime);

%% ===== Export Data =====
resultsFile = fullfile(resultsDir, 'exp_aco_bayg29_results.mat');
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'fullDist29', 'cityCoords', 'costMatrix30');
fprintf('\nResults saved to: %s\n', resultsFile);

%% ===== Figure 1: Optimal Path Map =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

% City scatter
scatter(cityCoords(:,1), cityCoords(:,2), 50, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'k', 'LineWidth', 0.5);

% City labels
for i = 1:nCities
    text(cityCoords(i,1) + 30, cityCoords(i,2) + 30, num2str(i), ...
        'FontSize', 7, 'Color', [0.2, 0.2, 0.2]);
end

% Best tour path
bestOrder30 = allOrders(bestRunIdx, :);
% Map back to original city indices (1→city1, 2..29→city2..29, 30→city1)
cityIdx = zeros(1, 30);
for i = 1:30
    if bestOrder30(i) == 1 || bestOrder30(i) == 30
        cityIdx(i) = 1;  % start/goal = city 1
    else
        cityIdx(i) = bestOrder30(i);  % city 2..29
    end
end

xPath = cityCoords(cityIdx, 1);
yPath = cityCoords(cityIdx, 2);
plot(xPath, yPath, '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.5);

% Start marker (city 1, start=goal, only label "Start")
plot(cityCoords(1,1), cityCoords(1,2), 'o', ...
    'MarkerSize', 14, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1), cityCoords(1,2) - 130, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — bayg29 (Cost: %.2f, Run #%d)', stats.bestCost, bestRunIdx), ...
    'FontSize', 13);
axis equal tight;
grid on; set(gca, 'GridAlpha', 0.15);
hold off;

%% ===== Figure 2: Convergence Curves (Iteration) =====
figure('Position', [820, 50, 700, 500], 'Color', 'w');

% Find max iterations across all runs, align by padding with final value
maxIter = max(cellfun(@(h) h.iterCount, allHistories));
costMatrix_iter = nan(nRuns, maxIter);

for run = 1:nRuns
    h = allHistories{run};
    n = h.iterCount;
    costMatrix_iter(run, 1:n) = h.bestCostHistory(1:n);
    % Forward-fill: remaining columns = final cost
    costMatrix_iter(run, n+1:end) = h.bestCostHistory(n);
end

% Median + 95% confidence band
medIter = median(costMatrix_iter, 1, 'omitnan');
loIter  = prctile(costMatrix_iter, 2.5, 1);
hiIter  = prctile(costMatrix_iter, 97.5, 1);
xIter = 1:maxIter;

fill([xIter, fliplr(xIter)], [loIter, fliplr(hiIter)], ...
    [0.2, 0.5, 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on;
plot(xIter, medIter, '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 2.0);
xlabel('Iteration'); ylabel('Best Cost');
title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 3: Convergence Curves (Time) =====
figure('Position', [820, 200, 700, 500], 'Color', 'w');

% Interpolate to common time grid
nTimePts = 500;
maxTime = max(cellfun(@(h) h.elapsedTime, allHistories));
tCommon = linspace(0, maxTime, nTimePts);
costMatrix_time = nan(nRuns, nTimePts);

for run = 1:nRuns
    h = allHistories{run};
    tRaw = h.timeHistory(1:h.iterCount);
    cRaw = h.bestCostHistory(1:h.iterCount);
    % Prepend t=0 with initial cost so interp range covers full time
    tRaw = [0; tRaw(:)];
    cRaw = [cRaw(1); cRaw(:)];
    % Remove duplicate time points for interp1
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
xlabel('Time (s)'); ylabel('Best Cost');
title('Convergence — Time (Median + 95% CI)');
legend('95% CI', 'Median', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 4: Cost Distribution =====
figure('Position', [50, 50, 750, 520], 'Color', 'w');

subplot(2,1,1);
histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5);
hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('Cost Distribution (N=%d)', nRuns));
legend(sprintf('Best=%.1f', stats.bestCost), sprintf('Median=%.1f', stats.medianCost), ...
    'Location', 'best');
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

function [distMatrix, coords] = parseBayg29(filepath)
%PARSEBAYG29 Parse bayg29.tsp file
%   Returns:
%     distMatrix - 29x29 symmetric distance matrix
%     coords     - 29x2 [x, y] display coordinates

    fid = fopen(filepath, 'r');
    if fid < 0
        error('Cannot open file: %s', filepath);
    end
    cleanup = onCleanup(@() fclose(fid));

    nCities = 29;
    distMatrix = zeros(nCities);
    coords = zeros(nCities, 2);
    inEdgeSection = false;
    inDisplaySection = false;

    % Parse upper-row distance entries
    rowVals = cell(nCities - 1, 1);
    currentRow = 1;

    while ~feof(fid)
        line = strtrim(fgetl(fid));

        if startsWith(line, 'EDGE_WEIGHT_SECTION')
            inEdgeSection = true;
            continue;
        elseif startsWith(line, 'DISPLAY_DATA_SECTION')
            inEdgeSection = false;
            inDisplaySection = true;
            continue;
        elseif strcmp(line, 'EOF')
            break;
        end

        if inEdgeSection && ~isempty(line) && ~startsWith(line, 'DISPLAY')
            nums = sscanf(line, '%f');
            if ~isempty(nums)
                rowVals{currentRow} = nums;
                currentRow = currentRow + 1;
            end
        elseif inDisplaySection && ~isempty(line) && ~startsWith(line, 'EOF')
            parts = sscanf(line, '%f');
            if length(parts) >= 3
                cityIdx = parts(1);
                coords(cityIdx, :) = [parts(2), parts(3)];
            end
        end
    end

    % Reconstruct symmetric matrix from UPPER_ROW
    for i = 1:(nCities - 1)
        vals = rowVals{i};
        expectedLen = nCities - i;
        if length(vals) ~= expectedLen
            warning('Row %d: expected %d entries, got %d', i, expectedLen, length(vals));
            vals = vals(1:min(expectedLen, length(vals)));
        end
        for k = 1:length(vals)
            j = i + k;
            distMatrix(i, j) = vals(k);
            distMatrix(j, i) = vals(k);
        end
    end
end
