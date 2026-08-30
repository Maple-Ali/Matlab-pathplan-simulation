%% exp_aco_kroB100_customSE — 可设定起点/终点城市编号的kroB100 TSP实验
%  数据集: kroB100 (100城市, EUC_2D坐标, 预计算距离矩阵)
%  与 exp_aco_kroB100.m 的区别：可自定义起点和终点城市编号
%  起点/终点可以是同一城市（回路），也可以是不同城市（路径）
%
%  用法：修改下方 startCity / endCity 后直接运行

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== TSP Algorithm Selector =====
%tspSolver = @(costMatrix, nPts) TSP_ACO_v2_4(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_ACO_X(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA_X(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_SA_v0_X(costMatrix, nPts);

% tspSolver = @(costMatrix, nPts) TSP_ACO(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0(costMatrix, nPts);


% ===== 起点/终点城市编号 (1~100) =====
startCity = 1;    % 起点城市编号
endCity   = 1;  % 终点城市编号（与起点相同 = 回路）

% ===== Experiment Config =====
nRuns = 40;
coordFile = fullfile(fileparts(mfilename('fullpath')), 'kroB100_coords.mat');
distFile  = fullfile(fileparts(mfilename('fullpath')), 'kroB100_dist_matrix.txt');
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

%% ===== Load kroB100 dataset =====
tmp = load(coordFile);  % → c100 (100×2)
cityCoords = tmp.c100;
fullDist100 = parseKroB100Dist(distFile);
nCities = 100;

% 校验输入
assert(startCity >= 1 && startCity <= nCities, 'startCity 必须在 1~%d 之间', nCities);
assert(endCity >= 1 && endCity <= nCities, 'endCity 必须在 1~%d 之间', nCities);

isLoop = (startCity == endCity);
fprintf('======== TSP Experiment: kroB100 (Custom Start/End) ========\n');
fprintf('Algorithm: %s\n', func2str(tspSolver));
fprintf('Cities: %d | Runs: %d\n', nCities, nRuns);
fprintf('Start: City %d (%.0f, %.0f)\n', startCity, cityCoords(startCity,1), cityCoords(startCity,2));
fprintf('Goal:  City %d (%.0f, %.0f)\n', endCity, cityCoords(endCity,1), cityCoords(endCity,2));
fprintf('Type:  %s\n', iff(isLoop, 'Loop (start=goal)', 'Path (start≠goal)'));
fprintf('\n');

%% ===== Build cost matrix =====
% 节点映射:
%   node 1       = startCity (起点)
%   node 2~nPts-1 = 其余城市 (按原始编号顺序，跳过 startCity 和 endCity)
%   node nPts     = endCity (终点)

% 确定中间城市序列 (排除 startCity 和 endCity；若回路则 startCity==endCity 只排除一次)
if isLoop
    midCities = setdiff(1:nCities, startCity);       % 99 个城市
else
    midCities = setdiff(1:nCities, [startCity, endCity]);  % 98 个城市
end
nPts = length(midCities) + 2;  % 回路=101, 非回路=100

costMatrix = zeros(nPts);

% start → midCities
costMatrix(1, 2:nPts-1) = fullDist100(startCity, midCities);
% midCities → start
costMatrix(2:nPts-1, 1) = fullDist100(midCities, startCity);
% midCities → goal
costMatrix(2:nPts-1, nPts) = fullDist100(midCities, endCity);
% goal → midCities
costMatrix(nPts, 2:nPts-1) = fullDist100(endCity, midCities);
% midCities ↔ midCities
costMatrix(2:nPts-1, 2:nPts-1) = fullDist100(midCities, midCities);

%% ===== 城市编号映射表 (用于结果还原) =====
nodeToCity = [startCity, midCities, endCity];  % 1×nPts

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
fprintf('Total wall time: %.1fs\n', totalWallTime);

%% ===== Export Data =====
startLabel = sprintf('city%d', startCity);
goalLabel  = sprintf('city%d', endCity);
resultsFile = fullfile(resultsDir, sprintf('exp_kroB100_%s_to_%s.mat', startLabel, goalLabel));
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'fullDist100', 'cityCoords', 'costMatrix', 'nodeToCity', 'startCity', 'endCity');
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

% Best tour path — map node indices back to city indices
bestOrderAll = allOrders(bestRunIdx, :);
cityIdx = nodeToCity(bestOrderAll);
if isLoop
    cityIdx = [cityIdx, cityIdx(1)];  % 回路: 闭合路径
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);

% Start marker
plot(cityCoords(startCity,1), cityCoords(startCity,2), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
% text(cityCoords(startCity,1) + 60, cityCoords(startCity,2) - 80, ...
%     sprintf('Start (C%d)', startCity), ...
%     'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
text(cityCoords(startCity,1) + 60, cityCoords(startCity,2) - 80, ...
    sprintf('Start', startCity), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

% Goal marker (if different from start)
if ~isLoop
    plot(cityCoords(endCity,1), cityCoords(endCity,2), 's', ...
        'MarkerSize', 8, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    text(cityCoords(endCity,1) + 60, cityCoords(endCity,2) - 80, ...
        sprintf('Goal (C%d)', endCity), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);
end

xlabel('X'); ylabel('Y');
% 回路时补充闭合边代价用于显示
dispCost = stats.bestCost;
if isLoop
    dispCost = dispCost + fullDist100(nodeToCity(allOrders(bestRunIdx, end)), startCity);
end
titleStr = sprintf('Best TSP Tour — kroB100 (C%d→C%d, Cost: %.2f, Run #%d)', ...
    startCity, endCity, dispCost, bestRunIdx);
title(titleStr, 'FontSize', 13);
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
xlabel('Iteration'); ylabel('Best Cost');
title(sprintf('Convergence — Iteration (Median + 95%% CI, C%d→C%d)', startCity, endCity));
legend('95% CI', 'Median', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 3: Convergence Curves (Time) =====
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
xlabel('Time (s)'); ylabel('Best Cost');
title(sprintf('Convergence — Time (Median + 95%% CI, C%d→C%d)', startCity, endCity));
legend('95% CI', 'Median', 'Location', 'northeast'); grid on;
hold off;

%% ===== Figure 4: Cost Distribution =====
figure('Position', [50, 200, 750, 520], 'Color', 'w');

subplot(2,1,1);
histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5);
hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('Cost Distribution (N=%d, C%d→C%d)', nRuns, startCity, endCity));
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

function distMatrix = parseKroB100Dist(distFile)
%PARSEKROB100DIST Parse kroB100 distance matrix (100x100, space-separated)
%   distMatrix - 100x100 symmetric distance matrix
    distMatrix = dlmread(distFile, ' ');
end

function result = iff(cond, a, b)
%IFF Conditional return
    if cond, result = a; else, result = b; end
end
