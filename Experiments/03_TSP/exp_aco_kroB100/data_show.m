%% data_show — 从已保存的 .mat 结果文件加载数据并绘制全部图表 (kroB100/kroA100/kroA150 通用)
%  用法: 修改下方 resultsFile 路径后直接运行

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== 指定结果文件 =====
resultsFile = fullfile(fileparts(mfilename('fullpath')), 'results', 'C_Map3_2.mat');

%% ===== 加载数据 =====
if ~exist(resultsFile, 'file')
    error('结果文件不存在: %s', resultsFile);
end
data = load(resultsFile);

% 必要字段 (kroB100/kroA100/kroA150 实验通用)
allOrders    = data.allOrders;
allCosts     = data.allCosts;
allHistories = data.allHistories;
bestRunIdx   = data.bestRunIdx;
stats        = data.stats;
cityCoords   = data.cityCoords;
nodeToCity   = data.nodeToCity;
startCity    = data.startCity;
endCity      = data.endCity;
nRuns        = size(allOrders, 1);
nPts         = size(allOrders, 2);
nCities      = size(cityCoords, 1);
isLoop       = (startCity == endCity);

fprintf('已加载: %s\n', resultsFile);
fprintf('城市: %d | 运行次数: %d | Start: C%d → End: C%d %s\n', ...
    nCities, nRuns, startCity, endCity, iff(isLoop, '(回路)', ''));
fprintf('Best: %.2f (Run #%d) | Avg: %.2f±%.2f\n', ...
    stats.bestCost, bestRunIdx, stats.avgCost, stats.stdCost);

%% ===== Figure 1: 城市坐标 + 最优 TSP 路径 =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

% 所有城市散点 + 编号
scatter(cityCoords(:,1), cityCoords(:,2), 30, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
for i = 1:nCities
    text(cityCoords(i,1) + 20, cityCoords(i,2) + 20, num2str(i), ...
        'FontSize', 5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end

% 最优路径
bestOrderAll = allOrders(bestRunIdx, :);
cityIdx = nodeToCity(bestOrderAll);
if isLoop
    cityIdx = [cityIdx, cityIdx(1)];  % 回路闭合
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);

% 起点标记
plot(cityCoords(startCity,1), cityCoords(startCity,2), 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(startCity,1) + 60, cityCoords(startCity,2) - 80, ...
    sprintf('Start (C%d)', startCity), ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

% 终点标记 (非回路时)
if ~isLoop
    plot(cityCoords(endCity,1), cityCoords(endCity,2), 's', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    text(cityCoords(endCity,1) + 60, cityCoords(endCity,2) - 80, ...
        sprintf('Goal (C%d)', endCity), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);
end

xlabel('X'); ylabel('Y');
titleStr = sprintf('Best TSP Tour (C%d→C%d, Cost: %.2f, Run #%d)', ...
    startCity, endCity, stats.bestCost, bestRunIdx);
title(titleStr, 'FontSize', 13);
axis equal tight;
grid on; set(gca, 'GridAlpha', 0.1);
hold off;

%% ===== Figure 2: 收敛曲线（迭代维度）=====
figure('Position', [820, 50, 700, 500], 'Color', 'w');
maxIter = max(cellfun(@(h) h.iterCount, allHistories));
cmIter = nan(nRuns, maxIter);
for run = 1:nRuns
    h = allHistories{run}; n = h.iterCount;
    cmIter(run, 1:n) = h.bestCostHistory(1:n);
    cmIter(run, n+1:end) = h.bestCostHistory(n);
end
medIter = median(cmIter, 1, 'omitnan');
loIter  = prctile(cmIter, 2.5, 1);
hiIter  = prctile(cmIter, 97.5, 1);
xIter = 1:maxIter;
fill([xIter, fliplr(xIter)], [loIter, fliplr(hiIter)], [0.2, 0.5, 0.9], ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none'); hold on;
plot(xIter, medIter, '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 2.0);
yline(stats.bestCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title(sprintf('Convergence — Iteration (C%d→C%d)', startCity, endCity));
legend('95% CI', 'Median', sprintf('Best=%.2f', stats.bestCost), 'Location', 'northeast'); grid on; hold off;

%% ===== Figure 3: 收敛曲线（时间维度）=====
figure('Position', [820, 580, 700, 500], 'Color', 'w');
nTimePts = 500;
maxTime = max(cellfun(@(h) h.elapsedTime, allHistories));
tCommon = linspace(0, maxTime, nTimePts);
cmTime = nan(nRuns, nTimePts);
for run = 1:nRuns
    h = allHistories{run};
    tRaw = h.timeHistory(1:h.iterCount);
    cRaw = h.bestCostHistory(1:h.iterCount);
    tRaw = [0; tRaw(:)]; cRaw = [cRaw(1); cRaw(:)];
    [tU, ia] = unique(tRaw); cU = cRaw(ia);
    if length(tU) >= 2
        cmTime(run, :) = interp1(tU, cU, tCommon, 'linear', cU(end));
    else
        cmTime(run, :) = cU(end);
    end
end
medTime = median(cmTime, 1, 'omitnan');
loTime  = prctile(cmTime, 2.5, 1);
hiTime  = prctile(cmTime, 97.5, 1);
fill([tCommon, fliplr(tCommon)], [loTime, fliplr(hiTime)], [0.8, 0.4, 0.2], ...
    'FaceAlpha', 0.15, 'EdgeColor', 'none'); hold on;
plot(tCommon, medTime, '-', 'Color', [0.8, 0.4, 0.2], 'LineWidth', 2.0);
yline(stats.bestCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost');
title(sprintf('Convergence — Time (C%d→C%d)', startCity, endCity));
legend('95% CI', 'Median', sprintf('Best=%.2f', stats.bestCost), 'Location', 'northeast'); grid on; hold off;

%% ===== Figure 4: 成本分布 =====
figure('Position', [50, 200, 750, 520], 'Color', 'w');
subplot(2,1,1);
histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5); hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('Cost Distribution (N=%d, C%d→C%d)', nRuns, startCity, endCity));
legend(sprintf('Best=%.2f', stats.bestCost), sprintf('Median=%.2f', stats.medianCost), 'Location', 'best');
grid on; hold off;
subplot(2,1,2);
boxplot(allCosts, 'Orientation', 'horizontal', 'Widths', 0.5); hold on;
scatter(stats.bestCost, 1, 60, [0.9, 0.2, 0.2], 'filled', 'MarkerEdgeColor', 'k');
xlabel('Best Cost'); title('Cost Box Plot');
set(gca, 'YTickLabel', []); grid on; hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ===== Local Functions =====
function result = iff(cond, a, b)
    if cond, result = a; else, result = b; end
end
