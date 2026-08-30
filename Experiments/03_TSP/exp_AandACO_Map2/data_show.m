%% plot_results — 从已保存的 .mat 结果文件加载数据并绘制全部图表
%  用法: 修改下方 resultsFile 路径后直接运行

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== 指定结果文件 =====
resultsFile = fullfile(fileparts(mfilename('fullpath')), 'results', 'C_Map2_3.mat');

%% ===== 加载数据 =====
if ~exist(resultsFile, 'file')
    error('结果文件不存在: %s', resultsFile);
end
data = load(resultsFile);

% 必要字段
allOrders    = data.allOrders;
allCosts     = data.allCosts;
allHistories = data.allHistories;
bestRunIdx   = data.bestRunIdx;
stats        = data.stats;
bestOrder    = data.bestOrder;
fullSmoothPath = data.fullSmoothPath;
navPoints    = data.navPoints;
staticObstacles = data.staticObstacles;
mapSize      = data.mapSize;
nRuns        = data.nRuns;

nPts = size(navPoints, 1);

fprintf('已加载: %s\n', resultsFile);
fprintf('运行次数: %d | Best: %.4f (Run #%d) | Avg: %.4f±%.4f\n', ...
    nRuns, stats.bestCost, bestRunIdx, stats.avgCost, stats.stdCost);

%% ===== 坐标转换 ([row,col] → [x,y]) =====
navXY = [navPoints(:,2) - 0.5, navPoints(:,1) - 0.5];

%% ===== Figure 1: 地图 + 最优平滑路径 =====
figure('Position', [50, 50, 720, 680], 'Color', 'w');
ax = axes; hold(ax, 'on');

for i = 1:size(staticObstacles, 1)
    rectangle(ax, 'Position', [staticObstacles(i,2)-1, staticObstacles(i,1)-1, 1, 1], ...
        'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', 'none');
end

for i = 1:nPts
    scatter(ax, navXY(i,1), navXY(i,2), 22, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
    text(ax, navXY(i,1) + 0.5, navXY(i,2) + 0.5, num2str(i), ...
        'FontSize', 5.5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end

plot(ax, fullSmoothPath(:,1), fullSmoothPath(:,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.2);

plot(ax, navXY(1,1), navXY(1,2), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(1,1), navXY(1,2) - 1.8, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

plot(ax, navXY(end,1), navXY(end,2), '^', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(end,1), navXY(end,2) + 1.8, 'Goal', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);

xlim(ax, [0, mapSize]); ylim(ax, [0, mapSize]);
set(ax, 'YDir', 'normal', 'XTick', 0:5:mapSize, 'YTick', 0:5:mapSize);
grid(ax, 'on'); set(ax, 'GridAlpha', 0.15);
xlabel('X (列)'); ylabel('Y (行)');
title(sprintf('Map1 — 最优路径 (Cost: %.4f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 12);
hold(ax, 'off');

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
title('收敛 — 迭代维度 (中位数 + 95% CI)');
legend('95% CI', 'Median', sprintf('Best=%.4f', stats.bestCost), 'Location', 'northeast'); grid on; hold off;

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
title('收敛 — 时间维度 (中位数 + 95% CI)');
legend('95% CI', 'Median', sprintf('Best=%.4f', stats.bestCost), 'Location', 'northeast'); grid on; hold off;

%% ===== Figure 4: 成本分布 =====
figure('Position', [50, 760, 750, 520], 'Color', 'w');
subplot(2,1,1);
histogram(allCosts, 10, 'FaceColor', [0.2, 0.5, 0.9], 'EdgeColor', 'k', 'LineWidth', 0.5); hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('成本分布 (N=%d)', nRuns));
legend(sprintf('Best=%.4f', stats.bestCost), sprintf('Median=%.4f', stats.medianCost), 'Location', 'best');
grid on; hold off;
subplot(2,1,2);
boxplot(allCosts, 'Orientation', 'horizontal', 'Widths', 0.5); hold on;
scatter(stats.bestCost, 1, 60, [0.9, 0.2, 0.2], 'filled', 'MarkerEdgeColor', 'k');
xlabel('Best Cost'); title('成本箱线图');
set(gca, 'YTickLabel', []); grid on; hold off;

fprintf('\n全部图片已生成。(未自动保存)\n');
