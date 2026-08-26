%% plot_kroA150_results — 加载实验结果并绘制全部图表
%  依赖文件: results/exp_aco_kroA150_results.mat
%  生成图形: 最优路径图、迭代收敛曲线、时间收敛曲线、成本分布

clear variables; close all;

% ===== 结果文件路径（按需修改） =====
resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
%此处设置绘图的数据名称
resultsFile = fullfile(resultsDir, 'exp_kroA150_city1_to_city150.mat');

if ~exist(resultsFile, 'file')
    error('结果文件不存在: %s\n请先运行实验脚本生成数据。', resultsFile);
end

fprintf('加载实验结果: %s\n', resultsFile);
loaded = load(resultsFile);

% ----- 提取必要变量 -----
allOrders    = loaded.allOrders;      % [nRuns x nPts]
allCosts     = loaded.allCosts;       % [nRuns x 1]
allHistories = loaded.allHistories;   % cell array, 每个元素是一个结构体
bestRunIdx   = loaded.bestRunIdx;
stats        = loaded.stats;          % 包含 bestCost, worstCost, avgCost, stdCost, medianCost
cityCoords   = loaded.cityCoords;     % [150 x 2]
costMatrix   = loaded.costMatrix;     % [151 x 151]

% 从数据中推断尺寸信息
nPts   = size(costMatrix, 1);        % 应为 151
nCities = nPts - 1;                  % 应为 150
nRuns  = size(allOrders, 1);

% 如果需要 fullDist150 做其他分析，可以取出: fullDist150 = loaded.fullDist150;

%% ===== 打印统计信息 =====
fprintf('\n======== 已加载实验结果 ========\n');
fprintf('城市数: %d | 运行次数: %d\n', nCities, nRuns);
fprintf('已知最优解: 26524\n');
fprintf('----- 成本统计 -----\n');
fprintf('最优:   %.2f  (Run #%d)\n', stats.bestCost, bestRunIdx);
fprintf('最差:   %.2f\n', stats.worstCost);
fprintf('平均:   %.2f ± %.2f\n', stats.avgCost, stats.stdCost);
fprintf('中位数: %.2f\n', stats.medianCost);
fprintf('与最优解差距: %.1f%%\n', (stats.bestCost - 26524) / 26524 * 100);

%% ===== Figure 1: 最优路径图 =====
figure('Position', [50, 50, 750, 680], 'Color', 'w');
hold on;

scatter(cityCoords(:,1), cityCoords(:,2), 25, [0.2, 0.5, 0.9], 'filled', ...
    'MarkerEdgeColor', 'none');

% 每隔15个城市标注编号
labelStep = 15;
for i = 1:labelStep:nCities
    text(cityCoords(i,1) + 30, cityCoords(i,2) + 30, num2str(i), ...
        'FontSize', 6, 'Color', [0.2, 0.2, 0.2]);
end

% 最优路径
bestOrderAll = allOrders(bestRunIdx, :);
cityIdx = zeros(1, nPts);
for i = 1:nPts
    if bestOrderAll(i) == 1 || bestOrderAll(i) == nPts
        cityIdx(i) = 1;
    else
        cityIdx(i) = bestOrderAll(i);
    end
end
plot(cityCoords(cityIdx,1), cityCoords(cityIdx,2), '-', ...
    'Color', [0.9, 0.3, 0.2], 'LineWidth', 0.8);

% 起点标记
plot(cityCoords(1,1), cityCoords(1,2), 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], ...
    'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(cityCoords(1,1) + 60, cityCoords(1,2) - 80, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, ...
    'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);

xlabel('X'); ylabel('Y');
title(sprintf('Best TSP Tour — kroA150 (Cost: %.2f, Run #%d)', ...
    stats.bestCost, bestRunIdx), 'FontSize', 13);
axis equal tight;
grid on; set(gca, 'GridAlpha', 0.1);
hold off;

%% ===== Figure 2: 迭代收敛曲线 =====
figure('Position', [820, 50, 700, 500], 'Color', 'w');

maxIter = max(cellfun(@(h) h.iterCount, allHistories));
costMatrix_iter = nan(nRuns, maxIter);

for run = 1:nRuns
    h = allHistories{run};
    n = h.iterCount;
    costMatrix_iter(run, 1:n) = h.bestCostHistory(1:n);
    % 达到最大迭代次数后保持终值
    costMatrix_iter(run, n+1:end) = h.bestCostHistory(n);
end

medIter = median(costMatrix_iter, 1, 'omitnan');
loIter  = prctile(costMatrix_iter, 2.5, 1);
hiIter  = prctile(costMatrix_iter, 97.5, 1);
xIter   = 1:maxIter;

fill([xIter, fliplr(xIter)], [loIter, fliplr(hiIter)], ...
    [0.2, 0.5, 0.9], 'FaceAlpha', 0.15, 'EdgeColor', 'none');
hold on;
plot(xIter, medIter, '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 2.0);
yline(26524, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title('Convergence — Iteration (Median + 95% CI)');
legend('95% CI', 'Median', 'Optimal (26524)', 'Location', 'northeast');
grid on; hold off;

%% ===== Figure 3: 时间收敛曲线 =====
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
        costMatrix_time(run, :) = interp1(tUnique, cUnique, tCommon, ...
            'linear', cUnique(end));
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
legend('95% CI', 'Median', 'Optimal (26524)', 'Location', 'northeast');
grid on; hold off;

%% ===== Figure 4: 成本分布 =====
figure('Position', [50, 760, 750, 520], 'Color', 'w');

subplot(2,1,1);
histogram(allCosts, 12, 'FaceColor', [0.2, 0.5, 0.9], ...
    'EdgeColor', 'k', 'LineWidth', 0.5);
hold on;
xline(stats.bestCost, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.5);
xline(stats.medianCost, '--', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Best Cost'); ylabel('Frequency');
title(sprintf('Cost Distribution (N=%d)', nRuns));
legend(sprintf('Best=%.1f', stats.bestCost), ...
       sprintf('Median=%.1f', stats.medianCost), 'Location', 'best');
grid on; hold off;

subplot(2,1,2);
boxplot(allCosts, 'Orientation', 'horizontal', 'Widths', 0.5);
hold on;
scatter(stats.bestCost, 1, 60, [0.9, 0.2, 0.2], 'filled', ...
    'MarkerEdgeColor', 'k');
xlabel('Best Cost');
title('Cost Box Plot');
set(gca, 'YTickLabel', []);
grid on; hold off;

fprintf('\n所有图表已生成。\n');