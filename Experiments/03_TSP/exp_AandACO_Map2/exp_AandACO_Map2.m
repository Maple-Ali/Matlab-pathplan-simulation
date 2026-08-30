%% exp_AandACO_Map2 — A* + ACO 多目标导航实验
%  流程: 全局规划(AStar_v1) → 拐角裁剪(SimplifyPath) → 平滑(SmoothPath) → 距离矩阵
%        TSP(ACO_v2_4) → 遍历顺序 → 生成简化+平滑后的路径
%  数据: map2_dataset.mat (由 Map_2.mat 的 mapData 一次性转换)
%  起点 = navPoints(1)，终点 = navPoints(70)，中间 68 个 = 目标
%  距离矩阵每次运行只计算一次，缓存到 distMatrix.mat

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== Config（可在此切换算法 / 运行次数）=====
planner   = @(map, s, g) AStar_v1(map, s, g, 0);   % 全局规划器，返回 [path, info]

% tspSolver = @(costMatrix, nPts) TSP_ACO_v2_4(costMatrix, nPts);  % TSP 求解器
% tspSolver = @(costMatrix, nPts) TSP_ACO_X(costMatrix, nPts);
tspSolver = @(costMatrix, nPts) TSP_GA_X(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0_X(costMatrix, nPts);

% tspSolver = @(costMatrix, nPts) TSP_ACO(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_GA(costMatrix, nPts);
% tspSolver = @(costMatrix, nPts) TSP_SA_v0(costMatrix, nPts);


nRuns = 40;          % TSP 重复运行次数
mapSize = 80;
safetyMargin = 0.4;  % SimplifyPath 拐角裁剪安全距离（栅格单位）
smoothDensity = 10;  % SmoothPath 平滑插值密度

expDir      = fileparts(mfilename('fullpath'));
datasetFile = fullfile(expDir, 'map2_dataset.mat');
distFile    = fullfile(expDir, 'distMatrix.mat');
resultsDir  = fullfile(expDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

%% ===== 加载数据集 =====
ds = load(datasetFile);   % mapSize, staticObstacles, navPoints
staticObstacles = ds.staticObstacles;   % M×2 [row, col]
navPoints = ds.navPoints;               % 70×2 [row, col]
nPts = size(navPoints, 1);
nMid = nPts - 2;   % 中间目标数 = 68

fprintf('======== A* + ACO Experiment: Map2 ========\n');
fprintf('Planner: %s | TSP: %s\n', func2str(planner), func2str(tspSolver));
fprintf('Map: %dx%d | Nav points: %d (start + %d targets + goal) | TSP runs: %d\n', ...
    mapSize, mapSize, nPts, nMid, nRuns);
fprintf('Start: [%d,%d] | Goal: [%d,%d] | Obstacles: %d\n', ...
    navPoints(1,1), navPoints(1,2), navPoints(end,1), navPoints(end,2), size(staticObstacles,1));
fprintf('后处理: SimplifyPath(safety=%.1f) + SmoothPath(density=%d)\n', safetyMargin, smoothDensity);

% 构建 Map 对象（设置静态障碍）
map = Map(mapSize);
map.setStaticObstacle(staticObstacles(:,1), staticObstacles(:,2));
occGrid = map.getOccupancyGrid();

%% ===== 构建/加载距离矩阵（A* → 简化 → 平滑 后的路径长度）=====
if exist(distFile, 'file')
    dtmp = load(distFile);
    distMatrix = dtmp.distMatrix;
    distMatrixTime = dtmp.distMatrixTime;
    fprintf('\n距离矩阵已缓存加载 (distMatrix.mat)，上次计算耗时 %.2fs\n', distMatrixTime);
else
    fprintf('\n计算 %dx%d 障碍感知距离矩阵 (A* → SimplifyPath → SmoothPath)...\n', nPts, nPts);
    distMatrix = zeros(nPts);
    tDist = tic;
    for i = 1:(nPts - 1)
        for j = (i + 1):nPts
            [~, cost] = planSmoothPath(planner, map, occGrid, ...
                navPoints(i,:), navPoints(j,:), safetyMargin, smoothDensity);
            distMatrix(i, j) = cost;
            distMatrix(j, i) = cost;
        end
    end
    distMatrixTime = toc(tDist);
    % 检查可达性
    nInf = sum(isinf(distMatrix(:)));
    if nInf > 0
        warning('存在 %d 个不可达点对 (inf 距离)', nInf);
    end
    save(distFile, 'distMatrix', 'distMatrixTime');
    fprintf('距离矩阵计算耗时 %.2fs，已保存到 distMatrix.mat\n', distMatrixTime);
end

%% ===== 运行 TSP（可多次）=====
costMatrix = distMatrix;   % 70×70：第1=起点，第70=终点，2~69=目标
allOrders    = zeros(nRuns, nPts);
allCosts     = zeros(nRuns, 1);
allHistories = cell(1, nRuns);

fprintf('\n运行 %d 次 TSP...\n', nRuns);
ticTotal = tic;
for run = 1:nRuns
    [bestOrder, bestCost, history] = tspSolver(costMatrix, nPts);
    allOrders(run, :) = bestOrder;
    allCosts(run)     = bestCost;
    allHistories{run} = history;
    if isfield(history, 'stopReason'), stopInfo = history.stopReason;
    else, stopInfo = 'N/A'; end
    fprintf('  Run %2d: cost=%.4f | iter=%d | time=%.2fs | %s\n', ...
        run, bestCost, history.iterCount, history.elapsedTime, stopInfo);
end
totalTSPTime = toc(ticTotal);

%% ===== 统计 =====
[stats.bestCost, bestRunIdx] = min(allCosts);
stats.worstCost  = max(allCosts);
stats.avgCost    = mean(allCosts);
stats.stdCost    = std(allCosts);
stats.medianCost = median(allCosts);

fprintf('\n===== 结果 =====\n');
fprintf('距离矩阵计算耗时: %.2fs\n', distMatrixTime);
fprintf('Best:   %.4f  (Run #%d)\n', stats.bestCost, bestRunIdx);
fprintf('Worst:  %.4f\n', stats.worstCost);
fprintf('Avg:    %.4f ± %.4f\n', stats.avgCost, stats.stdCost);
fprintf('Median: %.4f\n', stats.medianCost);
fprintf('TSP 总耗时: %.2fs (%.2fs/run)\n', totalTSPTime, totalTSPTime / nRuns);
fprintf('最优遍历顺序: %s\n', mat2str(allOrders(bestRunIdx, :)));

%% ===== 生成最终路径（简化+平滑后的连续路径拼接）=====
bestOrder = allOrders(bestRunIdx, :);
segSmooth = cell(nPts - 1, 1);
segCost   = zeros(nPts - 1, 1);
for k = 1:(nPts - 1)
    i = bestOrder(k); j = bestOrder(k + 1);
    [segSmooth{k}, ~] = planSmoothPath(planner, map, occGrid, ...
        navPoints(i,:), navPoints(j,:), safetyMargin, smoothDensity);
    segCost(k) = distMatrix(i, j);   % 代价取距离矩阵（与 TSP 目标一致）
end
% 拼接总路径（连续 [x, y]，去掉每段起点避免重复）
fullSmoothPath = segSmooth{1};
for k = 2:length(segSmooth)
    if size(segSmooth{k}, 1) >= 2
        fullSmoothPath = [fullSmoothPath; segSmooth{k}(2:end, :)];
    end
end
% 一致性校验：距离矩阵代价之和应等于 TSP bestCost
pathCostCheck = sum(segCost);
fprintf('拼接路径总代价(距离矩阵): %.4f (TSP bestCost: %.4f)\n', pathCostCheck, stats.bestCost);

%% ===== 导出结果 =====
resultsFile = fullfile(resultsDir, 'exp_AandACO_Map2_results.mat');
save(resultsFile, 'allOrders', 'allCosts', 'allHistories', 'bestRunIdx', 'stats', ...
    'bestOrder', 'segSmooth', 'fullSmoothPath', 'distMatrix', 'distMatrixTime', ...
    'navPoints', 'staticObstacles', 'mapSize', 'nRuns', 'safetyMargin', 'smoothDensity');
fprintf('\n结果已保存: %s\n', resultsFile);

%% ===== 坐标转换辅助（[row,col] → 连续 [x,y]=[col-0.5,row-0.5]）=====
navXY = [navPoints(:,2) - 0.5, navPoints(:,1) - 0.5];   % N×2 [x, y]

%% ===== Figure 1: 地图 + 导航点 + 最优平滑路径 =====
figure('Position', [50, 50, 720, 680], 'Color', 'w');
ax = axes; hold(ax, 'on');

% 障碍物（黑色矩形）
for i = 1:size(staticObstacles, 1)
    rectangle(ax, 'Position', [staticObstacles(i,2)-1, staticObstacles(i,1)-1, 1, 1], ...
        'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', 'none');
end

% 导航点编号
for i = 1:nPts
    scatter(ax, navXY(i,1), navXY(i,2), 22, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
    text(ax, navXY(i,1) + 0.5, navXY(i,2) + 0.5, num2str(i), ...
        'FontSize', 5.5, 'Color', [0.2, 0.2, 0.2], 'HorizontalAlignment', 'center');
end

% 最优平滑路径（简化+平滑后的连续路径）
plot(ax, fullSmoothPath(:,1), fullSmoothPath(:,2), '-', 'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.2);

% 起点（圆形，缩小）
plot(ax, navXY(1,1), navXY(1,2), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(1,1), navXY(1,2) - 1.8, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
% 终点（三角形，缩小）
plot(ax, navXY(end,1), navXY(end,2), '^', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(end,1), navXY(end,2) + 1.8, 'Goal', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);

xlim(ax, [0, mapSize]); ylim(ax, [0, mapSize]);
set(ax, 'YDir', 'normal', 'XTick', 0:5:mapSize, 'YTick', 0:5:mapSize);
grid(ax, 'on'); set(ax, 'GridAlpha', 0.15);
xlabel('X (列)'); ylabel('Y (行)');
title(sprintf('Map2 — A*+ACO 最优路径 (Cost: %.4f, Run #%d)', stats.bestCost, bestRunIdx), 'FontSize', 12);
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

%% ===== 局部函数 =====

function [smoothPath, cost] = planSmoothPath(planner, map, occGrid, startGrid, goalGrid, safetyMargin, density)
%PLANSMOOTHPATH 全局规划 → 拐角裁剪 → 平滑，返回连续路径及其欧氏长度
%   smoothPath: M×2 连续坐标 [x, y]
%   cost:       平滑后路径总长度（欧氏距离累加）

    path = planner(map, startGrid, goalGrid);   % A* 栅格路径 [row, col]
    if isempty(path)
        smoothPath = [];
        cost = inf;
        return;
    end
    simplePath = SimplifyPath(path, occGrid, map.mapSize, safetyMargin);  % 拐角裁剪 [row, col]
    smoothPath = SmoothPath(simplePath, density);                          % 平滑 [x, y]
    if size(smoothPath, 1) < 2
        cost = 0;
    else
        cost = sum(sqrt(sum(diff(smoothPath, 1, 1).^2, 2)));
    end
end
