%% exp_GoogleMap_path — GoogleMap 二值图栅格化 + 多目标 TSP 路径测试
%  流程: PNG(558×390) → 6×6 降采样 93 列 × 65 行栅格 → Map(93) 方形填充
%        全局规划(AStar_v3_1) → 拐角裁剪(SimplifyPath) → 平滑(SmoothPath)
%        → 障碍感知距离矩阵 → TSP(TSP_ACO_v2_4) → 拼接最优路径 → 绘图
%
%  坐标约定: 配置区使用栅格 [row, col]
%    row 1 = 图像底部（共 65 行），y 轴向上为正；col 1 = 图像左侧（共 93 列）
%    降采样规则: 每格 6×6 像素，黑色像素占比 ≥30% 判为障碍
%    矩形 65×93 地图扩为 93×93 方形（行 66:93 整行填障碍），核心代码零改动
%    绘图 YDir=normal（y 向上为正），图面与 PNG 原图一致
%
%  用法: 修改下方 Config 中 startPoint / goalPoint / midPoints 即可自定义导航点

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% ===== Config（可在此自定义起点/终点/中间节点，均为 [row, col] 栅格坐标）=====
startPoint = [38, 87];                       % 起点
goalPoint  = [12, 23];                       % 终点
midPoints  = [52,67; 52,39; 35,54; 28,41; 40,25; 13,36; 19,53];       % K×2 中间节点（可任意增删）

planner   = @(map, s, g) AStar_v3_1(map, s, g, 0);   % 全局规划器，返回 [path, info]
tspSolver = @(costMatrix, nPts) TSP_ACO_v2_4(costMatrix, nPts);  % TSP 求解器

nRuns = 1;           % TSP 重复运行次数
safetyMargin = 0.4;  % SimplifyPath 拐角裁剪安全距离（栅格单位）
smoothDensity = 10;  % SmoothPath 平滑插值密度

% ===== 图片 → 栅格参数 =====
imgFile    = fullfile(fileparts(mfilename('fullpath')), 'GoogleMap_01.png');
blockSize  = 6;      % 每格像素数（558=93×6, 390=65×6）
nRow       = 65;     % 栅格行数（图像高 390 / 6）
nCol       = 93;     % 栅格列数（图像宽 558 / 6）
blackRatio = 0.3;    % 黑色像素占比 ≥ 此值 → 障碍
% ========================================================================

expDir     = fileparts(mfilename('fullpath'));
resultsDir = fullfile(expDir, 'results');
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end

fprintf('======== GoogleMap 多目标路径测试 ========\n');
fprintf('Planner: %s | TSP: %s\n', func2str(planner), func2str(tspSolver));

%% ===== 二值图 → 栅格占用图 =====
img = imread(imgFile);
if size(img, 3) >= 3
    gray = mean(double(img(:,:,1:3)), 3);   % RGB 手动转灰度（避免工具箱依赖）
else
    gray = double(img);
end
blackMask = gray / 255 < 0.5;               % 黑 = 像素值 < 128

occGrid = zeros(nRow, nCol);
for r = 1:nRow
    for c = 1:nCol
        blk = blackMask((r-1)*blockSize + 1 : r*blockSize, ...
                        (c-1)*blockSize + 1 : c*blockSize);
        occGrid(r, c) = mean(blk(:)) >= blackRatio;   % ≥30% 黑 → 障碍
    end
end
occGrid = flipud(occGrid);   % 翻转: 行 1 = 图像底部，y 轴向上为正
fprintf('图片: %s (%s) | 栅格: %d 列 × %d 行 | 障碍: %d 格 (%.1f%%)\n', ...
    imgFile, mat2str(size(blackMask, [1, 2])), nCol, nRow, ...
    sum(occGrid(:)), mean(occGrid(:)) * 100);

%% ===== 构建 Map 对象（矩形 30×43 → 填充为 43×43 方形）=====
map = Map(nCol);   % 43×43
fullGrid = zeros(nCol, nCol, 'uint8');
fullGrid(1:nRow, 1:nCol) = occGrid;   % 真实地图区域
fullGrid(nRow+1:end, :) = 1;          % 行 31:43 整行填障碍，路径限制在真实区域
[obsR, obsC] = find(fullGrid);
map.setStaticObstacle(obsR, obsC);
occGridFull = map.getOccupancyGrid();   % 43×43，供 SimplifyPath 使用

%% ===== 导航点组装与校验 =====
navPoints = [startPoint; midPoints; goalPoint];   % 第1=起点，最后=终点，中间=目标
nPts = size(navPoints, 1);
nMid = nPts - 2;
fprintf('导航点: %d 个 (起点 + %d 中间 + 终点)\n', nPts, nMid);

for i = 1:nPts
    rp = navPoints(i, 1); cp = navPoints(i, 2);
    if rp < 1 || rp > nRow || cp < 1 || cp > nCol
        error('导航点 %d [%d,%d] 越界（有效范围 row 1~%d, col 1~%d）', ...
            i, rp, cp, nRow, nCol);
    end
    if occGrid(rp, cp)
        error('导航点 %d [%d,%d] 落在障碍栅格上，请更换坐标', i, rp, cp);
    end
end
fprintf('Start: [%d,%d] | Goal: [%d,%d] | 中间点: %s\n', ...
    startPoint(1), startPoint(2), goalPoint(1), goalPoint(2), mat2str(midPoints));

%% ===== 构建障碍感知距离矩阵（A* → 简化 → 平滑 后的路径长度）=====
fprintf('\n计算 %d×%d 障碍感知距离矩阵 (A* → SimplifyPath → SmoothPath)...\n', nPts, nPts);
distMatrix = zeros(nPts);
tDist = tic;
for i = 1:(nPts - 1)
    for j = (i + 1):nPts
        [~, cost] = planSmoothPath(planner, map, occGridFull, ...
            navPoints(i,:), navPoints(j,:), safetyMargin, smoothDensity);
        distMatrix(i, j) = cost;
        distMatrix(j, i) = cost;
    end
end
distMatrixTime = toc(tDist);
nInf = sum(isinf(distMatrix(:)));
if nInf > 0
    warning('存在 %d 个不可达点对 (inf 距离)', nInf);
end
fprintf('距离矩阵计算耗时: %.5fs\n', distMatrixTime);

%% ===== 运行 TSP（可多次）=====
costMatrix = distMatrix;   % 第1=起点，最后=终点，中间=目标
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
fprintf('Best:   %.4f  (Run #%d)\n', stats.bestCost, bestRunIdx);
fprintf('Avg:    %.4f ± %.4f\n', stats.avgCost, stats.stdCost);
fprintf('TSP 总耗时: %.5fs\n', totalTSPTime);
fprintf('最优遍历顺序: %s\n', mat2str(allOrders(bestRunIdx, :)));

%% ===== 生成最终路径（简化+平滑后的连续路径拼接）=====
bestOrder = allOrders(bestRunIdx, :);
segSmooth = cell(nPts - 1, 1);
segCost   = zeros(nPts - 1, 1);
for k = 1:(nPts - 1)
    i = bestOrder(k); j = bestOrder(k + 1);
    [segSmooth{k}, ~] = planSmoothPath(planner, map, occGridFull, ...
        navPoints(i,:), navPoints(j,:), safetyMargin, smoothDensity);
    segCost(k) = distMatrix(i, j);
end
fullSmoothPath = segSmooth{1};
for k = 2:length(segSmooth)
    if size(segSmooth{k}, 1) >= 2
        fullSmoothPath = [fullSmoothPath; segSmooth{k}(2:end, :)];
    end
end
pathCostCheck = sum(segCost);
fprintf('拼接路径总代价(距离矩阵): %.4f (TSP bestCost: %.4f)\n', pathCostCheck, stats.bestCost);

%% ===== 导出结果 =====
resultsFile = fullfile(resultsDir, 'exp_GoogleMap_path_results.mat');
save(resultsFile, 'occGrid', 'navPoints', 'startPoint', 'goalPoint', 'midPoints', ...
    'distMatrix', 'distMatrixTime', 'allOrders', 'allCosts', 'allHistories', ...
    'bestRunIdx', 'bestOrder', 'stats', 'segSmooth', 'fullSmoothPath', ...
    'nRuns', 'safetyMargin', 'smoothDensity', 'blockSize', 'blackRatio');
fprintf('\n结果已保存: %s\n', resultsFile);

%% ===== Figure 1: 地图 + 导航点 + 最优平滑路径 =====
%  连续坐标: [x, y] = [col - 0.5, row - 0.5]；YDir=normal，y 向上为正（图面与 PNG 一致）
navXY = [navPoints(:,2) - 0.5, navPoints(:,1) - 0.5];

figure('Position', [80, 80, 900, 640], 'Color', 'w');
ax = axes; hold(ax, 'on');

% 障碍物（黑色矩形，仅真实 30 行区域）
[obsRows, obsCols] = find(occGrid);
for i = 1:length(obsRows)
    rectangle(ax, 'Position', [obsCols(i)-1, obsRows(i)-1, 1, 1], ...
        'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', 'none');
end

% 中间目标点编号
for i = 2:(nPts - 1)
    scatter(ax, navXY(i,1), navXY(i,2), 26, [0.2, 0.5, 0.9], 'filled', 'MarkerEdgeColor', 'none');
    text(ax, navXY(i,1) + 0.6, navXY(i,2) + 0.6, num2str(i - 1), ...
        'FontSize', 10, 'Color', [0.1, 0.3, 0.7], 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center');
end

% 最优平滑路径
plot(ax, fullSmoothPath(:,1), fullSmoothPath(:,2), '-', ...
    'Color', [0.9, 0.3, 0.2], 'LineWidth', 1.2);

% 起点（圆形）/ 终点（三角形）
plot(ax, navXY(1,1), navXY(1,2), 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(1,1), navXY(1,2) - 1.2, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
plot(ax, navXY(end,1), navXY(end,2), '^', 'MarkerSize', 8, ...
    'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
text(ax, navXY(end,1), navXY(end,2) + 1.2, 'Goal', ...
    'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);

xlim(ax, [0, nCol]); ylim(ax, [0, nRow]);
set(ax, 'YDir', 'normal', ...               % y 向上为正（栅格已翻转，图面仍与 PNG 一致）
    'XTick', 0:5:nCol, 'YTick', 0:5:nRow, 'FontSize', 12);
axis(ax, 'equal');
grid(ax, 'on'); set(ax, 'GridAlpha', 0.15);
xlabel(ax, 'X (列)'); ylabel(ax, 'Y (行)');
title(sprintf('GoogleMap — A*+TSP 最优路径 (Cost: %.4f, %d 中间点)', ...
    stats.bestCost, nMid), 'FontSize', 12);
hold(ax, 'off');

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
