%% plot_path_on_map — 将实验路径覆盖到真实地图 GoogleMap.png 上
%  数据: results/exp_GoogleMap_path_results.mat（由 exp_GoogleMap_path.m 生成）
%  GoogleMap.png 与二值图同为 558×390，栅格映射一致（6 像素/格，93 列 × 65 行）
%
%  坐标: 直接使用实验的栅格连续坐标 [x, y] = [col - 0.5, row - 0.5]
%    row 1 = 图像底部，y 轴向上为正；图像 flipud 显示后与坐标系一致
%
%  绘图内容: 真实地图 + 最优平滑路径 + 按访问顺序编号的中间点 + Start/Goal
%  图片仅显示，不自动保存（saveOverlay=true 时输出叠加 PNG）

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

expDir = fileparts(mfilename('fullpath'));

% ===== Config =====
mapFile  = fullfile(expDir, 'GoogleMap_01.png');                        % 真实地图
resFile  = fullfile(expDir, 'results', 'exp_GoogleMap_path_results.mat');
saveOverlay = false;                       % true=输出叠加后的 PNG
outFile  = fullfile(expDir, 'results', 'exp_GoogleMap_path_overlay.png');
% ==================

%% ===== 加载数据 =====
S = load(resFile);   % fullSmoothPath, navPoints, bestOrder, stats, nRow/nCol 由 occGrid 得出
fullSmoothPath = S.fullSmoothPath;   % M×2 连续 [x, y]（y 向上）
navPoints      = S.navPoints;        % N×2 [row, col]，第1=起点，最后=终点
bestOrder      = S.bestOrder;        % 遍历顺序（navPoints 行号序列）
nPts = size(navPoints, 1);
nMid = nPts - 2;

occGrid = S.occGrid;                 % 65×93（行 1 = 图像底部）
nRow = size(occGrid, 1);
nCol = size(occGrid, 2);

img = imread(mapFile);
fprintf('真实地图: %s (%s)\n', mapFile, mat2str(size(img, [1, 2])));
fprintf('路径点数: %d | 导航点: %d (起点 + %d 中间 + 终点)\n', ...
    size(fullSmoothPath, 1), nPts, nMid);
fprintf('TSP 代价: %.4f | 遍历顺序: %s\n', S.stats.bestCost, mat2str(bestOrder));

% 导航点连续坐标 [x, y]（y 向上，row 1 = 底部）
navXY = [navPoints(:,2) - 0.5, navPoints(:,1) - 0.5];

% 中间点访问序号: bestOrder 中的位置（起点=第1段之前，中间点从 1 编号）
visitNum = zeros(nPts, 1);
for k = 1:numel(bestOrder)
    visitNum(bestOrder(k)) = k;   % 访问次序（1=起点）
end

%% ===== Figure: 真实地图 + 路径 =====
figure('Position', [80, 80, 1000, 720], 'Color', 'w');
ax = axes; hold(ax, 'on');

% 真实地图（flipud 使图像上下与 y 向上坐标系一致）
image(ax, [0, nCol], [0, nRow], flipud(img));
set(ax, 'YDir', 'normal');

% 最优平滑路径
plot(ax, fullSmoothPath(:,1), fullSmoothPath(:,2), '-', ...
    'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.8);

% 中间目标点（按访问顺序编号）
for i = 2:(nPts - 1)
    scatter(ax, navXY(i,1), navXY(i,2), 36, [1.0, 0.8, 0.0], 'filled', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 0.8);
    text(ax, navXY(i,1) + 1.0, navXY(i,2) + 1.0, num2str(visitNum(i) - 1), ...
        'FontSize', 11, 'Color', [1, 1, 1], 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', ...
        'BackgroundColor', [0.1, 0.1, 0.1], 'Margin', 1);
end

% 起点（圆形）/ 终点（三角形）
plot(ax, navXY(1,1), navXY(1,2), 'o', 'MarkerSize', 10, ...
    'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
text(ax, navXY(1,1), navXY(1,2) - 2.2, 'Start', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
    'Color', [1, 1, 1], 'BackgroundColor', [0.1, 0.5, 0.1], 'Margin', 2);
plot(ax, navXY(end,1), navXY(end,2), '^', 'MarkerSize', 10, ...
    'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.0);
text(ax, navXY(end,1), navXY(end,2) + 2.2, 'Goal', ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', ...
    'Color', [1, 1, 1], 'BackgroundColor', [0.7, 0.1, 0.1], 'Margin', 2);

axis(ax, 'equal');
xlim(ax, [0, nCol]); ylim(ax, [0, nRow]);
set(ax, 'FontSize', 12, 'XTick', 0:10:nCol, 'YTick', 0:10:nRow);
grid(ax, 'on'); set(ax, 'GridAlpha', 0.25);
xlabel(ax, 'X (列)', 'FontSize', 12);
ylabel(ax, 'Y (行)', 'FontSize', 12);
title(sprintf('GoogleMap — 最优路径 (Cost: %.4f, %d 中间点, 编号=访问顺序)', ...
    S.stats.bestCost, nMid), 'FontSize', 12);
hold(ax, 'off');

%% ===== 可选：保存叠加后的图片 =====
if saveOverlay
    exportgraphics(ax, outFile, 'Resolution', 300);
    fprintf('叠加图已保存: %s\n', outFile);
end

fprintf('\n图片已生成。(未自动保存)\n');

%% ===== 控制台：访问顺序明细 =====
fprintf('\n访问顺序:\n');
for k = 1:numel(bestOrder)
    i = bestOrder(k);
    if i == 1
        role = 'Start';
    elseif i == nPts
        role = 'Goal ';
    else
        role = sprintf('目标%d', visitNum(i) - 1);
    end
    fprintf('  %d. %s  [%d, %d]\n', k, role, navPoints(i,1), navPoints(i,2));
end
