%% exp_smooth_path — SimplifyPath + SmoothPath 路径平滑效果测试
%  地图: 简化路径测试
%  对比简化路径与平滑路径，原始路径不显示

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% 加载地图
[map, mapData] = loadPresetMap('杂乱不规则');
startGrid = mapData.startPoint;
goalGrid  = mapData.goalPoint;
n = map.mapSize;
occGrid = map.getOccupancyGrid();

fprintf('地图: 简化路径测试 | 起点=[%d,%d] | 终点=[%d,%d]\n', startGrid, goalGrid);

%% AStar_v1 规划原始路径
fprintf('运行 AStar_v1 (alpha=0.3, beta=3) ...\n');
[path, info] = AStar_v1(map, startGrid, goalGrid, 0, [], 0.3, 3);

%% SimplifyPath 拐角裁剪
fprintf('运行 SimplifyPath ...\n');
simplePath = SimplifyPath(path, occGrid, n, 0.3);

%% SmoothPath 三次样条平滑
fprintf('运行 SmoothPath ...\n');
smoothPath = SmoothPath(simplePath, 10);  % 输出 [x, y] 连续坐标

%% 路径代价计算
calcCost = @(p) sum(sqrt(sum(diff(p).^2, 2)));
costSimple  = calcCost(simplePath);       % [row,col] → 近似欧氏距离
costSmooth  = calcCost(smoothPath);       % [x,y] 连续 → 精确欧氏距离

% simplePath 代价用连续坐标重新计算（更准确对比）
simpleCont  = [simplePath(:,2)-0.5, simplePath(:,1)-0.5];
costSimpleCont = calcCost(simpleCont);

fprintf('\n=== 结果 ===\n');
fprintf('简化路径: %d 节点, 代价=%.4f\n', size(simplePath,1), costSimpleCont);
fprintf('平滑路径: %d 节点, 代价=%.4f\n', size(smoothPath,1), costSmooth);

%% 绘图
figH = figure('Position', [100, 100, 700, 600], 'Color', 'w');
ax = axes(figH);
hold(ax, 'on');

% 障碍物（纯黑）
[obsRows, obsCols] = find(occGrid);
for i = 1:length(obsRows)
    rectangle(ax, 'Position', [obsCols(i)-1, obsRows(i)-1, 1, 1], ...
        'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 0.3);
end

% 简化路径（红色虚线）— [row,col] 转居中坐标
plot(ax, simplePath(:,2)-0.5, simplePath(:,1)-0.5, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0);

% 平滑路径（绿色实线）— SmoothPath 已输出 [x,y] 连续坐标，直接使用
plot(ax, smoothPath(:,1), smoothPath(:,2), '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);

% 起点和终点标记 — 栅格中央
plot(ax, startGrid(2)-0.5, startGrid(1)-0.5, 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(ax, goalGrid(2)-0.5, goalGrid(1)-0.5, '^', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

text(ax, startGrid(2)-0.5, startGrid(1)-1.8, 'Start', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(ax, goalGrid(2)-0.5, goalGrid(1)+1.0, 'End', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

xlim(ax, [0, n]);
ylim(ax, [0, n]);
set(ax, 'YDir', 'normal', 'XTick', 0:1:n, 'YTick', 0:1:n);
grid(ax, 'on');
set(ax, 'GridLineStyle', '-', 'GridAlpha', 0.15);
xlabel(ax, 'X');
ylabel(ax, 'Y');
title(ax, sprintf('Path Smoothing (SimplifyPath + SmoothPath)\nSimplified: %d pts, cost=%.2f | Smoothed: %d pts, cost=%.2f', ...
    size(simplePath,1), costSimpleCont, size(smoothPath,1), costSmooth), 'FontSize', 12);

% 图例
hSimp = plot(ax, NaN, NaN, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0, 'DisplayName', 'Simplified Path');
hSmth = plot(ax, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5, 'DisplayName', 'Smoothed Path');
legend(ax, [hSimp, hSmth], 'Location', 'southoutside', 'Orientation', 'horizontal');

hold(ax, 'off');
