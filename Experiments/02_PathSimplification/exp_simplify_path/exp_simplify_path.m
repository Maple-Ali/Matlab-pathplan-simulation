%% exp_simplify_path — AStar_v1 + SimplifyPath 拐角裁剪效果测试
%  地图: 简化路径测试
%  对比原始路径与简化路径的代价和可视化

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% 加载地图
[map, mapData] = loadPresetMap('简化路径测试');
startGrid = mapData.startPoint;
goalGrid  = mapData.goalPoint;
n = map.mapSize;
occGrid = map.getOccupancyGrid();

fprintf('地图: 简化路径测试 | 起点=[%d,%d] | 终点=[%d,%d]\n', startGrid, goalGrid);

%% AStar_v1 规划原始路径
fprintf('运行 AStar_v1 (alpha=0.3, beta=3) ...\n');
tic;
[path, info] = AStar_v1(map, startGrid, goalGrid, 0, [], 0.3, 3);
tAstar = toc;
fprintf('  扩展节点: %d | 路径点数: %d | 耗时: %.4fs\n', info.expandedNodes, size(path,1), tAstar);

%% 路径代价计算函数
calcCost = @(p) sum(sqrt(sum(diff(p).^2, 2)));

%% SimplifyPath 简化
fprintf('运行 SimplifyPath ...\n');
tic;
[simplePath] = SimplifyPath(path, occGrid, n, 0.3);
tSimple = toc;
fprintf('  简化后路径点数: %d | 耗时: %.4fs\n', size(simplePath,1), tSimple);

%% 计算路径代价
costOriginal = calcCost(path);
costSimple  = calcCost(simplePath);
fprintf('\n=== 结果 ===\n');
fprintf('原始路径: %d 节点, 代价=%.4f\n', size(path,1), costOriginal);
fprintf('简化路径: %d 节点, 代价=%.4f\n', size(simplePath,1), costSimple);
fprintf('节点减少: %d (%.1f%%)\n', size(path,1)-size(simplePath,1), ...
    (1-size(simplePath,1)/size(path,1))*100);
fprintf('代价变化: %+.4f (%.2f%%)\n', costSimple-costOriginal, ...
    (costSimple-costOriginal)/costOriginal*100);

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

% 原始路径（红色细虚线）— 栅格中央
plot(ax, path(:,2)-0.5, path(:,1)-0.5, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0);

% 简化路径（绿色细实线）— 栅格中央
plot(ax, simplePath(:,2)-0.5, simplePath(:,1)-0.5, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);

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
title(ax, sprintf('Path Simplification (SimplifyPath)\nOriginal: %d pts, cost=%.2f | Simplified: %d pts, cost=%.2f', ...
    size(path,1), costOriginal, size(simplePath,1), costSimple), 'FontSize', 12);

% 图例
hOrig = plot(ax, NaN, NaN, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0, 'DisplayName', 'Original Path');
hSimp = plot(ax, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5, 'DisplayName', 'Simplified Path');
legend(ax, [hOrig, hSimp], 'Location', 'southoutside', 'Orientation', 'horizontal');

hold(ax, 'off');
