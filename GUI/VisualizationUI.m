function [fig, ax] = VisualizationUI()
%VISUALIZATIONUI 可视化窗口
%   [fig, ax] = VisualizationUI() 创建并返回可视化窗口及其坐标轴

fig = uifigure('Name', '仿真可视化', ...
    'Position', [1250, 100, 600, 600], 'Resize', 'off');

ax = uiaxes(fig, 'Position', [30, 30, 540, 540]);
title(ax, '实时仿真视图');
xlabel(ax, 'X (列)');
ylabel(ax, 'Y (行)');

% 初始坐标轴设置
hold(ax, 'on');
axis(ax, 'equal');
grid(ax, 'on');
set(ax, 'GridAlpha', 0.3);
end
