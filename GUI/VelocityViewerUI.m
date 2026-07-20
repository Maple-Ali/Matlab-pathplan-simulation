function VelocityViewerUI(results)
%VELOCITYVIEWERUI 机器人速度曲线窗口
%   VelocityViewerUI(results) 绘制仿真过程中各机器人合速度随时间变化曲线

numRobots = 1;
if isfield(results, 'numRobots')
    numRobots = results.numRobots;
end

% 创建窗口
fig = uifigure('Name', '机器人速度曲线', ...
    'Position', [250, 150, 700, 450], 'Resize', 'off');

% 标题
uilabel(fig, 'Text', '机器人速度曲线', ...
    'Position', [250, 415, 200, 25], ...
    'FontSize', 14, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% 坐标轴
ax = uiaxes(fig, 'Position', [60, 50, 580, 350]);
hold(ax, 'on');
grid(ax, 'on');
xlabel(ax, '时间 (s)');
ylabel(ax, '合速度大小');
title(ax, '各机器人速度-时间曲线');

% 找到最大速度（用于参考线和 y 轴上限）
maxSpeed = 0;
for r = 1:numRobots
    if isfield(results.robotDetails, 'speedHistory') && ...
       ~isempty(results.robotDetails(r).speedHistory)
        maxSpeed = max(maxSpeed, max(results.robotDetails(r).speedHistory));
    end
end
if maxSpeed == 0
    maxSpeed = 1.0;
end

% 绘制各机器人速度曲线
legendLabels = cell(1, numRobots);
for r = 1:numRobots
    if ~isfield(results.robotDetails, 'speedHistory') || ...
       isempty(results.robotDetails(r).speedHistory)
        continue;
    end
    t = results.robotDetails(r).timeHistory;
    v = results.robotDetails(r).speedHistory;
    c = plotTools('multiRobotColor', r);
    plot(ax, t, v, '-', 'Color', c, 'LineWidth', 1.5);
    legendLabels{r} = sprintf('机器人 %d', r);
end

% 最大速度参考线
yline(ax, maxSpeed, '--r', 'LineWidth', 1);
% 将参考线加入图例
hold(ax, 'on');
hRef = plot(ax, nan, nan, '--r', 'LineWidth', 1);
legendLabels{end+1} = sprintf('最大速度 = %.1f', maxSpeed);

legend(ax, legendLabels(~cellfun(@isempty, legendLabels)), 'Location', 'best');

% y 轴留 10% 余量
ylim(ax, [0, maxSpeed * 1.15]);

hold(ax, 'off');
end
