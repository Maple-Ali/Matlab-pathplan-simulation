function DataViewerUI(results, simParams)
%DATAVIEWERUI 数据展示界面
%   DataViewerUI(results, simParams) 显示仿真统计数据

fig = uifigure('Name', '仿真数据报告', ...
    'Position', [300, 200, 500, 480], 'Resize', 'off');

% 标题
uilabel(fig, 'Text', '仿真数据报告', ...
    'Position', [150, 440, 200, 30], ...
    'FontSize', 16, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% 基本信息
y = 400;
infoPanel = uipanel(fig, 'Title', '基本信息', 'Position', [20, y - 100, 460, 110]);

uilabel(infoPanel, 'Text', sprintf('全局规划算法: %s', simParams.globalAlgo), ...
    'Position', [10, 60, 200, 20]);
uilabel(infoPanel, 'Text', sprintf('局部规划算法: %s', simParams.localAlgo), ...
    'Position', [230, 60, 200, 20]);
uilabel(infoPanel, 'Text', sprintf('拐角裁剪: %s', boolToStr(simParams.enableSimplify)), ...
    'Position', [10, 35, 200, 20]);
uilabel(infoPanel, 'Text', sprintf('路径平滑: %s', boolToStr(simParams.enableSmooth)), ...
    'Position', [230, 35, 200, 20]);
uilabel(infoPanel, 'Text', sprintf('地图大小: %d×%d', simParams.mapSize, simParams.mapSize), ...
    'Position', [10, 10, 200, 20]);

% 路径统计
y = 280;
pathPanel = uipanel(fig, 'Title', '路径统计', 'Position', [20, y - 100, 460, 110]);

uilabel(pathPanel, 'Text', sprintf('原始路径长度: %.2f 单位', results.rawPathLen), ...
    'Position', [10, 60, 220, 20]);
if simParams.enableSimplify
    uilabel(pathPanel, 'Text', sprintf('简化后长度: %.2f 单位', results.simplePathLen), ...
        'Position', [10, 35, 220, 20]);
end
if simParams.enableSmooth
    uilabel(pathPanel, 'Text', sprintf('平滑后长度: %.2f 单位', results.smoothPathLen), ...
        'Position', [10, 10, 220, 20]);
end
uilabel(pathPanel, 'Text', sprintf('规划耗时: %.1f 毫秒', results.planTime * 1000), ...
    'Position', [240, 60, 200, 20]);

% 仿真统计
y = 160;
simPanel = uipanel(fig, 'Title', '仿真统计', 'Position', [20, y - 100, 460, 110]);

uilabel(simPanel, 'Text', sprintf('总仿真时间: %.2f 秒', results.totalTime), ...
    'Position', [10, 60, 200, 20]);
uilabel(simPanel, 'Text', sprintf('机器人行驶距离: %.2f 单位', results.totalDistance), ...
    'Position', [10, 35, 200, 20]);

if results.collision
    colLabel = uilabel(simPanel, 'Text', '碰撞状态: 发生碰撞!', ...
        'Position', [10, 10, 200, 20]);
    colLabel.FontColor = [1, 0, 0];
else
    uilabel(simPanel, 'Text', '碰撞状态: 安全', ...
        'Position', [10, 10, 200, 20]);
end

uilabel(simPanel, 'Text', sprintf('TSP 耗时: %.1f 毫秒', results.tspTime * 1000), ...
    'Position', [240, 60, 200, 20]);

if isfield(results, 'tspCost')
    uilabel(simPanel, 'Text', sprintf('TSP 总成本: %.2f', results.tspCost), ...
        'Position', [240, 35, 200, 20]);
end

% 访问顺序
if ~isempty(results.visitOrder)
    orderStr = '';
    for i = 1:size(results.visitOrder, 1)
        orderStr = [orderStr, sprintf('(%d,%d) ', ...
            results.visitOrder(i, 1), results.visitOrder(i, 2))];
    end
    uilabel(simPanel, 'Text', ['访问顺序: ', orderStr], ...
        'Position', [10, 0, 440, 20]);
end

% 关闭按钮
uibutton(fig, 'Text', '关闭', 'Position', [200, 20, 100, 30], ...
    'ButtonPushedFcn', @(~,~) close(fig));
end

function s = boolToStr(b)
    if b, s = '是'; else, s = '否'; end
end
