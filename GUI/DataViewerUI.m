function DataViewerUI(results, simParams)
%DATAVIEWERUI 数据展示界面
%   DataViewerUI(results, simParams) 显示仿真统计数据（支持多机器人）
%   布局：左侧"基本信息"+"仿真统计"，右侧各机器人面板（含路径统计）

numRobots = 1;
if isfield(results, 'numRobots'), numRobots = results.numRobots; end

% 动态窗口尺寸
titlePad = 20;  % uipanel 标题栏占用空间
leftW = 300; rightW = 370; panelGap = 20;
leftX = 20; rightX = leftX + leftW + panelGap;
robotPanelH = 125;
rightH = numRobots * robotPanelH + (numRobots - 1) * 5;
infoH = 150;
simH = 150;
leftH = infoH + 15 + simH;
contentH = max(leftH, rightH);
figW = rightX + rightW + 20;
figH = contentH + 80;

fig = uifigure('Name', '仿真数据报告', ...
    'Position', [200, max(50, 700 - figH), figW, figH], 'Resize', 'off');

% 标题
uilabel(fig, 'Text', '仿真数据报告', ...
    'Position', [(figW - 200) / 2, figH - 40, 200, 30], ...
    'FontSize', 16, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% ==================== 左侧：基本信息 ====================
yInfoTop = figH - 80;
infoPanel = uipanel(fig, 'Title', '基本信息', ...
    'Position', [leftX, yInfoTop - infoH, leftW, infoH]);

p = infoH - titlePad;
uilabel(infoPanel, 'Text', sprintf('全局规划算法: %s', simParams.globalAlgo), ...
    'Position', [10, p - 25, 130, 20]);
uilabel(infoPanel, 'Text', sprintf('局部规划算法: %s', simParams.localAlgo), ...
    'Position', [150, p - 25, 140, 20]);
uilabel(infoPanel, 'Text', sprintf('拐角裁剪: %s', boolToStr(simParams.enableSimplify)), ...
    'Position', [10, p - 50, 130, 20]);
uilabel(infoPanel, 'Text', sprintf('路径平滑: %s', boolToStr(simParams.enableSmooth)), ...
    'Position', [150, p - 50, 140, 20]);

if numRobots > 1
    uilabel(infoPanel, 'Text', sprintf('机器人数量: %d (协同)', numRobots), ...
        'Position', [10, p - 75, 130, 20]);
else
    uilabel(infoPanel, 'Text', sprintf('地图大小: %d x %d', simParams.mapSize, simParams.mapSize), ...
        'Position', [10, p - 75, 130, 20]);
end
uilabel(infoPanel, 'Text', sprintf('规划耗时: %.1f 毫秒', results.planTime * 1000), ...
    'Position', [150, p - 75, 140, 20]);
uilabel(infoPanel, 'Text', sprintf('TSP 耗时: %.1f 毫秒', results.tspTime * 1000), ...
    'Position', [10, p - 100, 200, 20]);

% ==================== 左侧：仿真统计 ====================
ySimTop = yInfoTop - infoH - 15;
simPanel = uipanel(fig, 'Title', '仿真统计', ...
    'Position', [leftX, ySimTop - simH, leftW, simH]);

p = simH - titlePad;
if numRobots > 1
    uilabel(simPanel, 'Text', sprintf('总仿真时间: %.2f 秒 (%d 台并行)', results.totalTime, numRobots), ...
        'Position', [10, p - 25, 270, 20]);
    uilabel(simPanel, 'Text', sprintf('总行驶距离: %.2f 单位 (各机器人之和)', results.totalDistance), ...
        'Position', [10, p - 50, 270, 20]);
else
    uilabel(simPanel, 'Text', sprintf('总仿真时间: %.2f 秒', results.totalTime), ...
        'Position', [10, p - 25, 200, 20]);
    uilabel(simPanel, 'Text', sprintf('机器人行驶距离: %.2f 单位', results.totalDistance), ...
        'Position', [10, p - 50, 200, 20]);
end

if results.collision
    colLabel = uilabel(simPanel, 'Text', '碰撞状态: 发生碰撞!', ...
        'Position', [10, p - 75, 200, 20]);
    colLabel.FontColor = [1, 0, 0];
else
    uilabel(simPanel, 'Text', '碰撞状态: 安全', ...
        'Position', [10, p - 75, 200, 20]);
end

if isfield(results, 'tspCost')
    uilabel(simPanel, 'Text', sprintf('TSP 总成本: %.2f', results.tspCost), ...
        'Position', [10, p - 100, 200, 20]);
end

% ==================== 右侧：各机器人统计（含路径信息） ====================
if isfield(results, 'robotDetails')
    for r = 1:numRobots
        rd = results.robotDetails(r);
        rY = yInfoTop - r * (robotPanelH + 5);
        rPanel = uipanel(fig, 'Title', sprintf('机器人 %d 统计', r), ...
            'Position', [rightX, rY, rightW, robotPanelH]);

        p = robotPanelH - titlePad;
        nAssigned = size(rd.assignedTargets, 1);
        uilabel(rPanel, 'Text', sprintf('分配目标: %d 个', nAssigned), ...
            'Position', [10, p - 25, 110, 20]);
        uilabel(rPanel, 'Text', sprintf('行驶距离: %.2f 单位', rd.totalDistance), ...
            'Position', [130, p - 25, 120, 20]);
        uilabel(rPanel, 'Text', sprintf('规划耗时: %.1f 毫秒', results.planTime * 1000), ...
            'Position', [260, p - 25, 110, 20]);

        if isfield(rd, 'goalPoint') && ~isempty(rd.goalPoint)
            uilabel(rPanel, 'Text', sprintf('终点: (%d, %d)', rd.goalPoint(2), rd.goalPoint(1)), ...
                'Position', [10, p - 47, 170, 20]);
        end
        uilabel(rPanel, 'Text', sprintf('原始路径: %.2f 单位', rd.rawLen), ...
            'Position', [185, p - 47, 170, 20]);
        if simParams.enableSimplify
            uilabel(rPanel, 'Text', sprintf('简化后: %.2f 单位', rd.simpleLen), ...
                'Position', [10, p - 69, 170, 20]);
        end
        if simParams.enableSmooth
            uilabel(rPanel, 'Text', sprintf('平滑后: %.2f 单位', rd.smoothLen), ...
                'Position', [185, p - 69, 170, 20]);
        end

        % 访问顺序
        visitOrder = rd.visitOrder;
        if ~isempty(visitOrder)
            orderStr = '';
            for i = 1:min(size(visitOrder, 1), 6)
                orderStr = [orderStr, sprintf('(%d,%d) ', visitOrder(i, 2), visitOrder(i, 1))];
            end
            if size(visitOrder, 1) > 6
                orderStr = [orderStr, '...'];
            end
            uilabel(rPanel, 'Text', ['访问顺序: ', orderStr], ...
                'Position', [10, p - 91, rightW - 20, 20]);
        end
    end
end

end

function s = boolToStr(b)
    if b, s = '是'; else, s = '否'; end
end
