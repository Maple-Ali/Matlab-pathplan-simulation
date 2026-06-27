function DataViewerUI(results, simParams)
%DATAVIEWERUI 数据展示界面
%   DataViewerUI(results, simParams) 显示仿真统计数据（支持多机器人）
%   布局：上方"基本信息"表格，下方"机器人统计"表格

numRobots = 1;
if isfield(results, 'numRobots'), numRobots = results.numRobots; end

% 动态窗口尺寸
nDataCols = numRobots + 1;  % 机器人列 + 合计列
colW = 110;
rowNameW = 130;  % RowName 列宽
tableW = rowNameW + nDataCols * colW + 30;
figW = max(tableW + 40, 600);
figH = 600;

fig = uifigure('Name', '仿真数据报告', ...
    'Position', [200, max(50, 700 - figH), figW, figH], 'Resize', 'off');

% 标题
uilabel(fig, 'Text', '仿真数据报告', ...
    'Position', [(figW - 200) / 2, figH - 40, 200, 30], ...
    'FontSize', 16, 'FontWeight', 'bold', ...
    'HorizontalAlignment', 'center');

% ==================== 上方：基本信息表 ====================
infoPanelH = 200;
infoPanel = uipanel(fig, 'Title', '基本信息', ...
    'Position', [15, figH - infoPanelH - 45, figW - 30, infoPanelH]);

infoData = buildInfoTable(simParams, results, numRobots);
uitable(infoPanel, ...
    'Position', [5, 5, figW - 40, infoPanelH - 25], ...
    'Data', infoData, ...
    'ColumnName', {'参数', '值'}, ...
    'ColumnWidth', {140, figW - 190}, ...
    'RowName', [], ...
    'FontSize', 11, ...
    'ColumnEditable', [false false]);

% ==================== 下方：机器人统计表 ====================
statPanelY = 15;
statPanelH = figH - infoPanelH - 45 - 15 - 15;
statPanel = uipanel(fig, 'Title', '机器人统计', ...
    'Position', [15, statPanelY, figW - 30, statPanelH]);

[statData, rowNames, colNames, colWidths] = buildStatTable(results, simParams, numRobots);
uitable(statPanel, ...
    'Position', [5, 5, tableW, statPanelH - 25], ...
    'Data', statData, ...
    'ColumnName', colNames, ...
    'ColumnWidth', colWidths, ...
    'RowName', rowNames, ...
    'FontSize', 11, ...
    'ColumnEditable', false(1, nDataCols));

end

%% ==================== 辅助函数 ====================

function data = buildInfoTable(simParams, results, numRobots)
    rows = {};
    rows{end+1, 1} = '全局规划算法';   rows{end, 2} = simParams.globalAlgo;
    rows{end+1, 1} = '局部规划算法';   rows{end, 2} = simParams.localAlgo;
    if isfield(simParams, 'tspAlgo'),      rows{end+1, 1} = 'TSP 求解算法'; rows{end, 2} = simParams.tspAlgo; end
    if isfield(simParams, 'clusterAlgo'),  rows{end+1, 1} = '聚类算法';     rows{end, 2} = simParams.clusterAlgo; end
    rows{end+1, 1} = '拐角裁剪';       rows{end, 2} = boolToStr(simParams.enableSimplify);
    rows{end+1, 1} = '路径平滑';       rows{end, 2} = boolToStr(simParams.enableSmooth);
    if numRobots > 1
        rows{end+1, 1} = '机器人数量'; rows{end, 2} = sprintf('%d (协同)', numRobots);
    else
        rows{end+1, 1} = '地图大小';   rows{end, 2} = sprintf('%d x %d', simParams.mapSize, simParams.mapSize);
    end
    rows{end+1, 1} = '规划耗时';       rows{end, 2} = sprintf('%.1f 毫秒', results.planTime * 1000);
    rows{end+1, 1} = 'TSP 耗时';       rows{end, 2} = sprintf('%.1f 毫秒', results.tspTime * 1000);
    rows{end+1, 1} = '总仿真时间';     rows{end, 2} = sprintf('%.2f 秒', results.totalTime);
    if results.collision
        rows{end+1, 1} = '碰撞状态';   rows{end, 2} = '发生碰撞!';
    else
        rows{end+1, 1} = '碰撞状态';   rows{end, 2} = '安全';
    end
    if isfield(results, 'tspCost')
        rows{end+1, 1} = 'TSP 总成本'; rows{end, 2} = sprintf('%.2f', results.tspCost);
    end
    data = rows;
end

function [data, rowNames, colNames, colWidths] = buildStatTable(results, simParams, numRobots)
    colW = 110;
    nDataCols = numRobots + 1;

    % 列名：R1, R2, ..., 合计
    colNames = cell(1, nDataCols);
    for r = 1:numRobots
        colNames{r} = sprintf('R%d', r);
    end
    colNames{end} = '合计';
    colWidths = repmat({colW}, 1, nDataCols);

    % 指标行定义
    metrics = {
        '分配目标个数',   @(rd, ~) size(rd.assignedTargets, 1),  true,  '%d';
        '原始路径长度',   @(rd, ~) rd.rawLen,                     true,  '%.2f';
        '简化后路径长度', @(rd, ~) rd.simpleLen,                  true,  '%.2f';
        '平滑后路径长度', @(rd, ~) rd.smoothLen,                  true,  '%.2f';
        '行驶距离',       @(rd, ~) rd.totalDistance,              true,  '%.2f';
        '终点坐标',       @(rd, ~) formatCoord(rd.goalPoint),     false, '%s';
    };

    nMetrics = size(metrics, 1);
    rowNames = metrics(:, 1)';
    data = cell(nMetrics, nDataCols);

    for m = 1:nMetrics
        sumVal = 0;
        needSum = metrics{m, 3};
        fmt = metrics{m, 4};
        fn = metrics{m, 2};

        for r = 1:numRobots
            if isfield(results, 'robotDetails')
                rd = results.robotDetails(r);
            else
                rd = struct('assignedTargets', zeros(0,2), 'rawLen', 0, ...
                    'simpleLen', 0, 'smoothLen', 0, 'totalDistance', 0, ...
                    'goalPoint', [], 'visitOrder', []);
            end
            val = fn(rd, simParams);
            if isnumeric(val)
                data{m, r} = sprintf(fmt, val);
                if needSum, sumVal = sumVal + val; end
            else
                data{m, r} = val;
            end
        end

        % 合计列
        if needSum
            data{m, nDataCols} = sprintf(fmt, sumVal);
        else
            data{m, nDataCols} = '—';
        end
    end
end

function s = formatCoord(pt)
    if isempty(pt)
        s = '—';
    else
        s = sprintf('(%d,%d)', pt(2), pt(1));
    end
end

function s = boolToStr(b)
    if b, s = '是'; else, s = '否'; end
end
