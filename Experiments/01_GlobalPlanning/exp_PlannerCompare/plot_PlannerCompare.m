%% plot_PlannerCompare — 全局路径规划算法对比绘图
%  参考 exp_PlannerCompare.m 生成的数据（results/*.mat 中的 results 结构体）绘制对比图
%  支持合并多个结果文件（不同批次/算法组），按 算法×地图 对比
%
%  绘图内容:
%    Figure 1: 各地图路径对比（紧凑排列，多算法路径叠加 + 起终点，底部单个横向图例）
%    Figure 2: 规划耗时对比（分组柱状图 + 标准差误差棒，log Y）
%    Figure 3: 路径代价对比（分组柱状图）
%    Figure 4: 路径长度对比（分组柱状图）
%    Figure 5: 扩展节点数对比（分组柱状图）
%
%  字号约定: 坐标轴/刻度/标题 12pt，图内数据标注 10pt 加粗黑色
%  图片仅显示，不自动保存

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

% ===== Config: 结果文件列表（results/ 目录下的 .mat） =====
%  每个文件为 exp_PlannerCompare.m 保存的结果，可同时加载多个进行合并对比
%  同一 (算法, 地图) 出现多次时保留先加载的记录
resultFiles = {
    'PlannerCompare_results.mat';   % AStar_v3_1
    % 'C2_2.mat';                     % AStar_v0 / Dijkstra / RRT
    % 'C2_1.mat';                   % AStar_v3_1 + Simplify/Smooth（与上面重复时保留先前的）
};

% 算法显示名（可选；未列出的算法用原名）
plannerDisplay = {
    'AStar_v3_1', 'Improved A*';
    'AStar_v0',   'A*';
    'Dijkstra',   'Dijkstra';
    'RRT',        'RRT';
    'AStar',      'AStar';
    'AStar_v1',   'AStar_v1';
    'AStar_v1_1', 'AStar_v1_1';
    'AStar_v1_2', 'AStar_v1_2';
    'AStar_v4',   'AStar_v4';
};

% 算法配色（可选；未列出的算法用 lines 自动分配）
plannerColors = {
    'AStar_v3_1', [1.00, 0.00, 0.00];   % Red
    'AStar_v0',   [0.00, 0.00, 1.0];    % Blue
    'Dijkstra',   [0.00, 1.00, 0.00];   % Green
    'RRT',        [0.00, 0.80, 1.00];   % blue
    'AStar',      [0.30, 0.75, 0.93];   % cyan
    'AStar_v1',   [0.64, 0.08, 0.18];   % maroon
    'AStar_v1_1', [0.93, 0.69, 0.13];   % yellow
    'AStar_v1_2', [0.62, 0.75, 0.45];   % light green
    'AStar_v4',   [0.55, 0.27, 0.07];   % brown
};

timeField = 'elapsed';         % 'elapsed'=总耗时(含后处理) | 'algoElapsed'=纯算法耗时
costField = 'pathCostOrig';    % 'pathCostOrig'=原始路径代价 | 'pathCost'=处理后代价
% ========================================================================

%% ===== 加载并合并数据 =====
allResults = [];
for fi = 1:numel(resultFiles)
    fname = fullfile(resultsDir, resultFiles{fi});
    if ~exist(fname, 'file')
        warning('Missing: %s', fname);
        continue;
    end
    S = load(fname);
    if ~isfield(S, 'results') || isempty(S.results)
        warning('文件中无 results 数据: %s', fname);
        continue;
    end
    if ~isempty(allResults) && ~isempty(setxor(fieldnames(allResults), fieldnames(S.results)))
        error('结果文件字段不一致，无法合并: %s', fname);
    end
    if isempty(allResults)
        allResults = S.results(:);
    else
        allResults = [allResults; S.results(:)]; %#ok<AGROW>
    end
    fprintf('Loaded %-28s  %d 条记录\n', resultFiles{fi}, numel(S.results));
end
if isempty(allResults)
    error('未加载到任何结果数据，请检查 results 目录与 resultFiles 配置');
end

% 同一 (算法, 地图) 去重：保留先加载的记录
keep = true(numel(allResults), 1);
seen = containers.Map('KeyType', 'char', 'ValueType', 'logical');
for i = 1:numel(allResults)
    key = [allResults(i).plannerName '|' allResults(i).mapName];
    if isKey(seen, key)
        warning('重复记录 %s + %s，保留先前加载的', allResults(i).plannerName, allResults(i).mapName);
        keep(i) = false;
    else
        seen(key) = true;
    end
end
allResults = allResults(keep);

% 提取算法 / 地图列表（按首次出现顺序）
plannerNames = unique({allResults.plannerName}, 'stable');
mapNames     = unique({allResults.mapName}, 'stable');
nPlanner = numel(plannerNames);
nMap     = numel(mapNames);

% 显示名与配色
displayNames = plannerNames;
colors = lines(nPlanner);
for ai = 1:nPlanner
    row = find(strcmp(plannerDisplay(:,1), plannerNames{ai}), 1);
    if ~isempty(row), displayNames{ai} = plannerDisplay{row, 2}; end
    row = find(strcmp(plannerColors(:,1), plannerNames{ai}), 1);
    if ~isempty(row), colors(ai,:) = plannerColors{row, 2}; end
end

% 记录索引矩阵 (算法 × 地图)
recIdx = nan(nPlanner, nMap);
for i = 1:numel(allResults)
    ai = find(strcmp(plannerNames, allResults(i).plannerName), 1);
    mi = find(strcmp(mapNames,     allResults(i).mapName),     1);
    recIdx(ai, mi) = i;
end

% 指标矩阵 (地图 × 算法，bar 直接可用)
timeM     = nan(nMap, nPlanner);
timeStdM  = nan(nMap, nPlanner);
costM     = nan(nMap, nPlanner);
lenM      = nan(nMap, nPlanner);
nodeM     = nan(nMap, nPlanner);
for ai = 1:nPlanner
    for mi = 1:nMap
        if isnan(recIdx(ai, mi)), continue; end
        r = allResults(recIdx(ai, mi));
        if isfield(r, timeField), timeM(mi, ai)    = r.(timeField);    end
        if isfield(r, 'elapsedStd'), timeStdM(mi, ai) = r.elapsedStd;  end
        if isfield(r, costField), costM(mi, ai)    = r.(costField);    end
        if isfield(r, 'pathLength'), lenM(mi, ai)  = r.pathLength;     end
        if isfield(r, 'expandedNodes'), nodeM(mi, ai) = r.expandedNodes; end
    end
end

% 控制台汇总
fprintf('\n=== 结果汇总 (共 %d 条记录, %d 算法 × %d 地图) ===\n', ...
    numel(allResults), nPlanner, nMap);
fprintf('%-12s %-14s %12s %12s %10s %10s %5s\n', ...
    '算法', '地图', '耗时(ms)', '路径代价', '路径长度', '扩展节点', '次数');
fprintf('%s\n', repmat('-', 1, 80));
for i = 1:numel(allResults)
    r = allResults(i);
    fprintf('%-12s %-14s %7.2f±%-5.2f %12.2f %10.0f %10.0f %5d\n', ...
        r.plannerName, r.mapName, r.(timeField), r.elapsedStd, ...
        r.(costField), r.pathLength, r.expandedNodes, r.nRuns);
end

%% ===== Figure 1: 各地图路径对比 =====
figure('Position', [50, 50, 420*nMap, 520], 'Color', 'w');
tl = tiledlayout(1, nMap, 'TileSpacing', 'compact', 'Padding', 'compact');
legendHandles = gobjects(0);
legendNames = {};        % 全图共享图例：句柄 + 名称（每种算法仅取一次）
for mi = 1:nMap
    mapName = mapNames{mi};
    [~, mapData] = loadPresetMap(mapName);
    n = mapData.mapSize;

    nexttile(tl);
    hold on;

    % 障碍物（纯黑，x=row, y=col，与 exp_PlannerCompare 绘图约定一致）
    obs = mapData.staticObstacles;
    for oi = 1:size(obs, 1)
        rectangle('Position', [obs(oi,1)-1, obs(oi,2)-1, 1, 1], ...
            'FaceColor', [0, 0, 0], 'EdgeColor', 'none');
    end

    % 各算法路径
    startGrid = []; goalGrid = [];
    for ai = 1:nPlanner
        if isnan(recIdx(ai, mi)), continue; end
        r = allResults(recIdx(ai, mi));
        if isempty(startGrid), startGrid = r.startGrid; goalGrid = r.goalGrid; end
        if isempty(r.path), continue; end
        hp = plot(r.path(:,1), r.path(:,2), '-', ...
            'Color', colors(ai,:), 'LineWidth', 1.2);
        % 全图共享图例：每种算法只保留第一次出现的句柄
        if ~any(strcmp(legendNames, displayNames{ai}))
            legendHandles(end+1) = hp; %#ok<AGROW>
            legendNames{end+1} = displayNames{ai}; %#ok<AGROW>
        end
    end

    % 起点（圆形）/ 终点（三角形）
    if ~isempty(startGrid)
        sx = startGrid(1) - 0.5;  sy = startGrid(2) - 0.5;
        gx = goalGrid(1) - 0.5;   gy = goalGrid(2) - 0.5;
        plot(sx, sy, 'o', 'MarkerSize', 8, ...
            'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
        plot(gx, gy, '^', 'MarkerSize', 8, ...
            'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
        text(sx, sy - 1, 'Start', 'HorizontalAlignment', 'center', ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
        text(gx, gy + 1, 'Goal', 'HorizontalAlignment', 'center', ...
            'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);
    end

    axis equal; axis([0, n, 0, n]); grid on;
    set(gca, 'GridAlpha', 0.2, 'FontSize', 16);
    % xlabel('X (行)', 'FontSize', 12);
    % ylabel('Y (列)', 'FontSize', 12);
    title(mapName, 'FontSize', 12);
    hold off;
end

% 全图共用一个图例（横向排列，置于底部）
if ~isempty(legendHandles)
    lgd = legend(legendHandles, legendNames, 'Orientation', 'horizontal', ...
        'FontSize', 12, 'Box', 'on');
    lgd.Layout.Tile = 'south';
end
% sgtitle('Path Comparison per Map', 'FontSize', 12);

%% ===== Figure 2-5: 指标对比柱状图 =====
plotMetricBars(timeM, timeStdM, mapNames, displayNames, colors, ...
    '%.2f', 'Planning Time (ms)', 'Planning Time Comparison (log)', true);
plotMetricBars(costM, [], mapNames, displayNames, colors, ...
    '%.2f', 'Path Cost', 'Path Cost Comparison', false);
plotMetricBars(lenM, [], mapNames, displayNames, colors, ...
    '%.0f', 'Path Length (points)', 'Path Length Comparison', false);
plotMetricBars(nodeM, [], mapNames, displayNames, colors, ...
    '%.0f', 'Expanded Nodes', 'Expanded Nodes Comparison', false);

fprintf('\nAll figures ready. (Not auto-saved)\n');

%% ======================== 局部函数 ========================
function plotMetricBars(Y, errY, mapNames, seriesNames, colors, labelFmt, yLabelText, titleText, useLog)
%PLOTMETRICBARS 分组柱状图：X=地图，每组内每算法一根柱（组内柱体紧贴），可选误差棒
%   Y: nMap × nPlanner；errY: 同尺寸误差棒（空数组表示不绘制）
%   useLog: true 时 Y 轴取 log 标尺（仅当全部数据为正时生效）
    figure('Position', [60, 60, 900, 520], 'Color', 'w');
    hold on;
    nMap = numel(mapNames);
    nSeries = size(Y, 2);

    % 自动回退：log 标尺要求数据全为正
    if useLog && ~all(Y(~isnan(Y)) > 0)
        useLog = false;
    end

    b = bar(1:nMap, Y, 'grouped', 'BarWidth', 0.8, ...
        'EdgeColor', 'k', 'LineWidth', 0.5);
    for si = 1:nSeries
        b(si).FaceColor = 'flat';
        b(si).CData = repmat(colors(si,:), nMap, 1);
        if ~isempty(errY)
            errorbar(b(si).XEndPoints, Y(:,si), errY(:,si), ...
                'LineStyle', 'none', 'Color', [0.2, 0.2, 0.2], ...
                'LineWidth', 1.0, 'HandleVisibility', 'off');
        end
    end

    % 图内数据标注（10pt 加粗黑色）
    yMax = max(Y(:), [], 'omitnan');
    for si = 1:nSeries
        for mi = 1:nMap
            if isnan(Y(mi, si)), continue; end
            if useLog
                ytxt = Y(mi, si) * 1.08;           % log 轴用乘性偏移
            else
                ytxt = Y(mi, si) + yMax * 0.015;   % 线性轴用加性偏移
            end
            text(b(si).XEndPoints(mi), ytxt, sprintf(labelFmt, Y(mi, si)), ...
                'HorizontalAlignment', 'center', 'FontSize', 10, ...
                'FontWeight', 'bold', 'Color', 'k');
        end
    end

    if useLog, set(gca, 'YScale', 'log'); end
    set(gca, 'XTick', 1:nMap, 'XTickLabel', mapNames, ...
        'XTickLabelRotation', 15, 'FontSize', 12);
    ylabel(yLabelText, 'FontSize', 12);
    title(titleText, 'FontSize', 12);
    legend(seriesNames, 'Location', 'northwest', 'FontSize', 12);
    grid on;
    hold off;
end
