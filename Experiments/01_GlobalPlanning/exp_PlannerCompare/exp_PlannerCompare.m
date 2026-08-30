%% exp_PlannerCompare — 全局路径规划算法（AStar_v1_1 / Dijkstra）多地图对比实验
%  在选定预设地图上运行指定规划算法，统计耗时/路径长度/路径代价/扩展节点数
%  并绘制路径地图，保存实验数据（.mat + .txt 日志）
%
%  算法选择（参数使用各算法内部默认值）:
%    plannerNames = {'AStar_v1_1'}   → 仅 AStar_v1_1
%    plannerNames = {'AStar_v1'}     → 仅 AStar_v1
%    plannerNames = {'AStar_v0'}     → 仅 AStar_v0
%    plannerNames = {'Dijkstra'}     → 仅 Dijkstra
%    plannerNames = {'AStar_v0', 'AStar_v1', 'AStar_v1_1', 'Dijkstra'} → 四者对比
%
%  预设地图: D:\Claude-File\shiyan1\PresetMaps\*.mat
%    可用地图（部分无自带起终点，可在下方手动指定）:
%    Map1, Map2_kong, Map_2, Map_3, Full_1_50, 迷宫, tspcss,
%    杂乱不规则, 杂乱不规则_临障, 杂乱不规则_动临, 杂乱不规则_动障,
%    简化路径测试, 房间地图2, 旋转对称, 旋转对称_1, 模拟房间,
%    25随机, DWA测试, Full_1, bs, dz, ls1, 十, 临时障碍物测试,
%    模拟房间（多目标分配）

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% ======================== 参数配置（在此修改） ========================
plannerNames = {'AStar_v1_1'};   % 可选: 'AStar_v1_1' / 'AStar_v1' / 'AStar_v0' / 'Dijkstra'
% plannerNames = {'AStar_v0'};
% plannerNames = {'AStar_v1'};
% plannerNames = {'Dijkstra'};

showExplored = true;   % true=地图上显示扩展节点(灰色)与open集(黄色)；false=仅显示路径

% 地图配置: {地图名, startGrid, goalGrid}
%   startGrid/goalGrid 留空 [] → 使用地图自带；若地图无自带 → 默认角点 [2,2] / [size-1,size-1]
mapConfigs = {
    % 'Map1',       [], [];
    'Map2_kong',  [], [];
    % '迷宫',        [], [];
    % '迷宫x80',        [], [];
};
% ========================================================================

nPlanner = length(plannerNames);
nMap     = size(mapConfigs, 1);
fprintf('=== 全局路径规划算法对比实验 ===\n');
fprintf('算法: %s\n', strjoin(plannerNames, ' / '));
fprintf('地图: %s\n\n', strjoin(mapConfigs(:,1), ' / '));

%% ======================== 运行实验 ========================
results = struct('plannerName', {}, 'mapName', {}, 'mapSize', {}, ...
    'startGrid', {}, 'goalGrid', {}, ...
    'elapsed', {}, 'pathLength', {}, 'pathCost', {}, ...
    'expandedNodes', {}, 'openMaxSize', {}, ...
    'expandedCells', {}, 'openCells', {}, 'path', {});

tTotal = tic;
for pi = 1:nPlanner
    plannerName = plannerNames{pi};
    fprintf('--- 算法: %s ---\n', plannerName);
    for mi = 1:nMap
        mapName = mapConfigs{mi, 1};

        [map, mapData] = loadPresetMap(mapName);

        % 解析起终点
        if isempty(mapConfigs{mi, 2})
            startGrid = mapData.startPoint;
        else
            startGrid = mapConfigs{mi, 2};
        end
        if isempty(mapConfigs{mi, 3})
            goalGrid = mapData.goalPoint;
        else
            goalGrid = mapConfigs{mi, 3};
        end
        if isempty(startGrid), startGrid = [2, 2]; end
        if isempty(goalGrid),  goalGrid = [mapData.mapSize - 1, mapData.mapSize - 1]; end

        % 起终点合法性检查
        occ = map.getOccupancyGrid();
        if occ(startGrid(1), startGrid(2)) || occ(goalGrid(1), goalGrid(2))
            warning('地图 %s 的起点/终点位于障碍物上，跳过该组\n', mapName);
            continue;
        end

        % 运行规划器
        [path, info] = runPlanner(plannerName, map, startGrid, goalGrid, showExplored);

        % 记录结果
        results(end+1) = struct('plannerName', plannerName, ...
            'mapName', mapName, 'mapSize', mapData.mapSize, ...
            'startGrid', startGrid, 'goalGrid', goalGrid, ...
            'elapsed', info.elapsed, ...
            'pathLength', info.pathLength, ...
            'pathCost', info.pathCost, ...
            'expandedNodes', info.expandedNodes, ...
            'openMaxSize', info.openMaxSize, ...
            'expandedCells', info.expandedCells, ...
            'openCells', info.openCells, ...
            'path', path); %#ok<SAGROW>

        % 打印
        if isempty(path)
            fprintf('  [%s | %s] 无路径!\n', plannerName, mapName);
        else
            fprintf('  [%s | %s] 耗时=%.2fms 路径长=%d 代价=%.2f 扩展=%d\n', ...
                plannerName, mapName, info.elapsed, info.pathLength, ...
                info.pathCost, info.expandedNodes);
        end
    end
end
totalElapsed = toc(tTotal) * 1000;   % ms
fprintf('\n全部完成，总耗时: %.2f ms\n', totalElapsed);

%% ======================== 汇总表 ========================
fprintf('\n=== 结果汇总 ===\n');
fprintf('%-12s %-12s %10s %10s %10s %12s\n', '算法', '地图', '耗时(ms)', '路径长度', '路径代价', '扩展节点数');
fprintf('%s\n', repmat('-', 1, 68));
for i = 1:length(results)
    r = results(i);
    if isempty(r.path)
        fprintf('%-12s %-12s %10s %10s %10s %12s\n', ...
            r.plannerName, r.mapName, '-', '-', '-', '-');
    else
        fprintf('%-12s %-12s %10.2f %10d %10.2f %12d\n', ...
            r.plannerName, r.mapName, r.elapsed, r.pathLength, ...
            r.pathCost, r.expandedNodes);
    end
end

%% ======================== 保存数据（统一固定名称，自动覆盖旧数据） ========================
ts = datestr(now, 'yyyy-mm-dd HH:MM:SS');
saveDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

matFile = fullfile(saveDir, 'PlannerCompare_results.mat');
save(matFile, 'results', 'plannerNames', 'mapConfigs', 'showExplored', 'totalElapsed');

% 写日志
logFile = fullfile(saveDir, 'PlannerCompare_results_log.txt');
fid = fopen(logFile, 'w');
fprintf(fid, '=== 全局路径规划算法对比实验 ===\n');
fprintf(fid, '时间: %s\n', ts);
fprintf(fid, '算法: %s\n', strjoin(plannerNames, ' / '));
fprintf(fid, '地图: %s\n', strjoin(mapConfigs(:,1), ' / '));
fprintf(fid, '显示扩展节点: %d\n', showExplored);
fprintf(fid, '总耗时: %.2f ms\n\n', totalElapsed);

fprintf(fid, '--- 结果汇总 ---\n');
fprintf(fid, '%-12s %-12s %10s %10s %10s %12s %10s %10s\n', ...
    '算法', '地图', '耗时(ms)', '路径长度', '路径代价', '扩展节点数', '最大Open', '起终点');
fprintf(fid, '%s\n', repmat('-', 1, 100));
for i = 1:length(results)
    r = results(i);
    fprintf(fid, '%-12s %-12s %10.2f %10d %10.2f %12d %10s %s→%s\n', ...
        r.plannerName, r.mapName, r.elapsed, r.pathLength, r.pathCost, ...
        r.expandedNodes, mat2str(r.openMaxSize), mat2str(r.startGrid), mat2str(r.goalGrid));
end
fclose(fid);

fprintf('\n数据已保存: %s\n', matFile);
fprintf('日志已保存: %s\n', logFile);

%% ======================== 绘制路径地图（仅显示，不自动保存） ========================
for i = 1:length(results)
    r = results(i);
    if isempty(r.path), continue; end
    % 重新加载地图以获取障碍物
    [~, mapData] = loadPresetMap(r.mapName);
    figTitle = sprintf('%s — %s  (代价=%.2f, 扩展=%d, 耗时=%.2fms)', ...
        r.plannerName, r.mapName, r.pathCost, r.expandedNodes, r.elapsed);
    plotPathMap(mapData, r.path, r.startGrid, r.goalGrid, ...
        r.expandedCells, r.openCells, showExplored, figTitle);
end

fprintf('\n=== 实验完成 ===\n');

%% ======================== 局部函数 ========================

function [path, info] = runPlanner(plannerName, map, startGrid, goalGrid, showExplored)
%RUNPLANNER 统一运行规划器并统计指标
%   算法内部参数使用各自默认值，不在外部指定
%   对 AStar_v0/v1/v1_1 直接取 info；对 Dijkstra 通过 callback 计数扩展节点数并计算路径代价
%   showExplored=true 时通过 callback 收集扩展节点(expandedCells)与 open 集(openCells)
    tStart = tic;
    n = map.mapSize;
    expandedList  = zeros(0, 2);   % 扩展节点列表 [row, col]
    openHistMat   = false(n, n);   % 曾进入 open 集的节点
    expandedCount = 0;

    % 需要收集扩展/open 节点时才启用 callback
    cb = [];
    if showExplored
        cb = @onStep;
    end

    switch plannerName
        case 'AStar_v1_1'
            [path, info] = AStar_v1_1(map, startGrid, goalGrid, 0, cb);

        case 'AStar_v1'
            [path, info] = AStar_v1(map, startGrid, goalGrid, 0, cb);

        case 'AStar_v0'
            path = AStar_v0(map, startGrid, goalGrid, 0, cb);
            info = struct();
            info.pathLength    = size(path, 1);
            info.pathCost      = computePathCost(path);
            info.openMaxSize   = NaN;   % AStar_v0 无堆统计

        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0, cb);
            info = struct();
            info.pathLength    = size(path, 1);
            info.pathCost      = computePathCost(path);
            info.openMaxSize   = NaN;   % Dijkstra 使用线性扫描，无堆

        otherwise
            error('未知规划器: %s', plannerName);
    end

    % 汇总统计信息
    if showExplored
        info.expandedNodes = expandedCount;
        info.expandedCells = expandedList;
        [orow, ocol] = find(openHistMat);
        info.openCells = [orow, ocol];
    else
        if ~isfield(info, 'expandedNodes'), info.expandedNodes = 0; end
        info.expandedCells = zeros(0, 2);
        info.openCells     = zeros(0, 2);
    end
    info.elapsed = toc(tStart) * 1000;   % 单位: ms

    function action = onStep(stateInfo)
        % 仅统计节点扩展（type='step'），finish 不计数
        if ~isfield(stateInfo, 'type') || strcmp(stateInfo.type, 'step')
            expandedCount = expandedCount + 1;
            expandedList(end+1, :) = stateInfo.current; %#ok<AGROW>
            if isfield(stateInfo, 'openSet') && ~isempty(stateInfo.openSet)
                openHistMat = openHistMat | stateInfo.openSet;
            end
        end
        action = 'continue';
    end
end

function c = computePathCost(path)
%COMPUTEPATHCOST 计算 8 邻域栅格路径代价（与算法 moveCost 一致: 1 / sqrt(2)）
    if isempty(path)
        c = inf;
        return;
    end
    c = 0;
    for k = 1:size(path, 1) - 1
        dr = path(k+1, 1) - path(k, 1);
        dc = path(k+1, 2) - path(k, 2);
        c = c + sqrt(dr^2 + dc^2);
    end
end

function plotPathMap(mapData, path, startGrid, goalGrid, expandedCells, openCells, showExplored, figTitle)
%PLOTPATHMAP 绘制栅格地图 + 搜索范围 + 规划路径 + 起终点
%   障碍物以纯黑色矩形占满整个栅格；坐标已交换 x=row, y=col
%   open 集 → 黄色；扩展节点 → 灰色（showExplored=true 时绘制）
    figure('Position', [80, 80, 820, 700], 'Color', 'w');
    hold on;

    n = mapData.mapSize;

    % --- 搜索范围（先画底层: open 黄色 → 扩展灰色） ---
    if showExplored
        if ~isempty(openCells)
            for oi = 1:size(openCells, 1)
                rectangle('Position', [openCells(oi,1)-1, openCells(oi,2)-1, 1, 1], ...
                    'FaceColor', [1, 1, 0.8], 'EdgeColor', 'none');
            end
        end
        if ~isempty(expandedCells)
            for ei = 1:size(expandedCells, 1)
                rectangle('Position', [expandedCells(ei,1)-1, expandedCells(ei,2)-1, 1, 1], ...
                    'FaceColor', [0.9, 0.9, 0.9], 'EdgeColor', 'none');
            end
        end
    end

    % --- 障碍物: 纯黑色矩形，占满整个栅格 ---
    obs = mapData.staticObstacles;   % [row, col]
    for oi = 1:size(obs, 1)
        rectangle('Position', [obs(oi,1)-1, obs(oi,2)-1, 1, 1], ...
            'FaceColor', [0, 0, 0], 'EdgeColor', 'none');
    end

    % --- 规划路径 ---
    if ~isempty(path)
        px = path(:, 1) - 0.5;
        py = path(:, 2) - 0.5;
        plot(px, py, '-', 'Color', [0.0, 0.3, 1.0], 'LineWidth', 2.0);
    end

    % --- 起点（圆形）/ 终点（三角形） ---
    sx = startGrid(1) - 0.5;  sy = startGrid(2) - 0.5;
    gx = goalGrid(1) - 0.5;   gy = goalGrid(2) - 0.5;
    plot(sx, sy, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    text(sx, sy - 1, 'Start', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.2, 0.6, 0.2]);
    plot(gx, gy, '^', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    text(gx, gy + 1, 'Goal', 'HorizontalAlignment', 'center', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.2]);

    axis equal; axis([0, n, 0, n]); grid on;
    set(gca, 'GridAlpha', 0.2);
    xlabel('X (行)'); ylabel('Y (列)');
    title(figTitle, 'FontSize', 12);
    hold off;
end
