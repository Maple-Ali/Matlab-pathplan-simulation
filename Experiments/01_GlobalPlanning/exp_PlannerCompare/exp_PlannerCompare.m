%% exp_PlannerCompare — 全局路径规划算法多地图对比实验
%  在选定预设地图上运行指定规划算法，统计耗时/路径长度/路径代价/扩展节点数
%  支持多次运行取平均（nRuns≥2 时输出标准差），并绘制路径地图，保存实验数据（.mat + .txt 日志）
%
%  算法选择（参数使用各算法内部默认值）:
%    plannerNames = {'AStar_v1_1'}   → 仅 AStar_v1_1（障碍物距离自适应权重）
%    plannerNames = {'AStar_v1_2'}   → 仅 AStar_v1_2（按需局部障碍物距离计算）
%    plannerNames = {'AStar_v1'}     → 仅 AStar_v1
%    plannerNames = {'AStar'}        → 仅 AStar（原始版）
%    plannerNames = {'AStar_v0'}     → 仅 AStar_v0
%    plannerNames = {'AStar_v4'}     → 仅 AStar_v4（方向优先双层搜索）
%    plannerNames = {'RRT'}          → 仅 RRT（快速随机扩展树）
%    plannerNames = {'Dijkstra'}     → 仅 Dijkstra
%    plannerNames = {'AStar', 'AStar_v0', 'AStar_v1', 'AStar_v1_1', 'AStar_v1_2', 'AStar_v4', 'Dijkstra', 'RRT'} → 八者对比
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
% plannerNames = {'AStar_v0', 'AStar_v1', 'AStar_v1_1', 'AStar_v1_2', 'Dijkstra'};   % 可选: 'AStar_v1_1' / 'AStar_v1' / 'AStar_v0' / 'Dijkstra'
% plannerNames = {'AStar_v0', 'Dijkstra','RRT'};
plannerNames = {'AStar_v3_1'};
% plannerNames = {'AStar_v0'};
% plannerNames = {'AStar'};
% plannerNames = {'AStar_v1_1'};
% plannerNames = {'AStar_v1_2'};
% plannerNames = {'AStar_v1'};
% plannerNames = {'AStar_v4'};
% plannerNames = {'Dijkstra'};
% plannerNames = {'RRT'};

showExplored = true;   % true=地图上显示扩展节点(灰色)与open集(黄色)；false=仅显示路径

enableSimplify = false;   % true=启用 SimplifyPath 拐角裁剪
enableSmooth   = false;   % true=启用 SmoothPath 样条平滑（需先 SimplifyPath）
%  两者均启用时: 原始路径 → SimplifyPath → SmoothPath

nRuns = 10;   % 每组（算法×地图）重复运行次数，≥2 时输出标准差

% 地图配置: {地图名, startGrid, goalGrid}
%   startGrid/goalGrid 留空 [] → 使用地图自带；若地图无自带 → 默认角点 [2,2] / [size-1,size-1]
mapConfigs = {
    'Map2_kong',  [], [];
    'Map1',       [], [];
    '迷宫x80_3',        [], [];
    % '杂乱不规则',        [], [];
    % '迷宫x80',        [], [];
    % '迷宫x80_2',        [], [];
    % '迷宫',        [], [];

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
    'algoElapsed', {}, 'elapsed', {}, 'elapsedStd', {}, 'tSimplify', {}, 'tSmooth', {}, ...
    'pathLength', {}, 'pathCost', {}, 'pathCostOrig', {}, ...
    'expandedNodes', {}, 'openMaxSize', {}, ...
    'nRuns', {}, ...
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

        % 多次运行收集数据
        runAlgoElapsed = zeros(nRuns, 1);
        runElapsed    = zeros(nRuns, 1);
        runPathLen    = zeros(nRuns, 1);
        runPathCost   = zeros(nRuns, 1);
        runOrigCost   = zeros(nRuns, 1);
        runExpanded   = zeros(nRuns, 1);
        runOpenMax    = zeros(nRuns, 1);
        runSimplify   = zeros(nRuns, 1);
        runSmooth     = zeros(nRuns, 1);
        lastPath      = [];
        lastProcPath  = [];
        lastInfo      = [];
        lastOrigCost  = 0;
        firstExpCells = zeros(0, 2);
        firstOpenCells = zeros(0, 2);

        for ri = 1:nRuns
            % 运行规划器
            [path, info] = runPlanner(plannerName, map, startGrid, goalGrid, showExplored && ri == 1);

            % 路径后处理（独立计时）
            origCost = info.pathCost;
            procPath = path;
            algoElapsed = info.elapsed;   % 纯算法耗时
            tSimplify = 0;
            tSmooth = 0;
            if enableSimplify && ~isempty(path)
                tSimpStart = tic;
                procPath = SimplifyPath(path, map.getOccupancyGrid(), map.mapSize);
                tSimplify = toc(tSimpStart) * 1000;
            end
            if enableSmooth && ~isempty(procPath)
                tSmthStart = tic;
                procPath = SmoothPath(procPath);
                tSmooth = toc(tSmthStart) * 1000;
            end
            totalElapsed = algoElapsed + tSimplify + tSmooth;

            % 统一转为连续坐标
            procPathXY = procPath;
            if ~isempty(procPath)
                if enableSmooth
                    procPathXY = [procPath(:,2), procPath(:,1)];
                else
                    procPathXY = procPath - 0.5;
                end
            end
            procCost = computePathCostProcessed(procPathXY);

            % 收集本轮数据
            runAlgoElapsed(ri) = algoElapsed;
            runElapsed(ri)  = totalElapsed;
            runPathLen(ri)  = info.pathLength;
            runPathCost(ri) = procCost;
            runOrigCost(ri) = origCost;
            runExpanded(ri) = info.expandedNodes;
            runOpenMax(ri)  = info.openMaxSize;
            runSimplify(ri) = tSimplify;
            runSmooth(ri)   = tSmooth;

            % 保留最后一轮的路径与可视化数据
            lastPath     = path;
            lastProcPath = procPathXY;
            lastInfo     = info;
            lastOrigCost = origCost;

            % 第一轮保存可视化数据（showExplored 仅在第一轮生效）
            if ri == 1
                firstExpCells  = info.expandedCells;
                firstOpenCells = info.openCells;
            end
        end

        % 计算统计量
        avgAlgoElapsed = mean(runAlgoElapsed);
        avgElapsed  = mean(runElapsed);
        stdElapsed  = std(runElapsed);
        avgPathLen  = round(mean(runPathLen));
        avgPathCost = mean(runPathCost);
        avgOrigCost = mean(runOrigCost);
        avgExpanded = round(runExpanded(1));   % 确定性算法，取第一轮即可
        avgOpenMax  = round(runOpenMax(1));     % 确定性算法，取第一轮即可
        avgSimplify = mean(runSimplify);
        avgSmooth   = mean(runSmooth);

        % 记录结果（path/expandedCells/openCells 取最后一轮）
        results(end+1) = struct('plannerName', plannerName, ...
            'mapName', mapName, 'mapSize', mapData.mapSize, ...
            'startGrid', startGrid, 'goalGrid', goalGrid, ...
            'algoElapsed', avgAlgoElapsed, ...
            'elapsed', avgElapsed, ...
            'elapsedStd', stdElapsed, ...
            'tSimplify', avgSimplify, ...
            'tSmooth', avgSmooth, ...
            'pathLength', avgPathLen, ...
            'pathCost', avgPathCost, ...
            'pathCostOrig', avgOrigCost, ...
            'expandedNodes', avgExpanded, ...
            'openMaxSize', avgOpenMax, ...
            'nRuns', nRuns, ...
            'expandedCells', firstExpCells, ...
            'openCells', firstOpenCells, ...
            'path', lastProcPath); %#ok<SAGROW>

        % 打印
        if isempty(lastPath)
            fprintf('  [%s | %s] 无路径!\n', plannerName, mapName);
        else
            if enableSimplify || enableSmooth
                parts = sprintf('算法=%.2fms', avgAlgoElapsed);
                if avgSimplify > 0
                    parts = [parts, sprintf('+Simplify=%.2fms', avgSimplify)]; %#ok<AGROW>
                end
                if avgSmooth > 0
                    parts = [parts, sprintf('+Smooth=%.2fms', avgSmooth)]; %#ok<AGROW>
                end
                if nRuns >= 2
                    fprintf('  [%s | %s] 耗时=%s=%.2f±%.2fms 路径长=%d 原始代价=%.2f 处理后代价=%.2f 扩展=%d (%d次平均)\n', ...
                        plannerName, mapName, parts, avgElapsed, stdElapsed, ...
                        avgPathLen, avgOrigCost, avgPathCost, avgExpanded, nRuns);
                else
                    fprintf('  [%s | %s] 耗时=%s=%.2fms 路径长=%d 原始代价=%.2f 处理后代价=%.2f 扩展=%d\n', ...
                        plannerName, mapName, parts, avgElapsed, ...
                        avgPathLen, avgOrigCost, avgPathCost, avgExpanded);
                end
            else
                if nRuns >= 2
                    fprintf('  [%s | %s] 耗时=%.2f±%.2fms 路径长=%d 代价=%.2f 扩展=%d (%d次平均)\n', ...
                        plannerName, mapName, avgElapsed, stdElapsed, ...
                        avgPathLen, avgPathCost, avgExpanded, nRuns);
                else
                    fprintf('  [%s | %s] 耗时=%.2fms 路径长=%d 代价=%.2f 扩展=%d\n', ...
                        plannerName, mapName, avgElapsed, avgPathLen, ...
                        avgPathCost, avgExpanded);
                end
            end
        end
    end
end
totalElapsed = toc(tTotal) * 1000;   % ms
fprintf('\n全部完成，总耗时: %.2f ms\n', totalElapsed);

%% ======================== 汇总表 ========================
fprintf('\n=== 结果汇总 (nRuns=%d) ===\n', nRuns);
if enableSimplify || enableSmooth
    fprintf('%-12s %-12s %10s %8s %8s %10s %10s %10s %5s\n', '算法', '地图', '总耗时(ms)', '算法', 'Simplify', 'Smooth', '原始代价', '处理后代价', '次数');
    fprintf('%s\n', repmat('-', 1, 90));
    for i = 1:length(results)
        r = results(i);
        if isempty(r.path)
            fprintf('%-12s %-12s %10s %8s %8s %10s %10s %10s %5d\n', ...
                r.plannerName, r.mapName, '-', '-', '-', '-', '-', '-', r.nRuns);
        else
            fprintf('%-12s %-12s %10.2f %8.2f %8.2f %10.2f %10.2f %10.2f %5d\n', ...
                r.plannerName, r.mapName, r.elapsed, r.algoElapsed, r.tSimplify, r.tSmooth, ...
                r.pathCostOrig, r.pathCost, r.nRuns);
        end
    end
else
    fprintf('%-12s %-12s %14s %10s %10s %12s %6s\n', '算法', '地图', '耗时(ms)', '路径长度', '路径代价', '扩展节点数', '次数');
    fprintf('%s\n', repmat('-', 1, 78));
    for i = 1:length(results)
        r = results(i);
        if isempty(r.path)
            fprintf('%-12s %-12s %14s %10s %10s %12s %6d\n', ...
                r.plannerName, r.mapName, '-', '-', '-', '-', r.nRuns);
        else
            if r.nRuns >= 2
                fprintf('%-12s %-12s %7.2f±%-5.2f %10d %10.2f %12d %6d\n', ...
                    r.plannerName, r.mapName, r.elapsed, r.elapsedStd, r.pathLength, ...
                    r.pathCost, r.expandedNodes, r.nRuns);
            else
                fprintf('%-12s %-12s %14.2f %10d %10.2f %12d %6d\n', ...
                    r.plannerName, r.mapName, r.elapsed, r.pathLength, ...
                    r.pathCost, r.expandedNodes, r.nRuns);
            end
        end
    end
end

%% ======================== 保存数据（统一固定名称，自动覆盖旧数据） ========================
ts = datestr(now, 'yyyy-mm-dd HH:MM:SS');
saveDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

matFile = fullfile(saveDir, 'PlannerCompare_results.mat');
save(matFile, 'results', 'plannerNames', 'mapConfigs', 'showExplored', 'enableSimplify', 'enableSmooth', 'nRuns', 'totalElapsed');

% 写日志
logFile = fullfile(saveDir, 'PlannerCompare_results_log.txt');
fid = fopen(logFile, 'w');
fprintf(fid, '=== 全局路径规划算法对比实验 ===\n');
fprintf(fid, '时间: %s\n', ts);
fprintf(fid, '算法: %s\n', strjoin(plannerNames, ' / '));
fprintf(fid, '地图: %s\n', strjoin(mapConfigs(:,1), ' / '));
fprintf(fid, '显示扩展节点: %d\n', showExplored);
fprintf(fid, 'SimplifyPath: %s\n', mat2str(enableSimplify));
fprintf(fid, 'SmoothPath: %s\n', mat2str(enableSmooth));
fprintf(fid, '运行次数: %d\n', nRuns);
fprintf(fid, '总耗时: %.2f ms\n\n', totalElapsed);

fprintf(fid, '--- 结果汇总 ---\n');
fprintf(fid, '%-12s %-12s %14s %10s %10s %10s %12s %10s %6s %10s\n', ...
    '算法', '地图', '耗时(ms)', '路径长度', '原始代价', '处理后代价', '扩展节点数', '最大Open', '次数', '起终点');
fprintf(fid, '%s\n', repmat('-', 1, 120));
for i = 1:length(results)
    r = results(i);
    if r.nRuns >= 2
        fprintf(fid, '%-12s %-12s %7.2f±%-5.2f %10d %10.2f %10.2f %12d %10s %6d %s→%s\n', ...
            r.plannerName, r.mapName, r.elapsed, r.elapsedStd, r.pathLength, r.pathCostOrig, r.pathCost, ...
            r.expandedNodes, mat2str(r.openMaxSize), r.nRuns, mat2str(r.startGrid), mat2str(r.goalGrid));
    else
        fprintf(fid, '%-12s %-12s %14.2f %10d %10.2f %10.2f %12d %10s %6d %s→%s\n', ...
            r.plannerName, r.mapName, r.elapsed, r.pathLength, r.pathCostOrig, r.pathCost, ...
            r.expandedNodes, mat2str(r.openMaxSize), r.nRuns, mat2str(r.startGrid), mat2str(r.goalGrid));
    end
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
        r.plannerName, r.mapName, r.pathCostOrig, r.expandedNodes, r.elapsed);
    plotPathMap(mapData, r.path, r.startGrid, r.goalGrid, ...
        r.expandedCells, r.openCells, showExplored, figTitle);
end

fprintf('\n=== 实验完成 ===\n');

%% ======================== 局部函数 ========================

function [path, info] = runPlanner(plannerName, map, startGrid, goalGrid, showExplored)
%RUNPLANNER 统一运行规划器并统计指标
%   算法内部参数使用各自默认值，不在外部指定
%   耗时统计仅包含算法核心运行时间，不含 Callback 开销
%   showExplored=true 时通过 callback 收集扩展节点(expandedCells)与 open 集(openCells)（不计入耗时）
    n = map.mapSize;

    % ========== 仅统计算法核心运行时间（不含 Callback）==========
    switch plannerName
        case 'AStar_v1_1'
            tStart = tic;
            [path, info] = AStar_v1_1(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;

        case 'AStar_v4'
            [path, info] = AStar_v4(map, startGrid, goalGrid, 0, []);
            % AStar_v4 的 info.elapsed 已是 ms，info.expanded → expandedNodes
            info.pathLength    = size(path, 1);
            info.pathCost      = info.cost;
            info.expandedNodes = info.expanded;
            info.openMaxSize   = NaN;

        case 'AStar_v1_2'
            tStart = tic;
            [path, info] = AStar_v1_2(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;

        case 'AStar_v1'
            tStart = tic;
            [path, info] = AStar_v1(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;

        case 'AStar'
            tStart = tic;
            path = AStar(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;
            info = struct('elapsed', info.elapsed, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', computePathCost(path), ...
                          'openMaxSize', NaN);

        case 'AStar_v0'
            tStart = tic;
            path = AStar_v0(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;
            info = struct('elapsed', info.elapsed, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', computePathCost(path), ...
                          'openMaxSize', NaN);

        case 'AStar_v3_1'
            tStart = tic;
            path = AStar_v3_1(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;
            info = struct('elapsed', info.elapsed, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', computePathCost(path), ...
                          'openMaxSize', NaN);

        case 'Dijkstra'
            tStart = tic;
            path = Dijkstra(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;
            info = struct('elapsed', info.elapsed, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', computePathCost(path), ...
                          'openMaxSize', NaN);

        case 'RRT'
            tStart = tic;
            path = RRT(map, startGrid, goalGrid, 0, []);
            info.elapsed = toc(tStart) * 1000;
            info = struct('elapsed', info.elapsed, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', computePathCost(path), ...
                          'openMaxSize', NaN);

        otherwise
            error('未知规划器: %s', plannerName);
    end

    % ========== 可视化数据收集（不计入耗时）==========
    expandedList  = zeros(0, 2);
    openHistMat   = false(n, n);
    expandedCount = 0;

    if showExplored
        % 使用 Callback 收集扩展节点和 open 集信息
        % AStar_v4 的 callback 接口不同，跳过 open 集收集
        cb = @onStep;
        switch plannerName
            case 'AStar_v4'
                % AStar_v4 callback(pathSoFar, count)，从 pathSoFar 末尾提取扩展节点
                expandedV4 = zeros(0, 2);
                AStar_v4(map, startGrid, goalGrid, 0, @(psf, ~) collectV4(psf));
                info.expandedCells = expandedV4;
                info.openCells     = zeros(0, 2);
            case 'AStar_v1_1'
                AStar_v1_1(map, startGrid, goalGrid, 0, cb);
            case 'AStar_v1_2'
                AStar_v1_2(map, startGrid, goalGrid, 0, cb);
            case 'AStar_v1'
                AStar_v1(map, startGrid, goalGrid, 0, cb);
            case 'AStar'
                AStar(map, startGrid, goalGrid, 0, cb);
            case 'AStar_v0'
                AStar_v0(map, startGrid, goalGrid, 0, cb);
            case 'AStar_v3_1'
                AStar_v3_1(map, startGrid, goalGrid, 0, cb);
            case 'Dijkstra'
                Dijkstra(map, startGrid, goalGrid, 0, cb);
            case 'RRT'
                % RRT 无需收集扩展节点
        end
        % AStar_v4 已在耗时阶段设置 expandedNodes，不覆盖
        if ~strcmp(plannerName, 'AStar_v4')
            info.expandedNodes = expandedCount;
            info.expandedCells = expandedList;
        end
        [orow, ocol] = find(openHistMat);
        info.openCells = [orow, ocol];
    else
        if ~isfield(info, 'expandedNodes')
            if isfield(info, 'expanded')
                info.expandedNodes = info.expanded;
            else
                info.expandedNodes = 0;
            end
        end
        info.expandedCells = zeros(0, 2);
        info.openCells     = zeros(0, 2);
    end

    function action = onStep(stateInfo)
        if ~isfield(stateInfo, 'type') || strcmp(stateInfo.type, 'step')
            expandedCount = expandedCount + 1;
            expandedList(end+1, :) = stateInfo.current; %#ok<AGROW>
            if isfield(stateInfo, 'openSet') && ~isempty(stateInfo.openSet)
                openHistMat = openHistMat | stateInfo.openSet;
            end
        end
        action = 'continue';
    end

    function collectV4(pathSoFar)
        if ~isempty(pathSoFar)
            expandedV4(end+1, :) = pathSoFar(end, :); %#ok<AGROW>
        end
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

function c = computePathCostProcessed(path)
%COMPUTEPATHCOSTPROCESSED 计算处理后路径代价
%   栅格路径 [row,col]: 8 邻域代价（1/sqrt(2)）
%   连续路径 [x,y]: 欧氏距离累加
    if isempty(path) || size(path, 1) < 2
        c = inf;
        return;
    end
    d = diff(path, 1, 1);
    dists = sqrt(sum(d.^2, 2));
    c = sum(dists);
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
                    'FaceColor', [1, 1, 0.4], 'EdgeColor', 'none');
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

    % --- 规划路径（已为连续坐标 [x, y]） ---
    if ~isempty(path)
        plot(path(:,1), path(:,2), '-', 'Color', [0.0, 0.3, 1.0], 'LineWidth', 2.0);
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
