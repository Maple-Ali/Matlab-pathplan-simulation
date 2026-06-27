function varargout = ClusterTesterUI(mainUIState)
%CLUSTERTESTERUI 聚类算法对比测试窗口
%   ClusterTesterUI(mainUIState)
%   fig = ClusterTesterUI(mainUIState)
%   独立窗口，用于测试和对比多个聚类算法的性能。
%   mainUIState: MainUI 的 state 结构体（用于导入地图），可为空。

% ===== 窗口创建 =====
fig = uifigure('Name', '聚类算法对比测试', ...
    'Position', [80, 80, 1200, 600], ...
    'CloseRequestFcn', @onWindowClose, ...
    'Resize', 'off');

% ===== 地图数据（算法坐标 [row, col]） =====
mapData = struct('mapSize', 30, ...
    'occGrid', [], ...
    'startGrids', [], ...    % N×2 [row, col] 机器人起点（即 medoids）
    'targetGrids', [], ...   % K×2 [row, col] 目标点
    'goalGrids', []);        % N×2 [row, col] 各机器人终点

% ===== 控制状态 =====
ctrl = struct('running', false, ...
    'lastResults', []);

% ===== 可视化句柄 =====
h = struct('robotPlots', [], ...
    'robotLabels', [], ...
    'targetPlots', [], ...
    'targetLabels', [], ...
    'clusterColors', [], ...
    'pathPlots', {{}});

% ===== 编辑模式 =====
setMode = 'none';  % 'none', 'robot', 'goal', 'target', 'obstacle', 'delete'

% ===== UI 布局 =====
% 状态栏
statusLabel = uilabel(fig, 'Text', '就绪', ...
    'Position', [20, 574, 1160, 20], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold');

% 左侧地图坐标轴
ax = uiaxes(fig, 'Position', [20, 140, 480, 420]);
plotTools('setupAxes', ax, mapData.mapSize);
title(ax, '聚类测试地图');
ax.ButtonDownFcn = @onMapClick;

% 地图下方：机器人数量 + 编辑模式按钮
uilabel(fig, 'Text', '机器人数:', 'Position', [20, 108, 65, 20]);
robotCountSpinner = uispinner(fig, ...
    'Limits', [1, 5], 'Value', 1, 'Step', 1, ...
    'Position', [88, 106, 55, 24], ...
    'ValueChangedFcn', @(~,~) onRobotCountChanged());

modeBtns = struct();
modeNames = {'robot', 'goal', 'target', 'obstacle', 'delete'};
modeLabels = {'设起点', '设终点', '加目标', '加障碍', '删对象'};
for k = 1:length(modeNames)
    modeBtns.(modeNames{k}) = uibutton(fig, 'Text', modeLabels{k}, ...
        'Position', [155 + (k-1)*72, 106, 68, 24], ...
        'ButtonPushedFcn', @(~,~) toggleMode(modeNames{k}));
end

% ===== 右侧控制面板 =====
ctrlPanel = uipanel(fig, 'Title', '控制', ...
    'Position', [520, 370, 660, 200]);

% 聚类算法多选列表
uilabel(ctrlPanel, 'Text', '聚类算法:', 'Position', [10, 148, 70, 20]);
clusterList = uilistbox(ctrlPanel, ...
    'Items', getClusterAlgoList(), ...
    'MultiSelect', 'on', ...
    'Value', {'KMedoids_Cluster'}, ...
    'Position', [10, 15, 130, 133]);

% 路径规划器下拉框
uilabel(ctrlPanel, 'Text', '路径规划器:', 'Position', [155, 148, 80, 20]);
plannerDD = uidropdown(ctrlPanel, ...
    'Items', {'AStar', 'AStar_v0', 'AStar_v1', 'AStar_v2', 'AStar_v3', 'Dijkstra', 'Dijkstra_v1', 'RRT'}, ...
    'Value', 'AStar', 'Position', [240, 146, 120, 22]);

% 地图大小
uilabel(ctrlPanel, 'Text', '地图大小:', 'Position', [155, 118, 70, 20]);
mapSizeDD = uidropdown(ctrlPanel, ...
    'Items', {'20x20', '30x30', '40x40', '50x50'}, ...
    'Value', '30x30', 'Position', [240, 116, 120, 22], ...
    'ValueChangedFcn', @(~,~) onMapSizeChanged());

% 随机障碍 / 导入地图 / 清空障碍
uibutton(ctrlPanel, 'Text', '随机障碍', ...
    'Position', [390, 146, 100, 25], ...
    'ButtonPushedFcn', @(~,~) randomObstacles());
uibutton(ctrlPanel, 'Text', '导入主UI地图', ...
    'Position', [390, 116, 100, 25], ...
    'ButtonPushedFcn', @(~,~) importMapFromMainUI());
uibutton(ctrlPanel, 'Text', '清空障碍', ...
    'Position', [510, 146, 100, 25], ...
    'ButtonPushedFcn', @(~,~) clearObstacles());

% 信息标签
infoLabel = uilabel(ctrlPanel, 'Text', '机器人: 0  目标: 0', ...
    'Position', [390, 90, 250, 20], 'FontColor', [0.2 0.2 0.8]);

% 开始测试 / 重置 / 退出
uibutton(ctrlPanel, 'Text', '开始测试', ...
    'Position', [10, 5, 180, 25], ...
    'ButtonPushedFcn', @(~,~) onRun());
uibutton(ctrlPanel, 'Text', '重置', ...
    'Position', [210, 5, 120, 25], ...
    'ButtonPushedFcn', @(~,~) onReset());
uibutton(ctrlPanel, 'Text', '退出', ...
    'Position', [350, 5, 120, 25], ...
    'ButtonPushedFcn', @(~,~) close(fig));

% ===== Tab 组（控制面板下方、地图右侧） =====
tabGroup = uitabgroup(fig, 'Position', [520, 10, 660, 355]);

% Tab 1: 聚类结果
tabCluster = uitab(tabGroup, 'Title', '聚类结果');
clusterAx = uiaxes(tabCluster, 'Position', [10, 5, 305, 315]);
title(clusterAx, '聚类分配'); xlabel(clusterAx, 'X'); ylabel(clusterAx, 'Y');
clusterBarAx = uiaxes(tabCluster, 'Position', [330, 5, 305, 315]);
title(clusterBarAx, '每机器人分配目标数');
ylabel(clusterBarAx, '数量');

% Tab 2: 算法对比
tabCompare = uitab(tabGroup, 'Title', '算法对比');
compareTable = uitable(tabCompare, 'Position', [10, 5, 625, 315], ...
    'ColumnName', {'算法', '聚类耗时(s)', '总分配成本', ...
    '最大簇', '最小簇', '簇大小标准差', '均衡系数'}, ...
    'ColumnEditable', false(1, 7), ...
    'ColumnWidth', {130, 90, 100, 65, 65, 100, 85});

% Tab 3: 负载均衡
tabBalance = uitab(tabGroup, 'Title', '负载均衡');
balanceAx = uiaxes(tabBalance, 'Position', [10, 5, 305, 315]);
title(balanceAx, '每机器人目标数与路径成本');
balanceInfoLabel = uilabel(tabBalance, 'Text', '', ...
    'Position', [330, 80, 305, 200], 'FontSize', 11);

% Tab 4: 耗时对比
tabTime = uitab(tabGroup, 'Title', '耗时对比');
timeAx = uiaxes(tabTime, 'Position', [10, 5, 625, 315]);
title(timeAx, '聚类算法耗时对比');
ylabel(timeAx, '耗时 (s)');

% ===== 初始化 =====
if nargin >= 1 && ~isempty(mainUIState)
    importMapFromMainUI(mainUIState);
else
    initEmptyMap(30);
end

% ====================================================================
%                       模式切换
% ====================================================================

    function toggleMode(mode)
        if strcmp(setMode, mode)
            setMode = 'none';
        else
            setMode = mode;
        end
        fnames = fieldnames(modeBtns);
        for k = 1:length(fnames)
            modeBtns.(fnames{k}).Value = false;
        end
        if ~strcmp(setMode, 'none')
            modeBtns.(setMode).Value = true;
        end
        setStatus(sprintf('编辑模式: %s', setMode));
    end

% ====================================================================
%                       地图交互
% ====================================================================

    function onMapClick(~, evt)
        if isprop(evt, 'IntersectionPoint') && ~isempty(evt.IntersectionPoint)
            x = evt.IntersectionPoint(1);
            y = evt.IntersectionPoint(2);
        else
            cp = ax.CurrentPoint;
            x = cp(1, 1); y = cp(1, 2);
        end
        col = ceil(x); row = ceil(y);
        if row < 1 || row > mapData.mapSize || col < 1 || col > mapData.mapSize
            return;
        end

        switch setMode
            case 'robot'
                mapData.startGrids(end + 1, :) = [row, col];
                setStatus(sprintf('机器人 %d 起点已添加: (%d,%d)', ...
                    size(mapData.startGrids, 1), row, col));
            case 'goal'
                mapData.goalGrids(end + 1, :) = [row, col];
                setStatus(sprintf('机器人 %d 终点已添加: (%d,%d)', ...
                    size(mapData.goalGrids, 1), row, col));
            case 'target'
                mapData.targetGrids(end + 1, :) = [row, col];
                setStatus(sprintf('目标点 %d 已添加: (%d,%d)', ...
                    size(mapData.targetGrids, 1), row, col));
            case 'obstacle'
                mapData.occGrid(row, col) = ~mapData.occGrid(row, col);
                setStatus(sprintf('障碍物切换: (%d,%d)', row, col));
            case 'delete'
                deleteObjectAt(row, col);
            otherwise
                return;
        end
        updateInfoLabel();
        drawStaticMap();
    end

% ====================================================================
%                       地图操作
% ====================================================================

    function initEmptyMap(sz)
        mapData.mapSize = sz;
        mapData.occGrid = false(sz, sz);
        mapData.startGrids = [];
        mapData.targetGrids = [];
        mapData.goalGrids = [];
    end

    function drawStaticMap()
        cla(ax);
        hold(ax, 'on');
        axis(ax, 'equal');
        axis(ax, [0, mapData.mapSize, 0, mapData.mapSize]);
        grid(ax, 'on');
        set(ax, 'XTick', 0:1:mapData.mapSize, 'YTick', 0:1:mapData.mapSize, ...
            'GridAlpha', 0.3);

        % 障碍物
        [obsRows, obsCols] = find(mapData.occGrid);
        if ~isempty(obsRows)
            obsColor = plotTools('getColor', 'staticObs');
            for k = 1:length(obsRows)
                rectangle(ax, 'Position', ...
                    [obsCols(k)-1, obsRows(k)-1, 1, 1], ...
                    'FaceColor', obsColor, 'EdgeColor', 'none');
            end
        end

        % 机器人起点
        for k = 1:size(mapData.startGrids, 1)
            ry = mapData.startGrids(k, 1) - 0.5;
            rx = mapData.startGrids(k, 2) - 0.5;
            clr = plotTools('multiRobotColor', k);
            plot(ax, rx, ry, 's', ...
                'MarkerSize', 10, 'LineWidth', 2, ...
                'MarkerEdgeColor', clr, 'MarkerFaceColor', clr);
            text(ax, rx + 0.3, ry + 0.3, sprintf('R%d', k), ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', clr);
        end

        % 终点
        for k = 1:size(mapData.goalGrids, 1)
            gy = mapData.goalGrids(k, 1) - 0.5;
            gx = mapData.goalGrids(k, 2) - 0.5;
            clr = plotTools('multiRobotColor', k);
            plot(ax, gx, gy, 'x', ...
                'MarkerSize', 10, 'LineWidth', 2, 'Color', clr);
            text(ax, gx + 0.3, gy + 0.3, sprintf('G%d', k), ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', clr);
        end

        % 目标点
        for k = 1:size(mapData.targetGrids, 1)
            tr = mapData.targetGrids(k, 1);
            tc = mapData.targetGrids(k, 2);
            ty = tr - 0.5; tx = tc - 0.5;
            plot(ax, tx, ty, 'o', ...
                'MarkerSize', 8, 'LineWidth', 2, ...
                'MarkerEdgeColor', plotTools('getColor', 'target'), ...
                'MarkerFaceColor', [0.5, 0.5, 1.0]);
            text(ax, tx + 0.3, ty + 0.3, sprintf('T%d', k), ...
                'FontSize', 8, 'FontWeight', 'bold');
        end

        % 使所有子图元不拦截点击
        set(ax.Children, 'PickableParts', 'none');

        % 清除路径覆盖
        clearPathPlots();
        drawnow;
    end

    function clearPathPlots()
        for k = 1:length(h.pathPlots)
            if ~isempty(h.pathPlots{k}) && isvalid(h.pathPlots{k})
                delete(h.pathPlots{k});
            end
        end
        h.pathPlots = {};
    end

    function deleteObjectAt(row, col)
        % 检查目标点
        if ~isempty(mapData.targetGrids)
            match = mapData.targetGrids(:, 1) == row & mapData.targetGrids(:, 2) == col;
            if any(match)
                mapData.targetGrids(match, :) = [];
                setStatus(sprintf('目标点 (%d,%d) 已删除', row, col));
                updateInfoLabel();
                return;
            end
        end
        % 检查终点
        if ~isempty(mapData.goalGrids)
            match = mapData.goalGrids(:, 1) == row & mapData.goalGrids(:, 2) == col;
            if any(match)
                mapData.goalGrids(match, :) = [];
                setStatus(sprintf('终点 (%d,%d) 已删除', row, col));
                updateInfoLabel();
                return;
            end
        end
        % 检查机器人起点
        if ~isempty(mapData.startGrids)
            match = mapData.startGrids(:, 1) == row & mapData.startGrids(:, 2) == col;
            if any(match)
                mapData.startGrids(match, :) = [];
                setStatus(sprintf('机器人起点 (%d,%d) 已删除', row, col));
                updateInfoLabel();
                return;
            end
        end
        % 检查障碍物
        if mapData.occGrid(row, col)
            mapData.occGrid(row, col) = false;
            setStatus(sprintf('障碍物 (%d,%d) 已删除', row, col));
            return;
        end
        setStatus('该位置无对象可删除');
    end

    function updateInfoLabel()
        nRobots = size(mapData.startGrids, 1);
        nTargets = size(mapData.targetGrids, 1);
        nGoals = size(mapData.goalGrids, 1);
        infoLabel.Text = sprintf('机器人: %d  目标: %d  终点: %d', nRobots, nTargets, nGoals);
    end

    function importMapFromMainUI(state)
        if nargin < 1
            state = mainUIState;
        end
        if isempty(state)
            setStatus('无可用地图数据');
            return;
        end

        mapData.mapSize = state.mapSize;

        % 障碍物：UI坐标 (col, row) → 算法坐标 [row, col]
        mapData.occGrid = false(mapData.mapSize, mapData.mapSize);
        if isfield(state, 'staticObstacles') && ~isempty(state.staticObstacles)
            for i = 1:size(state.staticObstacles, 1)
                r = state.staticObstacles(i, 2);
                c = state.staticObstacles(i, 1);
                if r >= 1 && r <= mapData.mapSize && c >= 1 && c <= mapData.mapSize
                    mapData.occGrid(r, c) = true;
                end
            end
        end

        % 机器人起点：UI坐标 (col, row) → 算法坐标 [row, col]
        mapData.startGrids = [];
        if isfield(state, 'startPoints') && ~isempty(state.startPoints)
            for i = 1:size(state.startPoints, 1)
                mapData.startGrids(i, :) = [state.startPoints(i, 2), state.startPoints(i, 1)];
            end
        elseif isfield(state, 'startPoint') && ~isempty(state.startPoint)
            mapData.startGrids = [state.startPoint(2), state.startPoint(1)];
        end

        % 目标点
        mapData.targetGrids = [];
        if isfield(state, 'targetPoints') && ~isempty(state.targetPoints)
            mapData.targetGrids = [state.targetPoints(:, 2), state.targetPoints(:, 1)];
        end

        % 终点：UI坐标 (col, row) → 算法坐标 [row, col]
        mapData.goalGrids = [];
        if isfield(state, 'goalPoints') && ~isempty(state.goalPoints)
            for i = 1:size(state.goalPoints, 1)
                mapData.goalGrids(i, :) = [state.goalPoints(i, 2), state.goalPoints(i, 1)];
            end
        elseif isfield(state, 'goalPoint') && ~isempty(state.goalPoint)
            mapData.goalGrids = [state.goalPoint(2), state.goalPoint(1)];
        end

        % 更新控件
        mapSizeDD.Value = sprintf('%dx%d', mapData.mapSize, mapData.mapSize);
        robotCountSpinner.Value = size(mapData.startGrids, 1);

        updateInfoLabel();
        drawStaticMap();
        setStatus(sprintf('已导入 %dx%d 地图, %d 个机器人, %d 个目标点, %d 个终点', ...
            mapData.mapSize, mapData.mapSize, ...
            size(mapData.startGrids, 1), size(mapData.targetGrids, 1), ...
            size(mapData.goalGrids, 1)));
    end

    function randomObstacles()
        mapData.occGrid = rand(mapData.mapSize, mapData.mapSize) < 0.25;
        for k = 1:size(mapData.startGrids, 1)
            mapData.occGrid(mapData.startGrids(k, 1), mapData.startGrids(k, 2)) = false;
        end
        for k = 1:size(mapData.targetGrids, 1)
            mapData.occGrid(mapData.targetGrids(k, 1), mapData.targetGrids(k, 2)) = false;
        end
        for k = 1:size(mapData.goalGrids, 1)
            mapData.occGrid(mapData.goalGrids(k, 1), mapData.goalGrids(k, 2)) = false;
        end
        drawStaticMap();
        setStatus('已生成随机障碍物');
    end

    function clearObstacles()
        mapData.occGrid = false(mapData.mapSize, mapData.mapSize);
        drawStaticMap();
        setStatus('已清空障碍物');
    end

    function onMapSizeChanged()
        val = sscanf(mapSizeDD.Value, '%dx%d');
        newSz = val(1);
        if newSz ~= mapData.mapSize
            initEmptyMap(newSz);
            drawStaticMap();
            setStatus(sprintf('地图大小已切换为 %dx%d', newSz, newSz));
        end
    end

    function onRobotCountChanged()
        nRobots = robotCountSpinner.Value;
        % 如果已有更多机器人起点则截断
        if size(mapData.startGrids, 1) > nRobots
            mapData.startGrids = mapData.startGrids(1:nRobots, :);
        end
        if size(mapData.goalGrids, 1) > nRobots
            mapData.goalGrids = mapData.goalGrids(1:nRobots, :);
        end
        updateInfoLabel();
        drawStaticMap();
        setStatus(sprintf('机器人数已设为 %d', nRobots));
    end

    function onReset()
        initEmptyMap(mapData.mapSize);
        robotCountSpinner.Value = 1;
        ctrl.lastResults = [];
        drawStaticMap();
        updateInfoLabel();
        clearAllCharts();
        setStatus('已重置');
    end

% ====================================================================
%                       核心测试逻辑
% ====================================================================

    function onRun()
        if ctrl.running
            return;
        end
        nRobots = size(mapData.startGrids, 1);
        nTargets = size(mapData.targetGrids, 1);

        if nRobots < 1
            setStatus('错误: 请先设置至少一个机器人起点');
            return;
        end
        if nTargets < 1
            setStatus('错误: 请先添加至少一个目标点');
            return;
        end

        selectedAlgos = clusterList.Value;
        if isempty(selectedAlgos)
            setStatus('错误: 请至少选择一个聚类算法');
            return;
        end
        if ~iscell(selectedAlgos)
            selectedAlgos = {selectedAlgos};
        end

        plannerName = plannerDD.Value;

        ctrl.running = true;
        setStatus('正在构建地图...');
        drawnow;

        % 构建 Map 对象
        m = Map(mapData.mapSize);
        [obsR, obsC] = find(mapData.occGrid);
        if ~isempty(obsR)
            m.setStaticObstacle(obsR, obsC);
        end

        % 计算距离矩阵（用于聚类成本评估）
        setStatus('正在计算目标点间距离...');
        drawnow;
        distMatrix = computeDistMatrix(mapData.targetGrids);

        % 计算机器人到目标点距离矩阵（用于路径成本）
        robotTargetDist = computeRobotTargetDist(m, plannerName);

        nAlgos = length(selectedAlgos);
        allResults = struct('algoName', {}, 'nRobots', {}, 'nTargets', {}, ...
            'assignment', {}, 'medoids', {}, 'clusterHistory', {}, ...
            'elapsedTime', {}, 'totalCost', {}, ...
            'perRobotTargets', {}, 'perRobotCost', {}, ...
            'maxClusterSize', {}, 'minClusterSize', {}, ...
            'clusterSizeStd', {}, 'balanceCoeff', {});

        for ai = 1:nAlgos
            algoName = selectedAlgos{ai};
            setStatus(sprintf('正在运行 %s (%d/%d)...', algoName, ai, nAlgos));
            drawnow;

            if strcmp(algoName, 'NearestNeighbor')
                % 最近邻分配
                t0 = tic;
                assignment = zeros(nTargets, 1);
                for t = 1:nTargets
                    bestDist = inf;
                    bestR = 1;
                    for r = 1:nRobots
                        d = robotTargetDist(r, t);
                        if d < bestDist
                            bestDist = d;
                            bestR = r;
                        end
                    end
                    assignment(t) = bestR;
                end
                elapsed = toc(t0);
                medoids = mapData.startGrids;
                clusterHistory = struct('cost', 0, 'converged', true, ...
                    'nIter', 1, 'elapsedTime', elapsed);
            else
                % 调用聚类算法
                clusterFunc = str2func(algoName);
                t0 = tic;
                if needsMap(algoName)
                    [assignment, medoids, clusterHistory] = clusterFunc(...
                        mapData.targetGrids, nRobots, [], mapData.startGrids, m);
                else
                    [assignment, medoids, clusterHistory] = clusterFunc(...
                        mapData.targetGrids, nRobots, distMatrix, mapData.startGrids);
                end
                elapsed = toc(t0);
                clusterHistory.elapsedTime = elapsed;
            end

            % 统计每机器人分配
            perRobotTgt = zeros(nRobots, 1);
            perRobotCost = zeros(nRobots, 1);
            for r = 1:nRobots
                members = find(assignment == r);
                perRobotTgt(r) = length(members);
                for mi = 1:length(members)
                    perRobotCost(r) = perRobotCost(r) + robotTargetDist(r, members(mi));
                end
            end

            totalCost = sum(perRobotCost);
            maxCS = max(perRobotTgt);
            minCS = min(perRobotTgt);
            csStd = std(perRobotTgt);
            balCoeff = csStd / (mean(perRobotTgt) + eps);

            allResults(ai).algoName = algoName;
            allResults(ai).nRobots = nRobots;
            allResults(ai).nTargets = nTargets;
            allResults(ai).assignment = assignment;
            allResults(ai).medoids = medoids;
            allResults(ai).clusterHistory = clusterHistory;
            allResults(ai).elapsedTime = elapsed;
            allResults(ai).totalCost = totalCost;
            allResults(ai).perRobotTargets = perRobotTgt;
            allResults(ai).perRobotCost = perRobotCost;
            allResults(ai).maxClusterSize = maxCS;
            allResults(ai).minClusterSize = minCS;
            allResults(ai).clusterSizeStd = csStd;
            allResults(ai).balanceCoeff = balCoeff;
        end

        ctrl.lastResults = allResults;

        % 更新可视化
        clearAllCharts();
        if nAlgos == 1
            drawClusterResult(allResults(1));
        end
        updateClusterTab(allResults(1));
        updateComparisonTab(allResults);
        updateBalanceTab(allResults);
        updateTimeTab(allResults);

        ctrl.running = false;
        setStatus(sprintf('测试完成: %d 个算法, %d 个机器人, %d 个目标点', ...
            nAlgos, nRobots, nTargets));
    end

    function distMatrix = computeDistMatrix(points)
        n = size(points, 1);
        distMatrix = zeros(n, n);
        for i = 1:n
            for j = i+1:n
                d = norm(points(i,:) - points(j,:));
                distMatrix(i, j) = d;
                distMatrix(j, i) = d;
            end
        end
    end

    function rtDist = computeRobotTargetDist(mapObj, plannerName)
        nR = size(mapData.startGrids, 1);
        nT = size(mapData.targetGrids, 1);
        rtDist = zeros(nR, nT);
        for r = 1:nR
            for t = 1:nT
                p = callPlanner(plannerName, mapObj, ...
                    mapData.startGrids(r,:), mapData.targetGrids(t,:));
                if ~isempty(p)
                    rtDist(r, t) = pathLength(p);
                else
                    rtDist(r, t) = norm(mapData.startGrids(r,:) - mapData.targetGrids(t,:));
                end
            end
        end
    end

% ====================================================================
%                       可视化更新
% ====================================================================

    function drawClusterResult(result)
        % 在主地图上绘制聚类分配（着色目标点）
        clearPathPlots();
        nR = result.nRobots;
        cmap = lines(nR);

        for k = 1:size(mapData.targetGrids, 1)
            r = mapData.targetGrids(k, 1);
            c = mapData.targetGrids(k, 2);
            ty = r - 0.5; tx = c - 0.5;
            clr = cmap(result.assignment(k), :);
            hPlt = plot(ax, tx, ty, 'o', ...
                'MarkerSize', 10, 'LineWidth', 2, ...
                'MarkerEdgeColor', clr, 'MarkerFaceColor', clr * 0.7 + 0.3);
            h.pathPlots{end + 1} = hPlt;
        end
        set(ax.Children, 'PickableParts', 'none');
    end

    function updateClusterTab(result)
        % 左：聚类散点图
        cla(clusterAx);
        hold(clusterAx, 'on');
        nR = result.nRobots;
        cmap = lines(nR);
        for r = 1:nR
            members = find(result.assignment == r);
            if ~isempty(members)
                scatter(clusterAx, mapData.targetGrids(members, 2), ...
                    mapData.targetGrids(members, 1), 40, cmap(r, :), 'filled');
            end
        end
        % 绘制 medoids
        for r = 1:nR
            scatter(clusterAx, result.medoids(r, 2), result.medoids(r, 1), ...
                120, cmap(r, :), 'p', 'LineWidth', 2);
        end
        axis(clusterAx, 'equal');
        axis(clusterAx, [0, mapData.mapSize, 0, mapData.mapSize]);
        grid(clusterAx, 'on');
        title(clusterAx, sprintf('聚类分配 - %s', result.algoName));
        xlabel(clusterAx, '列'); ylabel(clusterAx, '行');
        hold(clusterAx, 'off');

        % 右：每机器人目标数柱状图
        cla(clusterBarAx);
        bar(clusterBarAx, 1:nR, result.perRobotTargets, ...
            'FaceColor', [0.3, 0.6, 0.9]);
        xlabel(clusterBarAx, '机器人编号');
        xticks(clusterBarAx, 1:nR);
        title(clusterBarAx, '每机器人分配目标数');
        grid(clusterBarAx, 'on');
    end

    function updateComparisonTab(results)
        nAlgos = length(results);
        data = cell(nAlgos, 7);
        for ai = 1:nAlgos
            data{ai, 1} = results(ai).algoName;
            data{ai, 2} = sprintf('%.4f', results(ai).elapsedTime);
            data{ai, 3} = sprintf('%.2f', results(ai).totalCost);
            data{ai, 4} = results(ai).maxClusterSize;
            data{ai, 5} = results(ai).minClusterSize;
            data{ai, 6} = sprintf('%.2f', results(ai).clusterSizeStd);
            data{ai, 7} = sprintf('%.3f', results(ai).balanceCoeff);
        end
        compareTable.Data = data;
    end

    function updateBalanceTab(results)
        result = results(1);
        nR = result.nRobots;
        cla(balanceAx);
        hold(balanceAx, 'on');
        x = 1:nR;
        yyaxis(balanceAx, 'left');
        bar(balanceAx, x - 0.2, result.perRobotTargets, 0.35, ...
            'FaceColor', [0.3, 0.6, 0.9], 'DisplayName', '目标数');
        ylabel(balanceAx, '目标数');
        yyaxis(balanceAx, 'right');
        bar(balanceAx, x + 0.2, result.perRobotCost, 0.35, ...
            'FaceColor', [0.9, 0.4, 0.3], 'DisplayName', '路径成本');
        ylabel(balanceAx, '路径成本');
        xlabel(balanceAx, '机器人编号');
        xticks(balanceAx, 1:nR);
        legend(balanceAx, 'Location', 'best');
        title(balanceAx, '负载均衡分析');
        grid(balanceAx, 'on');
        hold(balanceAx, 'off');

        balanceInfoLabel.Text = sprintf(['算法: %s\n均衡系数: %.3f (越小越均衡)\n' ...
            '簇大小: %d / %d (最大/最小)\n总分配成本: %.2f'], ...
            result.algoName, result.balanceCoeff, ...
            result.maxClusterSize, result.minClusterSize, result.totalCost);
    end

    function updateTimeTab(results)
        nAlgos = length(results);
        cla(timeAx);
        times = [results.elapsedTime];
        bar(timeAx, 1:nAlgos, times, 'FaceColor', [0.4, 0.75, 0.5]);
        xticklabels(timeAx, {results.algoName});
        xticks(timeAx, 1:nAlgos);
        ylabel(timeAx, '耗时 (s)');
        title(timeAx, '聚类算法耗时对比');
        grid(timeAx, 'on');
    end

    function clearAllCharts()
        cla(clusterAx);
        cla(clusterBarAx);
        compareTable.Data = {};
        cla(balanceAx);
        balanceInfoLabel.Text = '';
        cla(timeAx);
    end

% ====================================================================
%                       工具函数
% ====================================================================

    function setStatus(msg)
        statusLabel.Text = msg;
        drawnow;
    end

    function onWindowClose(~, ~)
        delete(fig);
    end

end

% =========================================================================
% 局部函数
% =========================================================================

function path = callPlanner(algoName, map, startGrid, goalGrid)
    switch algoName
        case 'AStar'
            path = AStar(map, startGrid, goalGrid, 0);
        case 'AStar_v0'
            path = AStar_v0(map, startGrid, goalGrid, 0);
        case 'AStar_v1'
            path = AStar_v1(map, startGrid, goalGrid, 0);
        case 'AStar_v2'
            path = AStar_v2(map, startGrid, goalGrid, 0);
        case 'AStar_v3'
            path = AStar_v3(map, startGrid, goalGrid, 0);
        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0);
        case 'Dijkstra_v1'
            path = Dijkstra_v1(map, startGrid, goalGrid, 0);
        case 'RRT'
            path = RRT(map, startGrid, goalGrid, 0);
        otherwise
            path = AStar(map, startGrid, goalGrid, 0);
    end
end

function d = pathLength(path)
    d = 0;
    for i = 1:size(path, 1) - 1
        d = d + norm(path(i+1, :) - path(i, :));
    end
end

function algoList = getClusterAlgoList()
    clusterDir = fullfile(fileparts(mfilename('fullpath')), '..', 'ClusteringOptimization');
    files = dir(fullfile(clusterDir, '*_Cluster*.m'));
    algoList = cell(1, length(files) + 1);
    algoList{1} = 'NearestNeighbor';
    for i = 1:length(files)
        [~, algoList{i + 1}] = fileparts(files(i).name);
    end
end

function tf = needsMap(algoName)
    % 判断聚类算法是否需要 Map 对象（路径距离版本）
    tf = contains(algoName, '_v1') || contains(algoName, '_v2') || contains(algoName, 'DV_Cluster');
end
