function varargout = TSPTesterUI(mainUIState)
%TSPTESTERUI TSP 算法对比测试窗口
%   TSPTesterUI(mainUIState)
%   fig = TSPTesterUI(mainUIState)
%   独立窗口，用于测试和对比多个 TSP 算法的性能。
%   mainUIState: MainUI 的 state 结构体（用于导入地图），可为空。

% ===== 窗口创建 =====
fig = uifigure('Name', 'TSP 算法对比测试', ...
    'Position', [150, 80, 1000, 700], ...
    'CloseRequestFcn', @onWindowClose, ...
    'Resize', 'off');

% ===== 地图数据（算法坐标 [row, col]） =====
mapData = struct('mapSize', 30, ...
    'occGrid', [], ...
    'startGrid', [], ...
    'goalGrid', [], ...
    'targetGrids', []);    % K×2 [row, col]

% ===== 控制状态 =====
ctrl = struct('running', false, ...
    'lastResults', [], ...
    'lastCostMatrix', [], ...
    'lastAllPoints', []);

% ===== 可视化句柄 =====
h = struct('startPlot', [], ...
    'goalPlot', [], ...
    'targetPlots', [], ...
    'targetLabels', [], ...
    'obsRects', [], ...
    'pathPlots', {{}});

% ===== 编辑模式 =====
setMode = 'none';  % 'none', 'start', 'goal', 'target', 'obstacle', 'delete'

% ===== UI 布局 =====
% 状态栏
statusLabel = uilabel(fig, 'Text', '就绪', ...
    'Position', [20, 674, 960, 20], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold');

% 左侧地图坐标轴
ax = uiaxes(fig, 'Position', [20, 60, 480, 480]);
plotTools('setupAxes', ax, mapData.mapSize);
title(ax, 'TSP 测试地图');
ax.ButtonDownFcn = @onMapClick;

% 底部编辑模式按钮
setStartBtn = uibutton(fig, 'state', 'Text', '设起点', ...
    'Position', [25, 28, 65, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('start', evt.Value));
setGoalBtn = uibutton(fig, 'state', 'Text', '设终点', ...
    'Position', [95, 28, 65, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('goal', evt.Value));
addTargetBtn = uibutton(fig, 'state', 'Text', '加目标点', ...
    'Position', [165, 28, 75, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('target', evt.Value));
addObsBtn = uibutton(fig, 'state', 'Text', '加障碍', ...
    'Position', [245, 28, 65, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('obstacle', evt.Value));
delObjBtn = uibutton(fig, 'state', 'Text', '删对象', ...
    'Position', [315, 28, 65, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('delete', evt.Value));

allModeBtns = [setStartBtn, setGoalBtn, addTargetBtn, addObsBtn, delObjBtn];

% 地图大小选择
uilabel(fig, 'Text', '地图:', 'Position', [390, 30, 30, 18]);
mapSizeDD = uidropdown(fig, ...
    'Items', {'20×20', '30×30', '40×40'}, ...
    'Value', '30×30', ...
    'Position', [420, 28, 75, 22], ...
    'ValueChangedFcn', @(~,~) onMapSizeChanged());

% ===== 右侧面板 =====
panel = uipanel(fig, 'Position', [520, 10, 470, 680], ...
    'Title', '控制与结果', 'FontSize', 11);

% --- 算法选择区 ---
algoPanel = uipanel(panel, 'Title', '算法选择', ...
    'Position', [6, 530, 458, 130], 'FontSize', 10);

uilabel(algoPanel, 'Text', 'TSP 算法 (可多选):', ...
    'Position', [10, 85, 120, 18]);
tspList = uilistbox(algoPanel, ...
    'Items', getTSPAlgoList(), ...
    'MultiSelect', 'on', ...
    'Value', {'TSP_GA'}, ...
    'Position', [10, 20, 200, 65]);

uilabel(algoPanel, 'Text', '路径规划器:', ...
    'Position', [230, 85, 80, 18]);
plannerDD = uidropdown(algoPanel, ...
    'Items', {'AStar', 'AStar_v0', 'AStar_v1', 'AStar_v2', 'AStar_v3', 'Dijkstra', 'Dijkstra_v1', 'RRT'}, ...
    'Value', 'AStar', ...
    'Position', [230, 60, 100, 22]);

uilabel(algoPanel, 'Text', '运行次数:', ...
    'Position', [230, 30, 70, 18]);
runCountSpinner = uispinner(algoPanel, ...
    'Limits', [1, 50], 'Value', 1, 'Step', 1, ...
    'Position', [305, 28, 60, 22]);

% --- 操作按钮区 ---
btnPanel = uipanel(panel, 'Position', [6, 475, 458, 50], 'FontSize', 10);

runBtn = uibutton(btnPanel, 'push', 'Text', '▶ 开始测试', ...
    'Position', [10, 8, 120, 32], ...
    'ButtonPushedFcn', @(~,~) onRun(), ...
    'BackgroundColor', [0.8, 1.0, 0.8]);

resetBtn = uibutton(btnPanel, 'push', 'Text', '↺ 重置', ...
    'Position', [140, 8, 80, 32], ...
    'ButtonPushedFcn', @(~,~) onReset());

importBtn = uibutton(btnPanel, 'push', 'Text', '导入主界面地图', ...
    'Position', [230, 8, 110, 32], ...
    'ButtonPushedFcn', @(~,~) importMapFromMainUI());

randomBtn = uibutton(btnPanel, 'push', 'Text', '随机障碍', ...
    'Position', [350, 8, 95, 32], ...
    'ButtonPushedFcn', @(~,~) randomObstacles());

% --- 结果区（Tab 组）---
tabGroup = uitabgroup(panel, 'Position', [6, 10, 458, 460]);

% Tab 1: 收敛曲线
tabConv = uitab(tabGroup, 'Title', '收敛曲线');
convAx = uiaxes(tabConv, 'Position', [40, 50, 400, 370]);
title(convAx, '收敛曲线');
xlabel(convAx, '迭代/排列');
ylabel(convAx, '最优成本');
grid(convAx, 'on');

% Tab 2: 成本矩阵
tabHeat = uitab(tabGroup, 'Title', '成本矩阵');
heatAx = uiaxes(tabHeat, 'Position', [40, 50, 400, 370]);
title(heatAx, '成本矩阵热力图');

% Tab 3: 算法对比
tabComp = uitab(tabGroup, 'Title', '算法对比');
compAx = uiaxes(tabComp, 'Position', [40, 200, 400, 220]);
title(compAx, '算法对比');
ylabel(compAx, '平均最优成本');
grid(compAx, 'on');

% Tab 4: 耗时-成本收敛曲线
tabTime = uitab(tabGroup, 'Title', '耗时-成本');
timeAx = uiaxes(tabTime, 'Position', [40, 50, 400, 370]);
title(timeAx, '耗时-成本收敛曲线');
xlabel(timeAx, '累计耗时 (s)');
ylabel(timeAx, '最优成本');
grid(timeAx, 'on');

statsLabel = uilabel(tabComp, 'Text', '', ...
    'Position', [10, 10, 430, 180], ...
    'VerticalAlignment', 'top', 'FontSize', 9);

% ===== 初始化 =====
initEmptyMap(mapData.mapSize);
if nargin >= 1 && ~isempty(mainUIState)
    importMapFromMainUI(mainUIState);
end
drawStaticMap();
if nargout > 0
    varargout{1} = fig;
end

% ================================================================
%                      嵌套回调函数
% ================================================================

    function onModeBtnChanged(mode, val)
        if val
            allModes = {'start', 'goal', 'target', 'obstacle', 'delete'};
            btns = {setStartBtn, setGoalBtn, addTargetBtn, addObsBtn, delObjBtn};
            for k = 1:length(allModes)
                if ~strcmp(allModes{k}, mode)
                    btns{k}.Value = false;
                end
            end
            setMode = mode;
        else
            setMode = 'none';
        end
    end

    function onMapClick(~, evt)
        cp = ax.CurrentPoint;
        x = cp(1, 1); y = cp(1, 2);
        col = ceil(x); row = ceil(y);
        if row < 1 || row > mapData.mapSize || col < 1 || col > mapData.mapSize
            return;
        end

        switch setMode
            case 'start'
                mapData.startGrid = [row, col];
                setStatus('起点已设置');
            case 'goal'
                mapData.goalGrid = [row, col];
                setStatus('终点已设置');
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
        drawStaticMap();
    end

    function initEmptyMap(sz)
        mapData.mapSize = sz;
        mapData.occGrid = false(sz, sz);
        mapData.startGrid = [];
        mapData.goalGrid = [];
        mapData.targetGrids = [];
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

        % 起点
        if ~isempty(mapData.startGrid)
            sy = mapData.startGrid(1) - 0.5;
            sx = mapData.startGrid(2) - 0.5;
            plot(ax, sx, sy, 'o', ...
                'MarkerSize', 10, 'LineWidth', 2, ...
                'MarkerEdgeColor', plotTools('getColor', 'start'), ...
                'MarkerFaceColor', 'none');
            text(ax, sx + 0.3, sy + 0.3, 'S', 'FontSize', 9, ...
                'FontWeight', 'bold', 'Color', plotTools('getColor', 'start'));
        end

        % 终点
        if ~isempty(mapData.goalGrid)
            gy = mapData.goalGrid(1) - 0.5;
            gx = mapData.goalGrid(2) - 0.5;
            plot(ax, gx, gy, 'x', ...
                'MarkerSize', 12, 'LineWidth', 3, ...
                'Color', plotTools('getColor', 'goal'));
            text(ax, gx + 0.3, gy + 0.3, 'G', 'FontSize', 9, ...
                'FontWeight', 'bold', 'Color', plotTools('getColor', 'goal'));
        end

        % 目标点
        h.targetPlots = gobjects(0);
        h.targetLabels = gobjects(0);
        for k = 1:size(mapData.targetGrids, 1)
            r = mapData.targetGrids(k, 1);
            c = mapData.targetGrids(k, 2);
            ty = r - 0.5; tx = c - 0.5;
            h.targetPlots(end + 1) = plot(ax, tx, ty, 'o', ...
                'MarkerSize', 8, 'LineWidth', 2, ...
                'MarkerEdgeColor', plotTools('getColor', 'target'), ...
                'MarkerFaceColor', [0.5, 0.5, 1.0]);
            h.targetLabels(end + 1) = text(ax, tx + 0.3, ty + 0.3, ...
                sprintf('T%d', k), 'FontSize', 8, 'FontWeight', 'bold');
        end

        % 使所有子图元不拦截点击，确保 ButtonDownFcn 能正确获取点击坐标
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

    function clearConvergenceChart()
        % 显式删除 fill 阴影带对象（cla 可能无法清除 fill）
        fillObjs = findobj(convAx, 'Type', 'Patch');
        for k = 1:length(fillObjs)
            delete(fillObjs(k));
        end
        cla(convAx);
    end

    function clearTimeCostChart()
        fillObjs = findobj(timeAx, 'Type', 'Patch');
        for k = 1:length(fillObjs)
            delete(fillObjs(k));
        end
        cla(timeAx);
    end

    function deleteObjectAt(row, col)
        % 检查目标点
        if ~isempty(mapData.targetGrids)
            match = mapData.targetGrids(:, 1) == row & mapData.targetGrids(:, 2) == col;
            if any(match)
                mapData.targetGrids(match, :) = [];
                setStatus(sprintf('目标点 (%d,%d) 已删除', row, col));
                return;
            end
        end
        % 检查起点
        if ~isempty(mapData.startGrid) && isequal(mapData.startGrid, [row, col])
            mapData.startGrid = [];
            setStatus('起点已删除');
            return;
        end
        % 检查终点
        if ~isempty(mapData.goalGrid) && isequal(mapData.goalGrid, [row, col])
            mapData.goalGrid = [];
            setStatus('终点已删除');
            return;
        end
        % 检查障碍物
        if mapData.occGrid(row, col)
            mapData.occGrid(row, col) = false;
            setStatus(sprintf('障碍物 (%d,%d) 已删除', row, col));
            return;
        end
        setStatus('该位置无对象可删除');
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

        % 起点
        if isfield(state, 'startPoint') && ~isempty(state.startPoint)
            mapData.startGrid = [state.startPoint(2), state.startPoint(1)];
        elseif isfield(state, 'startPoints') && ~isempty(state.startPoints)
            mapData.startGrid = [state.startPoints(1, 2), state.startPoints(1, 1)];
        end

        % 终点
        if isfield(state, 'goalPoint') && ~isempty(state.goalPoint)
            mapData.goalGrid = [state.goalPoint(2), state.goalPoint(1)];
        elseif isfield(state, 'goalPoints') && ~isempty(state.goalPoints)
            mapData.goalGrid = [state.goalPoints(1, 2), state.goalPoints(1, 1)];
        end

        % 目标点
        mapData.targetGrids = [];
        if isfield(state, 'targetPoints') && ~isempty(state.targetPoints)
            mapData.targetGrids = [state.targetPoints(:, 2), state.targetPoints(:, 1)];
        end

        % 更新地图大小下拉框
        mapSizeDD.Value = sprintf('%d×%d', mapData.mapSize, mapData.mapSize);

        drawStaticMap();
        setStatus(sprintf('已导入 %dx%d 地图, %d 个目标点', ...
            mapData.mapSize, mapData.mapSize, size(mapData.targetGrids, 1)));
    end

    function randomObstacles()
        mapData.occGrid = rand(mapData.mapSize, mapData.mapSize) < 0.25;
        if ~isempty(mapData.startGrid)
            mapData.occGrid(mapData.startGrid(1), mapData.startGrid(2)) = false;
        end
        if ~isempty(mapData.goalGrid)
            mapData.occGrid(mapData.goalGrid(1), mapData.goalGrid(2)) = false;
        end
        for k = 1:size(mapData.targetGrids, 1)
            mapData.occGrid(mapData.targetGrids(k, 1), mapData.targetGrids(k, 2)) = false;
        end
        drawStaticMap();
        setStatus('已生成随机障碍物');
    end

    function onMapSizeChanged()
        val = sscanf(mapSizeDD.Value, '%d×%d');
        newSz = val(1);
        if newSz ~= mapData.mapSize
            initEmptyMap(newSz);
            drawStaticMap();
            setStatus(sprintf('地图大小已切换为 %d×%d', newSz, newSz));
        end
    end

% ================================================================
%                      核心测试逻辑
% ================================================================

    function onRun()
        if ctrl.running
            return;
        end
        if isempty(mapData.startGrid)
            setStatus('错误: 请先设置起点');
            return;
        end
        if isempty(mapData.goalGrid)
            setStatus('错误: 请先设置终点');
            return;
        end
        if isempty(mapData.targetGrids)
            setStatus('错误: 请先添加至少一个目标点');
            return;
        end

        selectedAlgos = tspList.Value;
        if isempty(selectedAlgos)
            setStatus('错误: 请至少选择一个 TSP 算法');
            return;
        end
        if ~iscell(selectedAlgos)
            selectedAlgos = {selectedAlgos};
        end

        % 检查 Permutation 算法的规模限制
        nMid = size(mapData.targetGrids, 1);
        for ai = 1:length(selectedAlgos)
            if strcmp(selectedAlgos{ai}, 'TSP_Permutation') && nMid > 7
                setStatus(sprintf('错误: TSP_Permutation 不支持 %d 个目标点 (上限 7)', nMid));
                return;
            end
        end

        nRuns = runCountSpinner.Value;
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

        % 计算成本矩阵（所有算法共享）
        setStatus('正在计算成本矩阵...');
        drawnow;
        [costMatrix, allPoints] = computeCostMatrix(m, plannerName);

        % 检查可达性
        if any(isinf(costMatrix(1, 2:end-1)))
            setStatus('错误: 存在不可达的目标点，请检查地图');
            ctrl.running = false;
            return;
        end

        nPts = size(allPoints, 1);

        % 运行各算法
        nAlgos = length(selectedAlgos);
        allResults = struct('algoName', {}, 'nRuns', {}, ...
            'bestCosts', {}, 'histories', {}, 'times', {}, ...
            'bestCost', {}, 'worstCost', {}, 'avgCost', {}, ...
            'stdCost', {}, 'avgTime', {}, 'bestOrder', {}, 'bestHistory', {});

        for ai = 1:nAlgos
            algoName = selectedAlgos{ai};
            setStatus(sprintf('正在运行 %s (%d/%d)...', algoName, ai, nAlgos));
            drawnow;

            runBestCosts = zeros(nRuns, 1);
            runHistories = cell(nRuns, 1);
            runTimes = zeros(nRuns, 1);
            runOrders = cell(nRuns, 1);

            tspFunc = str2func(algoName);
            for ri = 1:nRuns
                [order, cost, hist] = tspFunc(costMatrix, nPts);
                runBestCosts(ri) = cost;
                runHistories{ri} = hist;
                runTimes(ri) = hist.elapsedTime;
                runOrders{ri} = order;
            end

            [~, bestRunIdx] = min(runBestCosts);

            allResults(ai).algoName = algoName;
            allResults(ai).nRuns = nRuns;
            allResults(ai).bestCosts = runBestCosts;
            allResults(ai).histories = runHistories;
            allResults(ai).times = runTimes;
            allResults(ai).orders = runOrders;
            allResults(ai).bestCost = min(runBestCosts);
            allResults(ai).worstCost = max(runBestCosts);
            allResults(ai).avgCost = mean(runBestCosts);
            allResults(ai).stdCost = std(runBestCosts);
            allResults(ai).avgTime = mean(runTimes);
            allResults(ai).bestOrder = runOrders{bestRunIdx};
            allResults(ai).bestHistory = runHistories{bestRunIdx};
        end

        ctrl.lastResults = allResults;
        ctrl.lastCostMatrix = costMatrix;
        ctrl.lastAllPoints = allPoints;

        % 更新可视化
        drawFinalPaths(allResults, allPoints);
        clearConvergenceChart();
        updateConvergenceTab(allResults);
        clearTimeCostChart();
        updateTimeCostTab(allResults);
        updateHeatmapTab(costMatrix, allPoints);
        updateComparisonTab(allResults);

        ctrl.running = false;
        setStatus(sprintf('测试完成: %d 个算法, 各 %d 次运行', nAlgos, nRuns));
    end

    function [costMatrix, allPoints] = computeCostMatrix(map, algoName)
        allPoints = [mapData.startGrid; mapData.targetGrids; mapData.goalGrid];
        nPts = size(allPoints, 1);
        costMatrix = inf(nPts, nPts);

        for i = 1:nPts
            for j = 1:nPts
                if i == j
                    costMatrix(i, j) = 0;
                    continue;
                end
                p = callPlanner(algoName, map, allPoints(i, :), allPoints(j, :));
                if ~isempty(p)
                    costMatrix(i, j) = pathLength(p);
                end
            end
            setStatus(sprintf('计算成本矩阵: %d/%d 行', i, nPts));
            drawnow;
        end
    end

% ================================================================
%                      可视化更新
% ================================================================

    function drawFinalPaths(results, allPoints)
        clearPathPlots();

        algoColors = {[1, 0, 0], [0, 0, 1], [0, 0.7, 0], [1, 0.5, 0], [0.6, 0, 0.6], [0, 0.6, 0.6]};

        for ai = 1:length(results)
            order = results(ai).bestOrder;
            color = algoColors{mod(ai - 1, length(algoColors)) + 1};

            % 绘制路径段
            for k = 1:(length(order) - 1)
                p1 = allPoints(order(k), :);
                p2 = allPoints(order(k + 1), :);
                h.pathPlots{end + 1} = plot(ax, ...
                    [p1(2) - 0.5, p2(2) - 0.5], [p1(1) - 0.5, p2(1) - 0.5], ...
                    '-', 'Color', color, 'LineWidth', 2, ...
                    'DisplayName', results(ai).algoName);
            end

            % 标注访问顺序
            visitIdx = 2:(length(order) - 1);
            for vi = 1:length(visitIdx)
                pt = allPoints(order(visitIdx(vi)), :);
                h.pathPlots{end + 1} = text(ax, pt(2) - 0.2, pt(1) + 0.3, ...
                    sprintf('%d', vi), 'FontSize', 9, 'FontWeight', 'bold', ...
                    'Color', color);
            end
        end

        if length(results) > 1
            legend(ax, 'show', 'Location', 'northeast');
        end
        drawnow;
    end

    function updateConvergenceTab(results)
        cla(convAx);
        hold(convAx, 'on');

        algoColors = {[1, 0, 0], [0, 0, 1], [0, 0.7, 0], [1, 0.5, 0], [0.6, 0, 0.6], [0, 0.6, 0.6]};

        for ai = 1:length(results)
            color = algoColors{mod(ai - 1, length(algoColors)) + 1};
            nRuns = results(ai).nRuns;

            if nRuns == 1
                hist = results(ai).histories{1};
                plot(convAx, 1:length(hist.bestCostHistory), hist.bestCostHistory, ...
                    '-', 'Color', color, 'LineWidth', 1.5, ...
                    'DisplayName', results(ai).algoName);
            else
                histories = results(ai).histories;
                minLen = min(cellfun(@(h) length(h.bestCostHistory), histories));
                allCurves = zeros(nRuns, minLen);
                for ri = 1:nRuns
                    allCurves(ri, :) = histories{ri}.bestCostHistory(1:minLen)';
                end
                meanCurve = mean(allCurves, 1);
                stdCurve = std(allCurves, 0, 1);
                x = 1:minLen;

                % 标准差阴影带
                fill(convAx, [x, fliplr(x)], ...
                    [meanCurve + stdCurve, fliplr(meanCurve - stdCurve)], ...
                    color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
                    'HandleVisibility', 'off');
                plot(convAx, x, meanCurve, '-', 'Color', color, 'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s (n=%d)', results(ai).algoName, nRuns));
            end
        end

        xlabel(convAx, '迭代/排列');
        ylabel(convAx, '最优成本');
        title(convAx, '收敛曲线');
        legend(convAx, 'show', 'Location', 'northeast');
        grid(convAx, 'on');
        hold(convAx, 'off');
    end

    function updateTimeCostTab(results)
        cla(timeAx);
        hold(timeAx, 'on');

        algoColors = {[1, 0, 0], [0, 0, 1], [0, 0.7, 0], [1, 0.5, 0], [0.6, 0, 0.6], [0, 0.6, 0.6]};

        for ai = 1:length(results)
            color = algoColors{mod(ai - 1, length(algoColors)) + 1};
            nRuns = results(ai).nRuns;

            if nRuns == 1
                hist = results(ai).histories{1};
                plot(timeAx, hist.timeHistory, hist.bestCostHistory, ...
                    '-', 'Color', color, 'LineWidth', 1.5, ...
                    'DisplayName', results(ai).algoName);
            else
                histories = results(ai).histories;
                minLen = min(cellfun(@(h) length(h.timeHistory), histories));
                allTimes = zeros(nRuns, minLen);
                allCosts = zeros(nRuns, minLen);
                for ri = 1:nRuns
                    allTimes(ri, :) = histories{ri}.timeHistory(1:minLen)';
                    allCosts(ri, :) = histories{ri}.bestCostHistory(1:minLen)';
                end
                meanTimes = mean(allTimes, 1);
                meanCosts = mean(allCosts, 1);
                stdCosts = std(allCosts, 0, 1);

                % 标准差阴影带
                fill(timeAx, [meanTimes, fliplr(meanTimes)], ...
                    [meanCosts + stdCosts, fliplr(meanCosts - stdCosts)], ...
                    color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', ...
                    'HandleVisibility', 'off');
                plot(timeAx, meanTimes, meanCosts, '-', 'Color', color, 'LineWidth', 1.5, ...
                    'DisplayName', sprintf('%s (n=%d)', results(ai).algoName, nRuns));
            end
        end

        xlabel(timeAx, '累计耗时 (s)');
        ylabel(timeAx, '最优成本');
        title(timeAx, '耗时-成本收敛曲线');
        legend(timeAx, 'show', 'Location', 'northeast');
        grid(timeAx, 'on');
        hold(timeAx, 'off');
    end

    function updateHeatmapTab(costMatrix, allPoints)
        cla(heatAx);

        nPts = size(costMatrix, 1);
        displayMatrix = costMatrix;
        displayMatrix(isinf(displayMatrix)) = NaN;

        imagesc(heatAx, displayMatrix);
        colorbar(heatAx);
        colormap(heatAx, 'hot');

        % 坐标轴标签
        labels = cell(1, nPts);
        labels{1} = 'S';
        labels{nPts} = 'G';
        for k = 2:(nPts - 1)
            labels{k} = sprintf('T%d', k - 1);
        end
        set(heatAx, 'XTick', 1:nPts, 'XTickLabel', labels);
        set(heatAx, 'YTick', 1:nPts, 'YTickLabel', labels);
        title(heatAx, '成本矩阵热力图');
        xlabel(heatAx, '目标点');
        ylabel(heatAx, '目标点');

        % 数值标注
        for i = 1:nPts
            for j = 1:nPts
                if ~isnan(displayMatrix(i, j))
                    text(heatAx, j, i, sprintf('%.1f', displayMatrix(i, j)), ...
                        'HorizontalAlignment', 'center', 'FontSize', 7);
                end
            end
        end
    end

    function updateComparisonTab(results)
        cla(compAx);

        nAlgos = length(results);
        names = cell(1, nAlgos);
        avgCosts = zeros(1, nAlgos);

        for ai = 1:nAlgos
            names{ai} = results(ai).algoName;
            avgCosts(ai) = results(ai).avgCost;
        end

        bar(compAx, avgCosts);
        set(compAx, 'XTickLabel', names, 'XTickLabelRotation', 15);
        ylabel(compAx, '平均最优成本');
        title(compAx, '算法对比');
        grid(compAx, 'on');

        % 统计表格
        statsStr = '';
        for ai = 1:nAlgos
            r = results(ai);
            statsStr = [statsStr, sprintf(['%s:\n  最优=%.2f  最差=%.2f  平均=%.2f\n' ...
                '  标准差=%.2f  平均耗时=%.4fs\n'], ...
                r.algoName, r.bestCost, r.worstCost, r.avgCost, r.stdCost, r.avgTime)];
        end
        set(statsLabel, 'Text', statsStr);
    end

% ================================================================
%                      重置与清理
% ================================================================

    function onReset()
        ctrl.running = false;
        ctrl.lastResults = [];
        ctrl.lastCostMatrix = [];
        ctrl.lastAllPoints = [];

        % 清空地图数据
        mapData.startGrid = [];
        mapData.goalGrid = [];
        mapData.targetGrids = [];
        mapData.occGrid = false(mapData.mapSize, mapData.mapSize);

        clearPathPlots();
        clearConvergenceChart();
        clearTimeCostChart();
        cla(heatAx);
        cla(compAx);
        set(statsLabel, 'Text', '');

        drawStaticMap();
        setStatus('已重置');
    end

    function onWindowClose(~, ~)
        ctrl.running = false;
        delete(fig);
    end

    function setStatus(msg)
        set(statusLabel, 'Text', sprintf('状态: %s', msg));
    end

end

% =========================================================================
%                      局部辅助函数
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

function len = pathLength(path)
    if isempty(path)
        len = inf;
        return;
    end
    len = 0;
    for i = 2:size(path, 1)
        dr = path(i, 1) - path(i - 1, 1);
        dc = path(i, 2) - path(i - 1, 2);
        if dr ~= 0 && dc ~= 0
            len = len + sqrt(2);
        else
            len = len + 1;
        end
    end
end

function algoList = getTSPAlgoList()
    tspDir = fullfile(fileparts(mfilename('fullpath')), '..', 'TSPOptimization');
    files = dir(fullfile(tspDir, 'TSP_*.m'));
    algoList = cell(1, length(files));
    for i = 1:length(files)
        [~, algoList{i}] = fileparts(files(i).name);
    end
    if isempty(algoList)
        algoList = {'TSP_GA'};
    end
end
