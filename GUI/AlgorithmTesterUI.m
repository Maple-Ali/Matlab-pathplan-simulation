function varargout = AlgorithmTesterUI(mainUIState)
%ALGORITHMTESTERUI 全局路径规划算法测试窗口
%   AlgorithmTesterUI(mainUIState)
%   fig = AlgorithmTesterUI(mainUIState)
%   独立窗口，用于测试和可视化全局路径规划算法的搜索过程。
%   mainUIState: MainUI 的 state 结构体（用于导入地图），可为空。

% ===== 窗口创建 =====
fig = uifigure('Name', '全局路径规划算法测试', ...
    'Position', [150, 100, 620, 530], ...
    'CloseRequestFcn', @onWindowClose, ...
    'Resize', 'off');

% ===== 地图数据（算法坐标 [row, col]） =====
mapData = struct('mapSize', 30, ...
    'occGrid', [], ...
    'startGrid', [], ...
    'goalGrid', []);

% ===== 控制状态 =====
ctrl = struct('running', false, ...
    'paused', false, ...
    'stopRequested', false, ...
    'stepRequested', false, ...
    'speed', 0.05);

% ===== 可视化句柄 =====
h = struct('openScatter', [], ...
    'closedScatter', [], ...
    'currentPlot', [], ...
    'pathPlot', [], ...
    'obsRects', [], ...
    'startPlot', [], ...
    'goalPlot', []);

% ===== 持久状态用于 step 间保存 open/closed =====
prevOpen = [];
prevClosed = [];
algStartTime = [];

% ===== 右键设置起点/终点的临时状态 =====
setMode = 'none';    % 'none', 'start', 'goal'

% ===== UI 布局 =====
% 左侧地图坐标轴
ax = uiaxes(fig, 'Position', [20, 40, 420, 420]);
plotTools('setupAxes', ax, mapData.mapSize);
title(ax, 'A* 搜索过程可视化');
ax.ButtonDownFcn = @onMapClick;

% 底部模式选择（使用 uibutton state 按钮实现多选一）
setStartBtn = uibutton(fig, 'state', 'Text', '设起点', ...
    'Position', [25, 8, 60, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('start', evt.Value));
setGoalBtn = uibutton(fig, 'state', 'Text', '设终点', ...
    'Position', [90, 8, 60, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('goal', evt.Value));
addObsBtn = uibutton(fig, 'state', 'Text', '加障碍', ...
    'Position', [155, 8, 60, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('obstacle', evt.Value));
delObsBtn = uibutton(fig, 'state', 'Text', '删障碍', ...
    'Position', [220, 8, 60, 22], ...
    'ValueChangedFcn', @(~, evt) onModeBtnChanged('delete', evt.Value));

allModeBtns = [setStartBtn, setGoalBtn, addObsBtn, delObsBtn];

% 右侧控制面板
panel = uipanel(fig, 'Position', [460, 10, 150, 510], ...
    'Title', '控制面板', 'FontSize', 11);

% 算法选择
uilabel(panel, 'Text', '算法:', 'Position', [10, 468, 40, 20]);
algoDD = uidropdown(panel, ...
    'Items', {'AStar', 'Dijkstra', 'RRT'}, ...
    'Value', 'AStar', ...
    'Position', [50, 465, 90, 22], ...
    'ValueChangedFcn', @(~,~) onReset());

% 速度滑块
uilabel(panel, 'Text', '速度:', 'Position', [10, 435, 40, 20]);
speedSlider = uislider(panel, ...
    'Position', [50, 443, 90, 3], ...
    'Limits', [0, 0.3], ...
    'Value', 0.05, ...
    'ValueChangedFcn', @(src,~) onSpeedChanged(src));
speedValLabel = uilabel(panel, 'Text', '0.05s', ...
    'Position', [105, 423, 35, 15], ...
    'HorizontalAlignment', 'right');

% 速度显示更新
speedSlider.ValueChangingFcn = @(src,~) set(speedValLabel, 'Text', ...
    sprintf('%.3fs', src.Value));

% 控制按钮
runBtn = uibutton(panel, 'push', 'Text', '▶ 运行', ...
    'Position', [10, 400, 60, 30], ...
    'ButtonPushedFcn', @(~,~) onRun(), ...
    'BackgroundColor', [0.8, 1.0, 0.8]);
pauseBtn = uibutton(panel, 'push', 'Text', '⏸ 暂停', ...
    'Position', [80, 400, 60, 30], ...
    'ButtonPushedFcn', @(~,~) onPause());
stepBtn = uibutton(panel, 'push', 'Text', '⏭ 单步', ...
    'Position', [10, 365, 60, 30], ...
    'ButtonPushedFcn', @(~,~) onStep());
resetBtn = uibutton(panel, 'push', 'Text', '↺ 重置', ...
    'Position', [80, 365, 60, 30], ...
    'ButtonPushedFcn', @(~,~) onReset());

% 统计面板
statsPanel = uipanel(panel, 'Position', [6, 150, 138, 200], ...
    'Title', '统计信息', 'FontSize', 10);
iterLabel = uilabel(statsPanel, 'Text', '迭代: 0', ...
    'Position', [5, 153, 128, 18]);
openLabel = uilabel(statsPanel, 'Text', 'Open集: 0', ...
    'Position', [5, 130, 128, 18]);
closedLabel = uilabel(statsPanel, 'Text', '已探索: 0', ...
    'Position', [5, 107, 128, 18]);
pathLabel = uilabel(statsPanel, 'Text', '路径长度: -', ...
    'Position', [5, 84, 128, 18]);
timeLabel = uilabel(statsPanel, 'Text', '耗时: -', ...
    'Position', [5, 61, 128, 18]);
statusLabel = uilabel(statsPanel, 'Text', '状态: 就绪', ...
    'Position', [5, 38, 128, 18], ...
    'FontWeight', 'bold', 'FontColor', [0, 0.4, 0]);

% 图例说明（小面板）
legendPanel = uipanel(panel, 'Position', [6, 10, 138, 130], ...
    'Title', '图例', 'FontSize', 10);
% 用 uiaxes 绘制图例色块
legAx = uiaxes(legendPanel, 'Position', [5, 5, 128, 100]);
legAx.Visible = 'off';
hold(legAx, 'on');
legendItems = {
    [0.75, 0.75, 0.75], '已探索';
    [0.6, 0.95, 0.6], 'Open Set';
    [1.0, 0.5, 0.0],  '当前节点';
    [0.0, 0.3, 1.0],  '最终路径';
};
yPos = 85;
for i = 1:size(legendItems, 1)
    rectangle(legAx, 'Position', [2, yPos-4, 12, 12], ...
        'FaceColor', legendItems{i,1}, 'EdgeColor', 'none');
    text(legAx, 18, yPos, legendItems{i,2}, ...
        'FontSize', 9, 'VerticalAlignment', 'middle');
    yPos = yPos - 22;
end
xlim(legAx, [0, 130]); ylim(legAx, [0, 105]);

% 导入/编辑按钮
importBtn = uibutton(panel, 'push', 'Text', '导入主界面地图', ...
    'Position', [5, 115, 140, 25], ...
    'ButtonPushedFcn', @(~,~) importMapFromMainUI());
clearBtn = uibutton(panel, 'push', 'Text', '随机障碍物', ...
    'Position', [5, 85, 140, 25], ...
    'ButtonPushedFcn', @(~,~) randomObstacles());

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
            % 互斥：关闭其他所有按钮
            allModes = {'start', 'goal', 'obstacle', 'delete'};
            btns = {setStartBtn, setGoalBtn, addObsBtn, delObsBtn};
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
        % 获取点击栅格坐标
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
            case 'obstacle'
                if mapData.occGrid(row, col)
                    mapData.occGrid(row, col) = false;
                else
                    mapData.occGrid(row, col) = true;
                end
                setStatus(sprintf('障碍物切换: (%d,%d)', row, col));
            case 'delete'
                mapData.occGrid(row, col) = false;
                setStatus(sprintf('障碍物删除: (%d,%d)', row, col));
            otherwise
                return;
        end
        onReset();
        drawStaticMap();
    end

    function initEmptyMap(sz)
        mapData.mapSize = sz;
        mapData.occGrid = false(sz, sz);
        mapData.startGrid = [];
        mapData.goalGrid = [];
    end

    function drawStaticMap()
        % 绘制地图底图（障碍物、起点、终点）
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
            h.startPlot = plot(ax, sx, sy, 'o', ...
                'MarkerSize', 10, 'LineWidth', 2, ...
                'MarkerEdgeColor', plotTools('getColor', 'start'), ...
                'MarkerFaceColor', 'none');
        end

        % 终点
        if ~isempty(mapData.goalGrid)
            gy = mapData.goalGrid(1) - 0.5;
            gx = mapData.goalGrid(2) - 0.5;
            h.goalPlot = plot(ax, gx, gy, 'x', ...
                'MarkerSize', 12, 'LineWidth', 3, ...
                'Color', plotTools('getColor', 'goal'));
        end

        % 清除搜索覆盖层句柄
        h.openScatter = [];
        h.closedScatter = [];
        h.currentPlot = [];
        h.pathPlot = [];

        drawnow;
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
        if ~isempty(state.staticObstacles)
            for i = 1:size(state.staticObstacles, 1)
                r = state.staticObstacles(i, 2);  % UI y → row
                c = state.staticObstacles(i, 1);  % UI x → col
                if r >= 1 && r <= mapData.mapSize && ...
                   c >= 1 && c <= mapData.mapSize
                    mapData.occGrid(r, c) = true;
                end
            end
        end

        % 起点：UI (col, row) → [row, col]
        if ~isempty(state.startPoint)
            mapData.startGrid = [state.startPoint(2), state.startPoint(1)];
        elseif ~isempty(state.startPoints)
            mapData.startGrid = [state.startPoints(1, 2), state.startPoints(1, 1)];
        end

        % 终点
        if ~isempty(state.goalPoint)
            mapData.goalGrid = [state.goalPoint(2), state.goalPoint(1)];
        elseif ~isempty(state.goalPoints)
            mapData.goalGrid = [state.goalPoints(1, 2), state.goalPoints(1, 1)];
        end

        onReset();
        drawStaticMap();
        setStatus(sprintf('已导入 %dx%d 地图', mapData.mapSize, mapData.mapSize));
    end

    function randomObstacles()
        % 生成随机障碍物（覆盖约25%的栅格）
        mapData.occGrid = rand(mapData.mapSize, mapData.mapSize) < 0.25;
        % 确保起终点不是障碍物
        if ~isempty(mapData.startGrid)
            mapData.occGrid(mapData.startGrid(1), mapData.startGrid(2)) = false;
        end
        if ~isempty(mapData.goalGrid)
            mapData.occGrid(mapData.goalGrid(1), mapData.goalGrid(2)) = false;
        end
        onReset();
        drawStaticMap();
        setStatus('已生成随机障碍物');
    end

% ================================================================
%                      控制回调
% ================================================================

    function onRun()
        if isempty(mapData.startGrid) || isempty(mapData.goalGrid)
            setStatus('错误: 请先设置起点和终点');
            return;
        end
        if ctrl.running && ~ctrl.paused
            return;  % 已在运行中
        end
        if ctrl.paused
            % 恢复运行
            ctrl.paused = false;
            ctrl.stepRequested = false;
            uiresume(fig);
            return;
        end

        % 全新启动
        ctrl.running = true;
        ctrl.paused = false;
        ctrl.stopRequested = false;
        ctrl.stepRequested = false;
        resetVisualization();
        setStatus('运行中...');
        runAlgorithm();
    end

    function onPause()
        if ctrl.running && ~ctrl.paused
            ctrl.paused = true;
            setStatus('已暂停');
        end
    end

    function onStep()
        if isempty(mapData.startGrid) || isempty(mapData.goalGrid)
            setStatus('错误: 请先设置起点和终点');
            return;
        end
        if ctrl.running && ctrl.paused
            % 暂停中 → 请求单步前进
            ctrl.stepRequested = true;
            uiresume(fig);
        elseif ~ctrl.running
            % 尚未开始 → 启动并暂停（将运行第1步后暂停）
            ctrl.running = true;
            ctrl.paused = true;
            ctrl.stopRequested = false;
            ctrl.stepRequested = true;
            resetVisualization();
            setStatus('单步模式...');
            runAlgorithm();
        else
            % 正在运行 → 切换为暂停
            ctrl.paused = true;
            setStatus('已暂停');
        end
    end

    function onReset()
        ctrl.stopRequested = true;
        ctrl.running = false;
        ctrl.paused = false;
        ctrl.stepRequested = false;
        % 如果在 uiwait 中则唤醒
        try
            uiresume(fig);
        catch
        end
        resetVisualization();
        updateStats(0, 0, 0, []);
        setStatus('已重置');
    end

    function onSpeedChanged(src, ~)
        ctrl.speed = src.Value;
    end

    function onWindowClose(~, ~)
        ctrl.stopRequested = true;
        ctrl.running = false;
        ctrl.paused = false;
        try
            uiresume(fig);
        catch
        end
        delete(fig);
    end

% ================================================================
%                      算法执行
% ================================================================

    function runAlgorithm()
        % 构建 Map 对象
        m = Map(mapData.mapSize);
        [obsR, obsC] = find(mapData.occGrid);
        if ~isempty(obsR)
            m.setStaticObstacle(obsR, obsC);
        end

        algStartTime = tic;

        % 调用 A*（带 callback）
        algoName = algoDD.Value;
        switch algoName
            case 'AStar'
                AStar(m, mapData.startGrid, mapData.goalGrid, 0, @algoCallback);
            case 'Dijkstra'
                Dijkstra(m, mapData.startGrid, mapData.goalGrid, 0, @algoCallback);
            case 'RRT'
                RRT(m, mapData.startGrid, mapData.goalGrid, 0, @algoCallback);
        end

        ctrl.running = false;
    end

    function action = algoCallback(info)
        % AStar/Dijkstra/RRT 的通用回调

        % 处理 'finish' 类型
        if strcmp(info.type, 'finish')
            elapsed = toc(algStartTime);
            updateStats(info.iteration, 0, 0, info.path);
            set(timeLabel, 'Text', sprintf('耗时: %.3fs', elapsed));
            drawFinalPath(info);
            if info.success
                setStatus(sprintf('完成! 路径 %d 点', size(info.path, 1)));
            else
                setStatus('搜索完成: 无可行路径');
            end
            action = 'continue';
            return;
        end

        % 更新统计显示
        nOpen = nnz(info.openSet);
        nClosed = nnz(info.closedSet);
        updateStats(info.iteration, nOpen, nClosed, []);

        % 增量更新可视化
        updateSearchVis(info);

        % 检查停止
        if ctrl.stopRequested
            ctrl.running = false;
            action = 'stop';
            setStatus('已停止');
            return;
        end

        % 单步/暂停控制
        if ctrl.paused && ~ctrl.stepRequested
            setStatus('已暂停 - 点击单步或运行');
            uiwait(fig);  % 等待 uiresume

            % 醒来后重新检查停止标志
            if ctrl.stopRequested
                ctrl.running = false;
                action = 'stop';
                setStatus('已停止');
                return;
            end
        end
        ctrl.stepRequested = false;

        % 如果是单步模式，下一步自动暂停
        if ctrl.paused
            setStatus('单步已完成 - 点击单步或运行');
        end

        % 速度控制
        if ~ctrl.paused && ctrl.speed > 0
            pause(ctrl.speed);
        end
        drawnow;

        action = 'continue';
    end

% ================================================================
%                      可视化更新
% ================================================================

    function resetVisualization()
        % 清除搜索覆盖层
        if ~isempty(h.openScatter) && isvalid(h.openScatter)
            delete(h.openScatter);
        end
        if ~isempty(h.closedScatter) && isvalid(h.closedScatter)
            delete(h.closedScatter);
        end
        if ~isempty(h.currentPlot) && isvalid(h.currentPlot)
            delete(h.currentPlot);
        end
        if ~isempty(h.pathPlot) && isvalid(h.pathPlot)
            delete(h.pathPlot);
        end
        h.openScatter = [];
        h.closedScatter = [];
        h.currentPlot = [];
        h.pathPlot = [];
        prevOpen = [];
        prevClosed = [];
    end

    function updateSearchVis(info)
        % 将栅格 [row, col] 转为连续坐标 [x, y] = [col-0.5, row-0.5]
        [openRows, openCols] = find(info.openSet);
        [closedRows, closedCols] = find(info.closedSet);

        openX = openCols - 0.5;
        openY = openRows - 0.5;
        closedX = closedCols - 0.5;
        closedY = closedRows - 0.5;

        % Open set: 浅绿散点
        if ~isempty(h.openScatter) && isvalid(h.openScatter)
            set(h.openScatter, 'XData', openX, 'YData', openY);
        else
            h.openScatter = scatter(ax, openX, openY, 15, ...
                plotTools('getColor', 'openSet'), 'filled', ...
                'MarkerFaceAlpha', 0.7, ...
                'HitTest', 'off', 'PickableParts', 'none');
        end

        % Closed set: 浅灰散点
        if ~isempty(h.closedScatter) && isvalid(h.closedScatter)
            set(h.closedScatter, 'XData', closedX, 'YData', closedY);
        else
            h.closedScatter = scatter(ax, closedX, closedY, 12, ...
                plotTools('getColor', 'closedSet'), 'filled', ...
                'MarkerFaceAlpha', 0.6, ...
                'HitTest', 'off', 'PickableParts', 'none');
        end

        % 当前节点: 橙色高亮
        curX = info.current(2) - 0.5;
        curY = info.current(1) - 0.5;
        if ~isempty(h.currentPlot) && isvalid(h.currentPlot)
            set(h.currentPlot, 'XData', curX, 'YData', curY);
        else
            h.currentPlot = plot(ax, curX, curY, 's', ...
                'MarkerSize', 12, ...
                'MarkerFaceColor', plotTools('getColor', 'currentNode'), ...
                'MarkerEdgeColor', 'none', ...
                'HitTest', 'off', 'PickableParts', 'none');
        end
    end

    function drawFinalPath(info)
        % 清除搜索覆盖层
        if ~isempty(h.openScatter) && isvalid(h.openScatter)
            delete(h.openScatter);
        end
        if ~isempty(h.closedScatter) && isvalid(h.closedScatter)
            delete(h.closedScatter);
        end
        if ~isempty(h.currentPlot) && isvalid(h.currentPlot)
            delete(h.currentPlot);
        end
        h.openScatter = [];
        h.closedScatter = [];
        h.currentPlot = [];

        % 绘制最终路径
        if ~isempty(info.path) && info.success
            pathX = info.path(:, 2) - 0.5;
            pathY = info.path(:, 1) - 0.5;
            pathColor = plotTools('getColor', 'finalPath');
            h.pathPlot = plot(ax, pathX, pathY, '-', ...
                'Color', pathColor, 'LineWidth', 2.5, ...
                'HitTest', 'off', 'PickableParts', 'none');
        end
        drawnow;
    end

% ================================================================
%                      统计更新
% ================================================================

    function updateStats(iters, nOpen, nClosed, path)
        set(iterLabel, 'Text', sprintf('迭代: %d', iters));
        set(openLabel, 'Text', sprintf('Open集: %d', nOpen));
        set(closedLabel, 'Text', sprintf('已探索: %d', nClosed));
        if ~isempty(path)
            set(pathLabel, 'Text', sprintf('路径长度: %d', size(path, 1)));
        else
            set(pathLabel, 'Text', '路径长度: 无解');
        end
    end

    function setStatus(msg)
        set(statusLabel, 'Text', sprintf('状态: %s', msg));
    end

end
