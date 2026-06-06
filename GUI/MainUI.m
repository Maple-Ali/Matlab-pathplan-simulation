function MainUI()
%MAINUI 主界面
%   地图编辑 + 控制面板 + 仿真启动

% --- 创建主窗口 ---
fig = uifigure('Name', '全向移动机器人多目标自主导航仿真系统', ...
    'Position', [100, 100, 1100, 650], 'Resize', 'off');

% --- 左侧：地图编辑区 ---
mapPanel = uipanel(fig, 'Title', '地图编辑区', ...
    'Position', [10, 10, 530, 630]);

mapAxes = uiaxes(mapPanel, 'Position', [10, 10, 510, 510]);
title(mapAxes, '栅格地图');
xlabel(mapAxes, 'X (列)');
ylabel(mapAxes, 'Y (行)');

% --- 编辑模式按钮组 ---
modeGroup = uibuttongroup(fig, 'Title', '编辑模式', ...
    'Position', [550, 500, 180, 140]);

modes = {'设置起点', '添加目标点', '设置终点', '添加静态障碍', '添加动态障碍', '删除对象'};
modeTags = {'start', 'target', 'goal', 'static', 'dynamic', 'delete'};
modeBtns = gobjects(6, 1);
for i = 1:6
    modeBtns(i) = uiradiobutton(modeGroup, 'Text', modes{i}, ...
        'Position', [10, 120 - i * 20, 160, 18]);
end
modeBtns(4).Value = true;  % 默认选择添加静态障碍

% --- 地图设置 ---
mapSettingsPanel = uipanel(fig, 'Title', '地图设置', ...
    'Position', [550, 420, 180, 70]);
uilabel(mapSettingsPanel, 'Text', '大小:', 'Position', [10, 20, 40, 20]);
mapSizeDropdown = uidropdown(mapSettingsPanel, ...
    'Items', {'20×20', '30×30', '40×40', '50×50'}, ...
    'Value', '30×30', 'Position', [55, 18, 100, 22]);

% --- 预设地图 ---
presetPanel = uipanel(fig, 'Title', '预设地图', ...
    'Position', [550, 340, 180, 70]);
presetDropdown = uidropdown(presetPanel, ...
    'Items', {'空白地图', '简单障碍', '迷宫地图', '走廊地图'}, ...
    'Value', '简单障碍', 'Position', [10, 18, 160, 22]);

% --- 右侧：控制面板 ---
ctrlPanel = uipanel(fig, 'Title', '控制面板', ...
    'Position', [740, 200, 350, 440]);

% 全局规划算法
uilabel(ctrlPanel, 'Text', '全局规划算法:', 'Position', [10, 400, 100, 20]);
globalAlgoDD = uidropdown(ctrlPanel, ...
    'Items', {'AStar', 'AStar_v1', 'AStar_v2', 'AStar_v3', 'Dijkstra', 'RRT'}, ...
    'Value', 'AStar', 'Position', [120, 398, 100, 22]);

% 局部规划算法
uilabel(ctrlPanel, 'Text', '局部规划算法:', 'Position', [10, 365, 100, 20]);
localAlgoDD = uidropdown(ctrlPanel, ...
    'Items', {'DWA', 'DWA_v0', 'DWA_v1', 'DWA_v2', 'TEB', 'MPC'}, ...
    'Value', 'DWA', 'Position', [120, 363, 100, 22]);

% TSP 求解算法
uilabel(ctrlPanel, 'Text', 'TSP 求解算法:', 'Position', [10, 333, 100, 20]);
tspAlgoDD = uidropdown(ctrlPanel, ...
    'Items', getTSPAlgoList(), ...
    'Value', 'TSP_GA', 'Position', [120, 331, 100, 22]);

% 后处理选项
simplifyCB = uicheckbox(ctrlPanel, 'Text', '启用拐角裁剪', ...
    'Value', false, 'Position', [10, 298, 130, 20]);
smoothCB = uicheckbox(ctrlPanel, 'Text', '启用路径平滑', ...
    'Value', true, 'Position', [160, 298, 130, 20]);

% 速度设置
uilabel(ctrlPanel, 'Text', '机器人最大速度:', 'Position', [10, 263, 110, 20]);
speedEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 1.0, ...
    'Position', [130, 261, 80, 22]);

uilabel(ctrlPanel, 'Text', '机器人半径:', 'Position', [10, 228, 80, 20]);
radiusEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 0.3, ...
    'Position', [95, 226, 80, 22]);

uilabel(ctrlPanel, 'Text', '每步延迟(秒):', 'Position', [10, 193, 90, 20]);
delayEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 0.02, ...
    'Position', [110, 191, 80, 22]);

% 机器人数量
uilabel(ctrlPanel, 'Text', '机器人数量:', 'Position', [10, 168, 80, 20]);
robotCountSpinner = uispinner(ctrlPanel, ...
    'Limits', [1, 5], 'Value', 1, 'Step', 1, ...
    'Position', [95, 166, 70, 22], ...
    'ValueChangedFcn', @(~,~) onRobotCountChanged());

% 路径显示选项
uilabel(ctrlPanel, 'Text', '仿真时显示:', 'Position', [10, 138, 80, 20]);
showRawPathCB = uicheckbox(ctrlPanel, 'Text', '全局规划原始路径', ...
    'Value', false, 'Position', [10, 116, 160, 20]);
showSimplePathCB = uicheckbox(ctrlPanel, 'Text', '简化后全局路径', ...
    'Value', false, 'Position', [180, 116, 160, 20]);
showSmoothPathCB = uicheckbox(ctrlPanel, 'Text', '平滑后路径', ...
    'Value', true, 'Position', [10, 94, 160, 20]);
showTrajCB = uicheckbox(ctrlPanel, 'Text', '机器人移动路径', ...
    'Value', true, 'Position', [180, 94, 160, 20]);

% 按钮
startBtn = uibutton(ctrlPanel, 'Text', '开始仿真', ...
    'Position', [10, 50, 150, 36], ...
    'ButtonPushedFcn', @(~,~) onStart());

resetBtn = uibutton(ctrlPanel, 'Text', '重置地图', ...
    'Position', [180, 50, 150, 36], ...
    'ButtonPushedFcn', @(~,~) onReset());

algoTestBtn = uibutton(ctrlPanel, 'Text', '全局规划测试', ...
    'Position', [10, 22, 155, 22], ...
    'ButtonPushedFcn', @(~,~) onOpenAlgoTester());

tspTestBtn = uibutton(ctrlPanel, 'Text', 'TSP算法测试', ...
    'Position', [175, 22, 155, 22], ...
    'ButtonPushedFcn', @(~,~) onOpenTSPTester());

exitBtn = uibutton(ctrlPanel, 'Text', '退出', ...
    'Position', [10, 2, 150, 18], ...
    'ButtonPushedFcn', @(~,~) close(fig));

% 可视化窗口按钮
vizBtn = uibutton(ctrlPanel, 'Text', '打开可视化窗口', ...
    'Position', [180, 2, 150, 18], ...
    'ButtonPushedFcn', @(~,~) onOpenViz());

% --- 状态栏 ---
statusLabel = uilabel(fig, 'Text', '就绪 - 请在地图上编辑', ...
    'Position', [10, 640, 1080, 20], 'HorizontalAlignment', 'left');

% --- 内部状态 ---
state = struct();
state.mapSize = 30;
state.staticObstacles = [];
state.dynamicObstacleDefs = [];  % [r1, c1, r2, c2, speed]
state.startPoint = [];
state.targetPoints = [];
state.goalPoint = [];
state.goalPoints = [];
state.goalPointIdx = 1;
state.dynObsCount = 0;
state.dynStart = [];
state.robotCount = 1;
state.startPoints = [];
state.startPointIdx = 1;
state.vizFig = [];
state.vizAxes = [];
state.algoTesterFig = [];
state.tspTesterFig = [];

% 初始化地图显示
setupMapDisplay();
onReset();

% --- 地图点击回调 ---
mapAxes.ButtonDownFcn = @onMapClick;

% --- 预设地图加载回调 ---
presetDropdown.ValueChangedFcn = @(~,~) loadPreset(presetDropdown.Value);

% ========================================================================
    function setupMapDisplay()
        cla(mapAxes);
        hold(mapAxes, 'on');
        axis(mapAxes, 'equal');
        axis(mapAxes, [0, state.mapSize, 0, state.mapSize]);
        grid(mapAxes, 'on');
        set(mapAxes, 'XTick', 0:1:state.mapSize, 'YTick', 0:1:state.mapSize);
        set(mapAxes, 'GridAlpha', 0.3);
        xlabel(mapAxes, 'X (列)');
        ylabel(mapAxes, 'Y (行)');
        title(mapAxes, '栅格地图');
    end

    function onMapClick(~, evt)
        % 优先使用事件中的交点坐标，兼容不同 MATLAB 版本
        if isprop(evt, 'IntersectionPoint') && ~isempty(evt.IntersectionPoint)
            x = evt.IntersectionPoint(1);
            y = evt.IntersectionPoint(2);
        else
            cp = mapAxes.CurrentPoint;
            x = cp(1, 1);
            y = cp(1, 2);
        end
        c = ceil(x);
        r = ceil(y);
        if r < 1 || r > state.mapSize || c < 1 || c > state.mapSize
            return;
        end

        % 获取当前模式
        selectedMode = modeGroup.SelectedObject.Text;

        switch selectedMode
            case '设置起点'
                if state.robotCount == 1
                    state.startPoint = [c, r];
                    statusLabel.Text = sprintf('起点已设置: (%d, %d)', c, r);
                else
                    state.startPoints(state.startPointIdx, :) = [c, r];
                    statusLabel.Text = sprintf('机器人 %d 起点已设置: (%d, %d) [%d/%d]', ...
                        state.startPointIdx, c, r, state.startPointIdx, state.robotCount);
                    if state.startPointIdx < state.robotCount
                        state.startPointIdx = state.startPointIdx + 1;
                    else
                        statusLabel.Text = sprintf('全部 %d 个机器人起点已设置', state.robotCount);
                    end
                end
            case '添加目标点'
                state.targetPoints(end + 1, :) = [c, r];
                statusLabel.Text = sprintf('目标点已添加: (%d, %d)', c, r);
            case '设置终点'
                if state.robotCount == 1
                    state.goalPoint = [c, r];
                    statusLabel.Text = sprintf('终点已设置: (%d, %d)', c, r);
                else
                    state.goalPoints(state.goalPointIdx, :) = [c, r];
                    statusLabel.Text = sprintf('机器人 %d 终点已设置: (%d, %d) [%d/%d]', ...
                        state.goalPointIdx, c, r, state.goalPointIdx, state.robotCount);
                    if state.goalPointIdx < state.robotCount
                        state.goalPointIdx = state.goalPointIdx + 1;
                    else
                        statusLabel.Text = sprintf('全部 %d 个机器人终点已设置', state.robotCount);
                    end
                end
            case '添加静态障碍'
                state.staticObstacles(end + 1, :) = [c, r];
                statusLabel.Text = sprintf('静态障碍已添加: (%d, %d)', c, r);
            case '添加动态障碍'
                % 第一次点击为起点，第二次为终点
                if isempty(state.dynStart)
                    state.dynStart = [c, r];
                    statusLabel.Text = sprintf('动态障碍起点: (%d, %d)，请点击终点', c, r);
                else
                    state.dynObsCount = state.dynObsCount + 1;
                    state.dynamicObstacleDefs(end + 1, :) = [state.dynStart(1), state.dynStart(2), c, r, 0.25]; %动态障碍物速度
                    statusLabel.Text = sprintf('动态障碍 %d 已添加', state.dynObsCount);
                    state.dynStart = [];
                end
            case '删除对象'
                % 简化：清除最近的障碍物或目标点
                removeNearest(r, c);
            otherwise
        end
        refreshMapDisplay();
    end

    function removeNearest(r, c)
        % state 以 (x,y) 存储，x=c 为列1，y=r 为列2
        x = c; y = r;
        % 检查静态障碍
        if ~isempty(state.staticObstacles)
            match = state.staticObstacles(:, 1) == x & state.staticObstacles(:, 2) == y;
            if any(match)
                state.staticObstacles(match, :) = [];
                statusLabel.Text = '静态障碍已删除';
                return;
            end
        end
        % 检查动态障碍（起点或终点）
        if ~isempty(state.dynamicObstacleDefs)
            match = (state.dynamicObstacleDefs(:, 1) == x & state.dynamicObstacleDefs(:, 2) == y) | ...
                    (state.dynamicObstacleDefs(:, 3) == x & state.dynamicObstacleDefs(:, 4) == y);
            if any(match)
                state.dynamicObstacleDefs(match, :) = [];
                state.dynObsCount = state.dynObsCount - sum(match);
                statusLabel.Text = '动态障碍已删除';
                return;
            end
        end
        % 检查目标点
        if ~isempty(state.targetPoints)
            match = state.targetPoints(:, 1) == x & state.targetPoints(:, 2) == y;
            if any(match)
                state.targetPoints(match, :) = [];
                statusLabel.Text = '目标点已删除';
                return;
            end
        end
        % 检查起/终点
        if ~isempty(state.startPoint) && state.startPoint(1) == x && state.startPoint(2) == y
            state.startPoint = [];
            statusLabel.Text = '起点已删除';
            return;
        end
        if ~isempty(state.startPoints)
            match = state.startPoints(:, 1) == x & state.startPoints(:, 2) == y;
            if any(match)
                state.startPoints(match, :) = [];
                statusLabel.Text = '机器人起点已删除';
                return;
            end
        end
        if ~isempty(state.goalPoint) && state.goalPoint(1) == x && state.goalPoint(2) == y
            state.goalPoint = [];
            statusLabel.Text = '终点已删除';
            return;
        end
        if ~isempty(state.goalPoints)
            match = state.goalPoints(:, 1) == x & state.goalPoints(:, 2) == y;
            if any(match)
                state.goalPoints(match, :) = [];
                statusLabel.Text = '机器人终点已删除';
                return;
            end
        end
        statusLabel.Text = '未找到可删除的对象';
    end

    function refreshMapDisplay()
        cla(mapAxes);
        setupMapDisplay();

        % state 以 (x,y) 存储，列1=x，列2=y
        % 绘制静态障碍
        for i = 1:size(state.staticObstacles, 1)
            x = state.staticObstacles(i, 1);
            y = state.staticObstacles(i, 2);
            rectangle(mapAxes, 'Position', [x-1, y-1, 1, 1], ...
                'FaceColor', [0, 0, 0], 'EdgeColor', 'none');
        end

        % 绘制起点
        if state.robotCount == 1 && ~isempty(state.startPoint)
            plot(mapAxes, state.startPoint(1) - 0.5, state.startPoint(2) - 0.5, ...
                'o', 'MarkerFaceColor', [0, 0.8, 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
        elseif state.robotCount > 1 && ~isempty(state.startPoints)
            for r = 1:size(state.startPoints, 1)
                color = plotTools('multiRobotColor', r);
                plot(mapAxes, state.startPoints(r, 1) - 0.5, state.startPoints(r, 2) - 0.5, ...
                    'o', 'MarkerFaceColor', color, 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
                text(mapAxes, state.startPoints(r, 1) - 0.2, state.startPoints(r, 2) - 0.2, ...
                    sprintf('R%d', r), 'FontSize', 8, 'Color', color, 'FontWeight', 'bold');
            end
        end

        % 绘制目标点
        for i = 1:size(state.targetPoints, 1)
            plot(mapAxes, state.targetPoints(i, 1) - 0.5, state.targetPoints(i, 2) - 0.5, ...
                'o', 'MarkerFaceColor', [0, 0, 1], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
        end

        % 绘制终点（多机器人各自颜色）
        if state.robotCount == 1 && ~isempty(state.goalPoint)
            plot(mapAxes, state.goalPoint(1) - 0.5, state.goalPoint(2) - 0.5, ...
                'o', 'MarkerFaceColor', [1, 0, 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
        elseif state.robotCount > 1 && ~isempty(state.goalPoints)
            for r = 1:size(state.goalPoints, 1)
                color = plotTools('multiRobotColor', r);
                plot(mapAxes, state.goalPoints(r, 1) - 0.5, state.goalPoints(r, 2) - 0.5, ...
                    'o', 'MarkerFaceColor', color, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
                text(mapAxes, state.goalPoints(r, 1) - 0.2, state.goalPoints(r, 2) - 0.2, ...
                    sprintf('G%d', r), 'FontSize', 8, 'Color', color, 'FontWeight', 'bold');
            end
        end

        % 绘制动态障碍物线段标记 (def = [x1,y1,x2,y2,speed])
        for i = 1:size(state.dynamicObstacleDefs, 1)
            def = state.dynamicObstacleDefs(i, :);
            plot(mapAxes, [def(1)-0.5, def(3)-0.5], [def(2)-0.5, def(4)-0.5], ...
                'm--', 'LineWidth', 1.5);
            plot(mapAxes, def(1)-0.5, def(2)-0.5, 'ms', 'MarkerSize', 6);
            plot(mapAxes, def(3)-0.5, def(4)-0.5, 'm^', 'MarkerSize', 6);
        end

        % 使所有子图元不拦截点击，确保ButtonDownFcn能正确获取点击坐标
        set(mapAxes.Children, 'PickableParts', 'none');
    end

    function loadPreset(name)
        onReset();
        state.mapSize = 30;
        sz = 30;

        % 所有坐标采用 (x,y) 顺序
        switch name
            case '空白地图'
                % 不做任何事
            case '简单障碍'
                % 几个散落障碍物 (x,y)
                obsList = [1,6; 2,6; 3,6; 4,6; 5,6; 6,6; 7,6; 8,6; 9,6; 10,6; 11,6; 12,6; 13,6; 14,6; 15,6; 16,6; 17,6; 18,6; 19,6; 20,6; 21,6; 22,6; 23,6; 24,6; 25,6; 26,6; 27,6; 28,6; 29,6; 30,6; ...
                           1,25; 2,25; 3,25; 4,25; 5,25; 6,25; 7,25; 8,25; 9,25; 10,25; 11,25; 12,25; 13,25; 14,25; 15,25; 16,25; 17,25; 18,25; 19,25; 20,25; 21,25; 22,25; 23,25; 24,25; 25,25; 26,25; 27,25; 28,25; 29,25; 30,25; 5,11; 6,11; 7,11; 8,11; 9,11; 10,11; 11,11; 5,12; 6,12; 7,12; 8,12; 9,12; 10,12; 11,12; 5,19; 6,19; 7,19; 8,19; 9,19; 10,19; 11,19; 5,20; 6,20; 7,20; 8,20; 9,20; 10,20; 11,20; 15,10; 15,11; 15,12; 15,13; 15,14; 15,15; 15,16; 15,17; 15,18; 15,19; 15,20; 15,21; 16,10; 16,11; 16,12; 16,13; 16,14; 16,15; 16,16; 16,17; 16,18; 16,19; 16,20; 16,21; 19,11; 19,12; 20,11; 20,12; 23,11; 23,12; 24,11; 24,12; 27,11; 27,12; 28,11; 28,12; 19,19; 19,20; 20,19; 20,20; 25,19; 25,20; 26,19; 26,20; 1,7; 1,24; 3,15; 3,16; 17,7; 17,24];
                state.staticObstacles = obsList;

            case '迷宫地图'
                for i = 3:3:27
                    for j = 1:20
                        if mod(floor(i/3), 2) == 1
                            state.staticObstacles(end+1, :) = [j, i];
                        else
                            state.staticObstacles(end+1, :) = [j + 10, i];
                        end
                    end
                end
                state.startPoint = [15, 1];
                state.targetPoints = [5, 15];
                state.goalPoint = [15, 29];
            case '走廊地图'
                for i = 8:12
                    for j = 1:15
                        state.staticObstacles(end+1, :) = [j, i];
                    end
                end
                for i = 18:22
                    for j = 15:30
                        state.staticObstacles(end+1, :) = [j, i];
                    end
                end
                state.startPoint = [2, 2];
                state.targetPoints = [8, 25; 25, 10];
                state.goalPoint = [28, 28];
        end

        mapSizeDropdown.Value = sprintf('%d×%d', state.mapSize, state.mapSize);
        refreshMapDisplay();
        statusLabel.Text = sprintf('已加载预设地图: %s', name);
    end

    function onReset()
        % 关闭算法测试窗口
        if ~isempty(state.algoTesterFig) && isvalid(state.algoTesterFig)
            delete(state.algoTesterFig);
            state.algoTesterFig = [];
        end
        % 关闭 TSP 测试窗口
        if ~isempty(state.tspTesterFig) && isvalid(state.tspTesterFig)
            delete(state.tspTesterFig);
            state.tspTesterFig = [];
        end

        state.staticObstacles = [];
        state.dynamicObstacleDefs = [];
        state.startPoint = [];
        state.startPoints = [];
        state.startPointIdx = 1;
        state.targetPoints = [];
        state.goalPoint = [];
        state.goalPoints = [];
        state.goalPointIdx = 1;
        state.dynObsCount = 0;
        try
            state.mapSize = sscanf(mapSizeDropdown.Value, '%d×%d');
            state.mapSize = state.mapSize(1);
        catch
            state.mapSize = 30;
        end
        refreshMapDisplay();
        statusLabel.Text = '地图已重置';
    end

    function onStart()
        if state.robotCount == 1 && isempty(state.startPoint)
            statusLabel.Text = '错误: 请先设置起点';
            return;
        end
        if state.robotCount > 1 && isempty(state.startPoints)
            statusLabel.Text = '错误: 请先设置所有机器人起点';
            return;
        end
        if state.robotCount > 1 && size(state.startPoints, 1) < state.robotCount
            statusLabel.Text = sprintf('错误: 还需设置 %d 个机器人起点', ...
                state.robotCount - size(state.startPoints, 1));
            return;
        end
        if state.robotCount == 1 && isempty(state.goalPoint)
            statusLabel.Text = '错误: 请先设置终点';
            return;
        end
        if state.robotCount > 1 && isempty(state.goalPoints)
            statusLabel.Text = '错误: 请先设置所有机器人终点';
            return;
        end
        if state.robotCount > 1 && size(state.goalPoints, 1) < state.robotCount
            statusLabel.Text = sprintf('错误: 还需设置 %d 个机器人终点', ...
                state.robotCount - size(state.goalPoints, 1));
            return;
        end

        statusLabel.Text = '正在规划路径...';
        drawnow;

        % 构建 simParams（state 用 (x,y)，转为 (row,col) 传算法层）
        simParams = struct();
        simParams.mapSize = state.mapSize;
        if ~isempty(state.staticObstacles)
            simParams.staticObstacles = [state.staticObstacles(:,2), state.staticObstacles(:,1)];
        else
            simParams.staticObstacles = [];
        end
        if ~isempty(state.dynamicObstacleDefs)
            simParams.dynamicObstacleDefs = [state.dynamicObstacleDefs(:,2), state.dynamicObstacleDefs(:,1), ...
                state.dynamicObstacleDefs(:,4), state.dynamicObstacleDefs(:,3), state.dynamicObstacleDefs(:,5)];
        else
            simParams.dynamicObstacleDefs = [];
        end
        if state.robotCount > 1
            simParams.startPoints = [state.startPoints(:,2), state.startPoints(:,1)];
        else
            simParams.startPoint = [state.startPoint(2), state.startPoint(1)];
        end
        if ~isempty(state.targetPoints)
            simParams.targetPoints = [state.targetPoints(:,2), state.targetPoints(:,1)];
        else
            simParams.targetPoints = [];
        end
        if state.robotCount > 1
            simParams.goalPoints = [state.goalPoints(:,2), state.goalPoints(:,1)];
            simParams.goalPoint = simParams.goalPoints(1, :);  % 向后兼容
        else
            simParams.goalPoint = [state.goalPoint(2), state.goalPoint(1)];
        end
        simParams.globalAlgo = globalAlgoDD.Value;
        simParams.localAlgo = localAlgoDD.Value;
        simParams.tspAlgo = tspAlgoDD.Value;
        simParams.enableSimplify = simplifyCB.Value;
        simParams.enableSmooth = smoothCB.Value;
        simParams.showRawPath = showRawPathCB.Value;
        simParams.showSimplePath = showSimplePathCB.Value;
        simParams.showSmoothPath = showSmoothPathCB.Value;
        simParams.showTraj = showTrajCB.Value;
        simParams.stepDelay = delayEdit.Value;
        simParams.robotMaxSpeed = speedEdit.Value;
        simParams.robotRadius = radiusEdit.Value;

        % 关联可视化
        if ~isempty(state.vizFig) && isvalid(state.vizFig)
            simParams.vizAxes = state.vizAxes;
        else
            onOpenViz();
            simParams.vizAxes = state.vizAxes;
        end

        statusLabel.Text = '仿真运行中...';
        drawnow;

        try
            results = SimulationManager(simParams);
            statusLabel.Text = '仿真完成';
            DataViewerUI(results, simParams);
        catch ME
            statusLabel.Text = sprintf('仿真错误: %s', ME.message);
            rethrow(ME);
        end
    end

    function onRobotCountChanged()
        state.robotCount = robotCountSpinner.Value;
        state.startPoints = [];
        state.startPointIdx = 1;
        state.startPoint = [];
        state.goalPoints = [];
        state.goalPointIdx = 1;
        state.goalPoint = [];
        refreshMapDisplay();
        if state.robotCount > 1
            statusLabel.Text = sprintf('已切换为 %d 机器人模式，请逐个设置起点', state.robotCount);
        else
            statusLabel.Text = '已切换为单机器人模式';
        end
    end

    function onOpenAlgoTester()
        if ~isempty(state.algoTesterFig) && isvalid(state.algoTesterFig)
            figure(state.algoTesterFig);
            return;
        end
        try
            state.algoTesterFig = AlgorithmTesterUI(state);
            statusLabel.Text = '算法测试窗口已打开';
        catch ME
            statusLabel.Text = sprintf('打开失败: %s', ME.message);
            fprintf(2, 'AlgorithmTesterUI 错误: %s\n', ME.getReport());
        end
    end

    function onOpenTSPTester()
        if ~isempty(state.tspTesterFig) && isvalid(state.tspTesterFig)
            figure(state.tspTesterFig);
            return;
        end
        try
            state.tspTesterFig = TSPTesterUI(state);
            statusLabel.Text = 'TSP 算法测试窗口已打开';
        catch ME
            statusLabel.Text = sprintf('打开失败: %s', ME.message);
            fprintf(2, 'TSPTesterUI 错误: %s\n', ME.getReport());
        end
    end

    function onOpenViz()
        if ~isempty(state.vizFig) && isvalid(state.vizFig)
            figure(state.vizFig);
            return;
        end
        [state.vizFig, state.vizAxes] = VisualizationUI();
    end
end

% =========================================================================
% 局部函数：自动扫描 TSPOptimization 文件夹获取可用算法列表
% =========================================================================
function algoList = getTSPAlgoList()
    % 扫描 TSPOptimization 文件夹下所有 TSP_*.m 文件
    tspDir = fullfile(fileparts(mfilename('fullpath')), '..', 'TSPOptimization');
    files = dir(fullfile(tspDir, 'TSP_*.m'));
    algoList = cell(1, length(files));
    for i = 1:length(files)
        [~, algoList{i}] = fileparts(files(i).name);
    end
    if isempty(algoList)
        algoList = {'TSP_GA'};  % 兜底默认值
    end
end
