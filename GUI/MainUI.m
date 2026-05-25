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
    'Items', {'AStar', 'Dijkstra', 'RRT'}, ...
    'Value', 'AStar', 'Position', [120, 398, 100, 22]);

% 局部规划算法
uilabel(ctrlPanel, 'Text', '局部规划算法:', 'Position', [10, 365, 100, 20]);
localAlgoDD = uidropdown(ctrlPanel, ...
    'Items', {'DWA', 'TEB', 'MPC'}, ...
    'Value', 'DWA', 'Position', [120, 363, 100, 22]);

% 后处理选项
simplifyCB = uicheckbox(ctrlPanel, 'Text', '启用拐角裁剪', ...
    'Value', false, 'Position', [10, 330, 130, 20]);
smoothCB = uicheckbox(ctrlPanel, 'Text', '启用路径平滑', ...
    'Value', true, 'Position', [160, 330, 130, 20]);

% 速度设置
uilabel(ctrlPanel, 'Text', '机器人最大速度:', 'Position', [10, 295, 110, 20]);
speedEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 1.0, ...
    'Position', [130, 293, 80, 22]);

uilabel(ctrlPanel, 'Text', '机器人半径:', 'Position', [10, 260, 80, 20]);
radiusEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 0.3, ...
    'Position', [95, 258, 80, 22]);

uilabel(ctrlPanel, 'Text', '每步延迟(秒):', 'Position', [10, 225, 90, 20]);
delayEdit = uieditfield(ctrlPanel, 'numeric', 'Value', 0.02, ...
    'Position', [110, 223, 80, 22]);

% 按钮
startBtn = uibutton(ctrlPanel, 'Text', '开始仿真', ...
    'Position', [10, 160, 150, 40], ...
    'ButtonPushedFcn', @(~,~) onStart());

resetBtn = uibutton(ctrlPanel, 'Text', '重置地图', ...
    'Position', [180, 160, 150, 40], ...
    'ButtonPushedFcn', @(~,~) onReset());

exitBtn = uibutton(ctrlPanel, 'Text', '退出', ...
    'Position', [10, 110, 320, 40], ...
    'ButtonPushedFcn', @(~,~) close(fig));

% 可视化窗口按钮
vizBtn = uibutton(ctrlPanel, 'Text', '打开可视化窗口', ...
    'Position', [10, 60, 320, 40], ...
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
state.dynObsCount = 0;
state.dynStart = [];
state.vizFig = [];
state.vizAxes = [];

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
                state.startPoint = [r, c];
                statusLabel.Text = sprintf('起点已设置: (%d, %d)', r, c);
            case '添加目标点'
                state.targetPoints(end + 1, :) = [r, c];
                statusLabel.Text = sprintf('目标点已添加: (%d, %d)', r, c);
            case '设置终点'
                state.goalPoint = [r, c];
                statusLabel.Text = sprintf('终点已设置: (%d, %d)', r, c);
            case '添加静态障碍'
                state.staticObstacles(end + 1, :) = [r, c];
                statusLabel.Text = sprintf('静态障碍已添加: (%d, %d)', r, c);
            case '添加动态障碍'
                % 第一次点击为起点，第二次为终点
                if isempty(state.dynStart)
                    state.dynStart = [r, c];
                    statusLabel.Text = sprintf('动态障碍起点: (%d, %d)，请点击终点', r, c);
                else
                    state.dynObsCount = state.dynObsCount + 1;
                    state.dynamicObstacleDefs(end + 1, :) = [state.dynStart(1), state.dynStart(2), r, c, 0.25]; %动态障碍物速度
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
        % 只匹配被点击的精确栅格（曼哈顿距离为0）
        % 检查静态障碍
        if ~isempty(state.staticObstacles)
            match = state.staticObstacles(:, 1) == r & state.staticObstacles(:, 2) == c;
            if any(match)
                state.staticObstacles(match, :) = [];
                statusLabel.Text = '静态障碍已删除';
                return;
            end
        end
        % 检查动态障碍（起点或终点）
        if ~isempty(state.dynamicObstacleDefs)
            match = (state.dynamicObstacleDefs(:, 1) == r & state.dynamicObstacleDefs(:, 2) == c) | ...
                    (state.dynamicObstacleDefs(:, 3) == r & state.dynamicObstacleDefs(:, 4) == c);
            if any(match)
                state.dynamicObstacleDefs(match, :) = [];
                state.dynObsCount = state.dynObsCount - sum(match);
                statusLabel.Text = '动态障碍已删除';
                return;
            end
        end
        % 检查目标点
        if ~isempty(state.targetPoints)
            match = state.targetPoints(:, 1) == r & state.targetPoints(:, 2) == c;
            if any(match)
                state.targetPoints(match, :) = [];
                statusLabel.Text = '目标点已删除';
                return;
            end
        end
        % 检查起/终点
        if ~isempty(state.startPoint) && state.startPoint(1) == r && state.startPoint(2) == c
            state.startPoint = [];
            statusLabel.Text = '起点已删除';
            return;
        end
        if ~isempty(state.goalPoint) && state.goalPoint(1) == r && state.goalPoint(2) == c
            state.goalPoint = [];
            statusLabel.Text = '终点已删除';
            return;
        end
        statusLabel.Text = '未找到可删除的对象';
    end

    function refreshMapDisplay()
        cla(mapAxes);
        setupMapDisplay();

        % 绘制静态障碍
        for i = 1:size(state.staticObstacles, 1)
            r = state.staticObstacles(i, 1);
            c = state.staticObstacles(i, 2);
            rectangle(mapAxes, 'Position', [c-1, r-1, 1, 1], ...
                'FaceColor', [0, 0, 0], 'EdgeColor', 'none');
        end

        % 绘制起点
        if ~isempty(state.startPoint)
            plot(mapAxes, state.startPoint(2) - 0.5, state.startPoint(1) - 0.5, ...
                'o', 'MarkerFaceColor', [0, 0.8, 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
        end

        % 绘制目标点
        for i = 1:size(state.targetPoints, 1)
            plot(mapAxes, state.targetPoints(i, 2) - 0.5, state.targetPoints(i, 1) - 0.5, ...
                'o', 'MarkerFaceColor', [0, 0, 1], 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
        end

        % 绘制终点
        if ~isempty(state.goalPoint)
            plot(mapAxes, state.goalPoint(2) - 0.5, state.goalPoint(1) - 0.5, ...
                'o', 'MarkerFaceColor', [1, 0, 0], 'MarkerEdgeColor', 'none', 'MarkerSize', 10);
        end

        % 绘制动态障碍物线段标记
        for i = 1:size(state.dynamicObstacleDefs, 1)
            def = state.dynamicObstacleDefs(i, :);
            plot(mapAxes, [def(2)-0.5, def(4)-0.5], [def(1)-0.5, def(3)-0.5], ...
                'm--', 'LineWidth', 1.5);
            plot(mapAxes, def(2)-0.5, def(1)-0.5, 'ms', 'MarkerSize', 6);
            plot(mapAxes, def(4)-0.5, def(3)-0.5, 'm^', 'MarkerSize', 6);
        end

        % 使所有子图元不拦截点击，确保ButtonDownFcn能正确获取点击坐标
        set(mapAxes.Children, 'PickableParts', 'none');
    end

    function loadPreset(name)
        onReset();
        state.mapSize = 30;
        sz = 30;

        switch name
            case '空白地图'
                % 不做任何事
            case '简单障碍'
                % 几个散落障碍物
                obsList = [6,1; 6,2; 6,3; 6,4; 6,5; 6,6; 6,7; 6,8; 6,9; 6,10; 6,11; 6,12; 6,13; 6,14; 6,15; 6,16; 6,17; 6,18; 6,19; 6,20; 6,21; 6,22; 6,23; 6,24; 6,25; 6,26; 6,27; 6,28; 6,29; 6,30; ...
                           25,1; 25,2; 25,3; 25,4; 25,5; 25,6; 25,7; 25,8; 25,9; 25,10; 25,11; 25,12; 25,13; 25,14; 25,15; 25,16; 25,17; 25,18; 25,19; 25,20; 25,21; 25,22; 25,23; 25,24; 25,25; 25,26; 25,27; 25,28; 25,29; 25,30; 11,5; 11,6; 11,7; 11,8; 11,9; 11,10; 11,11; 12,5; 12,6; 12,7; 12,8; 12,9; 12,10; 12,11; 19,5; 19,6; 19,7; 19,8; 19,9; 19,10; 19,11; 20,5; 20,6; 20,7; 20,8; 20,9; 20,10; 20,11; 10,15; 11,15; 12,15; 13,15; 14,15; 15,15; 16,15; 17,15; 18,15; 19,15; 20,15; 21,15; 10,16; 11,16; 12,16; 13,16; 14,16; 15,16; 16,16; 17,16; 18,16; 19,16; 20,16; 21,16; 11,19; 12,19; 11,20; 12,20; 11,23; 12,23; 11,24; 12,24; 11,27; 12,27; 11,28; 12,28; 19,19; 20,19; 19,20; 20,20; 19,25; 20,25; 19,26; 20,26; 7,1; 24,1; 15,3; 16,3; 7,17; 24,17];
                state.staticObstacles = obsList;

            case '迷宫地图'
                for i = 3:3:27
                    for j = 1:20
                        if mod(floor(i/3), 2) == 1
                            state.staticObstacles(end+1, :) = [i, j];
                        else
                            state.staticObstacles(end+1, :) = [i, j + 10];
                        end
                    end
                end
                state.startPoint = [1, 15];
                state.targetPoints = [15, 5];
                state.goalPoint = [29, 15];
            case '走廊地图'
                for i = 8:12
                    for j = 1:15
                        state.staticObstacles(end+1, :) = [i, j];
                    end
                end
                for i = 18:22
                    for j = 15:30
                        state.staticObstacles(end+1, :) = [i, j];
                    end
                end
                state.startPoint = [2, 2];
                state.targetPoints = [25, 8; 10, 25];
                state.goalPoint = [28, 28];
        end

        mapSizeDropdown.Value = sprintf('%d×%d', state.mapSize, state.mapSize);
        refreshMapDisplay();
        statusLabel.Text = sprintf('已加载预设地图: %s', name);
    end

    function onReset()
        state.staticObstacles = [];
        state.dynamicObstacleDefs = [];
        state.startPoint = [];
        state.targetPoints = [];
        state.goalPoint = [];
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
        if isempty(state.startPoint)
            statusLabel.Text = '错误: 请先设置起点';
            return;
        end
        if isempty(state.goalPoint)
            statusLabel.Text = '错误: 请先设置终点';
            return;
        end

        statusLabel.Text = '正在规划路径...';
        drawnow;

        % 构建 simParams
        simParams = struct();
        simParams.mapSize = state.mapSize;
        simParams.staticObstacles = state.staticObstacles;
        simParams.dynamicObstacleDefs = state.dynamicObstacleDefs;
        simParams.startPoint = state.startPoint;
        simParams.targetPoints = state.targetPoints;
        simParams.goalPoint = state.goalPoint;
        simParams.globalAlgo = globalAlgoDD.Value;
        simParams.localAlgo = localAlgoDD.Value;
        simParams.enableSimplify = simplifyCB.Value;
        simParams.enableSmooth = smoothCB.Value;
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

    function onOpenViz()
        if ~isempty(state.vizFig) && isvalid(state.vizFig)
            figure(state.vizFig);
            return;
        end
        [state.vizFig, state.vizAxes] = VisualizationUI();
    end
end
