function results = SimulationManager(simParams)
%SIMULATIONMANAGER 仿真控制主函数
%   results = SimulationManager(simParams)
%   simParams 结构体字段:
%     mapSize, staticObstacles, dynamicObstacleDefs
%     startPoint, targetPoints, goalPoint
%     globalAlgo, localAlgo
%     enableSimplify, enableSmooth
%     stepDelay, robotMaxSpeed, robotRadius
%     vizAxes (可选，可视化坐标轴)

% --- 1. 初始化地图 ---
map = Map(simParams.mapSize);

% 静态障碍物
if isfield(simParams, 'staticObstacles') && ~isempty(simParams.staticObstacles)
    map.setStaticObstacle(simParams.staticObstacles(:, 1), simParams.staticObstacles(:, 2));
end

% 动态障碍物
if isfield(simParams, 'dynamicObstacleDefs') && ~isempty(simParams.dynamicObstacleDefs)
    for i = 1:size(simParams.dynamicObstacleDefs, 1)
        def = simParams.dynamicObstacleDefs(i, :);
        obs = DynamicObstacle([def(1), def(2)], [def(3), def(4)], def(5));
        map.setDynamicObstacle(obs);
    end
end

% --- 2. 初始化机器人 ---
startCont = [simParams.startPoint(2) - 0.5, simParams.startPoint(1) - 0.5];
robot = OmnidirectionalRobot(startCont, simParams.robotMaxSpeed, simParams.robotRadius);

% --- 3. TSP 多目标排序 ---
tStart = tic;
targets = simParams.targetPoints;
goalGrid = simParams.goalPoint;
startGrid = simParams.startPoint;

if size(targets, 1) >= 1
    [orderedPoints, segPaths, tspCost] = TSPsolver(...
        startGrid, targets, goalGrid, map, simParams.globalAlgo);
    tTSP = toc(tStart);
else
    orderedPoints = [startGrid; goalGrid];
    segPaths = {AStar(map, startGrid, goalGrid, 0)};
    tTSP = 0;
end

% --- 4. 对各段路径做后处理 ---
tPlanStart = tic;
allRawPaths = {};
allSimplePaths = {};
allSmoothPaths = {};
totalRawLen = 0;
totalSimpleLen = 0;
totalSmoothLen = 0;

occGrid = map.getOccupancyGrid();

for s = 1:length(segPaths)
    rawPath = segPaths{s};
    if isempty(rawPath), continue; end
    allRawPaths{end + 1} = rawPath;
    totalRawLen = totalRawLen + calcPathLen(rawPath);

    simplePath = rawPath;
    if simParams.enableSimplify
        simplePath = SimplifyPath(rawPath, occGrid, map.mapSize);
    end
    allSimplePaths{end + 1} = simplePath;
    totalSimpleLen = totalSimpleLen + calcPathLen(simplePath);

    smoothPathCont = simplePath;
    if simParams.enableSmooth
        smoothPathCont = SmoothPath(simplePath, 10);
    end
    allSmoothPaths{end + 1} = smoothPathCont;
    totalSmoothLen = totalSmoothLen + calcPathLenCont(smoothPathCont);
end
tPlan = toc(tPlanStart);

% --- 5. 可视化：绘制静态地图和规划路径 ---
hasViz = isfield(simParams, 'vizAxes') && ~isempty(simParams.vizAxes);
if hasViz
    ax = simParams.vizAxes;
    plotTools('setupAxes', ax, map.mapSize);

    % 静态障碍物
    [sr, sc] = find(map.grid == 1);
    for i = 1:length(sr)
        x = sc(i) - 1; y = sr(i) - 1;
        rectangle(ax, 'Position', [x, y, 1, 1], ...
            'FaceColor', plotTools('getColor', 'staticObs'), 'EdgeColor', 'none');
    end

    % 起点、目标点、终点
    plot(ax, startGrid(2) - 0.5, startGrid(1) - 0.5, 'o', ...
        'MarkerFaceColor', plotTools('getColor', 'start'), ...
        'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    for t = 1:size(targets, 1)
        plot(ax, targets(t, 2) - 0.5, targets(t, 1) - 0.5, 'o', ...
            'MarkerFaceColor', plotTools('getColor', 'target'), ...
            'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    end
    plot(ax, goalGrid(2) - 0.5, goalGrid(1) - 0.5, 'o', ...
        'MarkerFaceColor', plotTools('getColor', 'goal'), ...
        'MarkerEdgeColor', 'none', 'MarkerSize', 8);

    % 原始路径（红色）
    for s = 1:length(allRawPaths)
        p = allRawPaths{s};
        plot(ax, p(:, 2) - 0.5, p(:, 1) - 0.5, '-', ...
            'Color', plotTools('getColor', 'globalPath'), 'LineWidth', 1);
    end

    % 简化路径（蓝色）
    if simParams.enableSimplify
        for s = 1:length(allSimplePaths)
            p = allSimplePaths{s};
            plot(ax, p(:, 2) - 0.5, p(:, 1) - 0.5, '--', ...
                'Color', plotTools('getColor', 'simplifiedPath'), 'LineWidth', 1);
        end
    end

    % 平滑路径（绿色）
    if simParams.enableSmooth
        for s = 1:length(allSmoothPaths)
            p = allSmoothPaths{s};
            plot(ax, p(:, 1), p(:, 2), '-', ...
                'Color', plotTools('getColor', 'smoothPath'), 'LineWidth', 1.5);
        end
    end

    % 动态障碍物初始位置标记
    dynH = gobjects(length(map.dynamicObstacles), 1);
    for i = 1:length(map.dynamicObstacles)
        obs = map.dynamicObstacles(i);
        dynH(i) = rectangle(ax, 'Position', ...
            [obs.currentPos(1) - 0.5, obs.currentPos(2) - 0.5, 1, 1], ...
            'FaceColor', plotTools('getColor', 'dynamicObs'), 'EdgeColor', 'none');
    end

    % 机器人标记
    robotH = plot(ax, robot.pos(1), robot.pos(2), 'o', ...
        'MarkerFaceColor', plotTools('getColor', 'robot'), ...
        'MarkerEdgeColor', 'k', 'MarkerSize', 10);

    % 实际轨迹
    trajH = plot(ax, robot.trajectory(:, 1), robot.trajectory(:, 2), '-', ...
        'Color', plotTools('getColor', 'actualTraj'), 'LineWidth', 2);

    % 预测轨迹
    predH = plot(ax, nan, nan, '--', 'Color', plotTools('getColor', 'predictTraj'));

    drawnow;
end

% --- 6. 仿真执行循环 ---
dt = 0.1;  % 控制周期
totalTime = 0;
collision = false;
arrivalThreshold = 0.15;  % 到达判定阈值（靠近栅格中心）

% 局部规划器参数
localParams = struct('maxSpeed', simParams.robotMaxSpeed);

% 构建完整参考轨迹（串联所有平滑路径段）
if simParams.enableSmooth
    fullRef = [];
    for s = 1:length(allSmoothPaths)
        fullRef = [fullRef; allSmoothPaths{s}];
    end
elseif simParams.enableSimplify
    fullRef = [];
    for s = 1:length(allSimplePaths)
        p = allSimplePaths{s};
        fullRef = [fullRef; p(:, 2) - 0.5, p(:, 1) - 0.5];
    end
else
    fullRef = [];
    for s = 1:length(allRawPaths)
        p = allRawPaths{s};
        fullRef = [fullRef; p(:, 2) - 0.5, p(:, 1) - 0.5];
    end
end

% 访问序列（连续坐标）
visitSequence = orderedPoints;
visitContSeq = [visitSequence(:, 2) - 0.5, visitSequence(:, 1) - 0.5];
currentTargetIdx = 1;
targetReached = false;
pauseTimer = 0;
pauseDuration = 1.0;  % 到达目标后停留1秒

lookAheadDist = 1.0;  % 前瞻距离

while currentTargetIdx <= size(visitContSeq, 1)
    localGoal = visitContSeq(currentTargetIdx, :);

    % 检查是否到达当前目标
    distToGoal = norm(robot.pos - localGoal);
    if distToGoal < arrivalThreshold && ~targetReached
        targetReached = true;
        pauseTimer = 0;
    end

    % 到达后停留
    if targetReached
        pauseTimer = pauseTimer + dt;
        vx = 0; vy = 0;
        robot.vel = [0, 0];
        predictTraj = [];

        if pauseTimer >= pauseDuration
            currentTargetIdx = currentTargetIdx + 1;
            targetReached = false;
        end
    else
        % 前瞻点：在参考路径上找前向点
        % 接近目标时直接用目标点
if distToGoal < lookAheadDist + 0.5
    lookAheadPt = visitContSeq(currentTargetIdx, :);
else
    lookAheadPt = findLookAhead(fullRef, robot.pos, lookAheadDist);
end

        % 调用局部规划器
        switch simParams.localAlgo
            case 'DWA'
                [vx, vy, predictTraj] = DWAPlanner(robot, lookAheadPt, map, [], dt, localParams);
            case 'TEB'
                [vx, vy, predictTraj] = TEBPlanner(robot, lookAheadPt, map, [], dt, localParams);
            case 'MPC'
                [vx, vy, predictTraj] = MPCPlanner(robot, lookAheadPt, map, [], dt, localParams);
            otherwise
                [vx, vy, predictTraj] = DWAPlanner(robot, lookAheadPt, map, [], dt, localParams);
        end

        % 机器人执行速度
        robot.applyVelocity(vx, vy, dt);
    end

    % 更新动态障碍物
    map.updateDynamicObstacles(dt);

    % 碰撞检测
    if checkCollision(robot.pos, robot.radius, map)
        collision = true;
        break;
    end

    % 边界检查
    if robot.pos(1) < robot.radius || robot.pos(1) > map.mapSize - robot.radius || ...
       robot.pos(2) < robot.radius || robot.pos(2) > map.mapSize - robot.radius
        collision = true;
        break;
    end

    totalTime = totalTime + dt;

    % 可视化更新
    if hasViz
        % 机器人
        set(robotH, 'XData', robot.pos(1), 'YData', robot.pos(2));
        % 轨迹
        set(trajH, 'XData', robot.trajectory(:, 1), ...
            'YData', robot.trajectory(:, 2));
        % 动态障碍物
        for i = 1:length(map.dynamicObstacles)
            obs = map.dynamicObstacles(i);
            set(dynH(i), 'Position', ...
                [obs.currentPos(1) - 0.5, obs.currentPos(2) - 0.5, 1, 1]);
        end
        % 预测轨迹
        if ~isempty(predictTraj)
            set(predH, 'XData', predictTraj(:, 1), 'YData', predictTraj(:, 2));
        else
            set(predH, 'XData', nan, 'YData', nan);
        end

        drawnow;
        pause(simParams.stepDelay);
    end

    % 仿真时间上限
    if totalTime > 300
        break;
    end
end

% --- 7. 统计结果 ---
totalDist = 0;
if size(robot.trajectory, 1) > 1
    traj = robot.trajectory;
    for i = 2:size(traj, 1)
        totalDist = totalDist + norm(traj(i, :) - traj(i - 1, :));
    end
end

results = struct(...
    'totalTime', totalTime, ...
    'totalDistance', totalDist, ...
    'collision', collision, ...
    'tspTime', tTSP, ...
    'tspCost', tspCost, ...
    'planTime', tPlan, ...
    'rawPathLen', totalRawLen, ...
    'simplePathLen', totalSimpleLen, ...
    'smoothPathLen', totalSmoothLen, ...
    'visitOrder', orderedPoints, ...
    'robotTrajectory', robot.trajectory, ...
    'allRawPaths', {allRawPaths}, ...
    'allSimplePaths', {allSimplePaths}, ...
    'allSmoothPaths', {allSmoothPaths});
end

% -------------------------------------------------------------------------
function len = calcPathLen(path)
    len = 0;
    for i = 2:size(path, 1)
        len = len + norm(path(i, :) - path(i - 1, :));
    end
end

function len = calcPathLenCont(path)
    len = 0;
    for i = 2:size(path, 1)
        len = len + norm(path(i, :) - path(i - 1, :));
    end
end

function pt = findLookAhead(refPath, robotPos, lookAheadDist)
    if size(refPath, 1) < 2
        pt = robotPos;
        return;
    end
    % 先找路径上距离机器人最近的点
    minDist = inf;
    closestIdx = 1;
    for i = 1:size(refPath, 1)
        d = norm(refPath(i, :) - robotPos);
        if d < minDist
            minDist = d;
            closestIdx = i;
        end
    end
    % 从最近点向前找距离 >= lookAheadDist 的点
    for i = closestIdx:size(refPath, 1)
        if norm(refPath(i, :) - robotPos) >= lookAheadDist
            pt = refPath(i, :);
            return;
        end
    end
    pt = refPath(end, :);
end
