function results = SimulationManager(simParams)
%SIMULATIONMANAGER 仿真控制主函数（支持多机器人协同）
%   results = SimulationManager(simParams)
%   simParams 结构体字段:
%     mapSize, staticObstacles, dynamicObstacleDefs
%     startPoint (单机器人) 或 startPoints (多机器人)
%     targetPoints, goalPoint (或 goalPoints 多机器人)
%     globalAlgo, localAlgo, tspAlgo
%     enableSimplify, enableSmooth
%     stepDelay, robotMaxSpeed, robotRadius
%     vizAxes (可选，可视化坐标轴)

% --- 1. 初始化地图 ---
map = Map(simParams.mapSize);

if isfield(simParams, 'staticObstacles') && ~isempty(simParams.staticObstacles)
    map.setStaticObstacle(simParams.staticObstacles(:, 1), simParams.staticObstacles(:, 2));
end

if isfield(simParams, 'dynamicObstacleDefs') && ~isempty(simParams.dynamicObstacleDefs)
    for i = 1:size(simParams.dynamicObstacleDefs, 1)
        def = simParams.dynamicObstacleDefs(i, :);
        obs = DynamicObstacle([def(1), def(2)], [def(3), def(4)], def(5));
        map.setDynamicObstacle(obs);
    end
end

% --- 1c. 临时静态障碍物（转为零速度动态障碍物处理，复用 DWA 动态避障逻辑）---
% tempObstacleDefs: M×3 矩阵，每行 [col, row, detectionRange]（GUI 存储格式）
if isfield(simParams, 'tempObstacleDefs') && ~isempty(simParams.tempObstacleDefs)
    for i = 1:size(simParams.tempObstacleDefs, 1)
        def = simParams.tempObstacleDefs(i, :);
        gridPos = [def(2), def(1)];  % GUI [c,r] → 算法 [row,col]
        % 起点=终点 + 速度0 = 静态不动的"动态障碍物"
        obs = DynamicObstacle(gridPos, gridPos, 0);
        map.setDynamicObstacle(obs);
    end
end

% --- 2. 初始化机器人 ---
% 兼容单/多机器人：startPoints 优先，否则 fallback 到 startPoint
if isfield(simParams, 'startPoints') && ~isempty(simParams.startPoints)
    startPoints = simParams.startPoints;
else
    startPoints = simParams.startPoint;
end
numRobots = size(startPoints, 1);

robots = cell(1, numRobots);
for r = 1:numRobots
    startCont = [startPoints(r, 2) - 0.5, startPoints(r, 1) - 0.5];
    robots{r} = OmnidirectionalRobot(startCont, simParams.robotMaxSpeed, simParams.robotRadius);
end

% --- 3. 任务分配 ---
tStart = tic;
targets = simParams.targetPoints;

% TSP 算法选择（向后兼容：未指定时默认 TSP_GA）
if isfield(simParams, 'tspAlgo') && ~isempty(simParams.tspAlgo)
    tspAlgo = simParams.tspAlgo;
else
    tspAlgo = 'TSP_GA';
end

% 聚类算法选择（向后兼容：未指定时默认 NearestNeighbor）
if isfield(simParams, 'clusterAlgo') && ~isempty(simParams.clusterAlgo)
    clusterAlgo = simParams.clusterAlgo;
else
    clusterAlgo = 'NearestNeighbor';
end

% 构建各机器人终点矩阵 goalPoints (N×2)
if isfield(simParams, 'goalPoints') && ~isempty(simParams.goalPoints)
    goalPoints = simParams.goalPoints;
else
    % 向后兼容：单终点复制给所有机器人
    goalGrid = simParams.goalPoint;
    goalPoints = repmat(goalGrid, numRobots, 1);
end

% 拐角裁剪参数（用于 TSP 成本矩阵计算）
enableSimplify = isfield(simParams, 'enableSimplify') && simParams.enableSimplify;
if enableSimplify
    occGrid = map.getOccupancyGrid();
    safetyMargin = simParams.robotRadius + 0.2;
else
    occGrid = [];
    safetyMargin = 0.4;
end

if numRobots > 1 && size(targets, 1) >= 1
    % 多机器人：最近邻聚类 + 各机器人独立 TSP
    robotTasks = MultiRobotTaskAllocation(startPoints, targets, goalPoints, map, simParams.globalAlgo, tspAlgo, clusterAlgo, enableSimplify, occGrid, safetyMargin);
    tTSP = toc(tStart);
    % 汇总 TSP 成本
    tspCost = sum([robotTasks.tspCost]);
    orderedPoints = [];
    for r = 1:numRobots
        orderedPoints = [orderedPoints; robotTasks(r).orderedPoints];
    end
elseif size(targets, 1) >= 1
    % 单机器人：原有 TSP 流程
    [orderedPoints, segPaths, tspCost] = TSPsolver(...
        startPoints(1, :), targets, goalPoints(1, :), map, simParams.globalAlgo, tspAlgo, enableSimplify, occGrid, safetyMargin);
    tTSP = toc(tStart);
    robotTasks = struct('orderedPoints', orderedPoints, ...
        'segPaths', {segPaths}, 'tspCost', tspCost, ...
        'assignedTargets', targets, 'goalPoint', goalPoints(1, :));
else
    % 无目标点
    tTSP = 0;
    tspCost = 0;
    robotTasks = struct();
    for r = 1:numRobots
        myGoal = goalPoints(r, :);
        robotTasks(r).orderedPoints = [startPoints(r, :); myGoal];
        robotTasks(r).segPaths = {callPlanner(simParams.globalAlgo, map, startPoints(r, :), myGoal)};
        robotTasks(r).tspCost = 0;
        robotTasks(r).assignedTargets = [];
        robotTasks(r).goalPoint = myGoal;
    end
    orderedPoints = [];
    for r = 1:numRobots
        orderedPoints = [orderedPoints; robotTasks(r).orderedPoints];
    end
end

% --- 4. 对各机器人路径做后处理 ---
tPlanStart = tic;
if isempty(occGrid)
    occGrid = map.getOccupancyGrid();
end

totalRawLen = 0;
totalSimpleLen = 0;
totalSmoothLen = 0;

for r = 1:numRobots
    segPaths = robotTasks(r).segPaths;
    allRaw = {}; allSimple = {}; allSmooth = {};
    rawLen = 0; simpleLen = 0; smoothLen = 0;

    for s = 1:length(segPaths)
        rawPath = segPaths{s};
        if isempty(rawPath), continue; end
        allRaw{end + 1} = rawPath;
        rawLen = rawLen + calcPathLen(rawPath);

        simplePath = rawPath;
        if simParams.enableSimplify
            simplePath = SimplifyPath(rawPath, occGrid, map.mapSize, simParams.robotRadius + 0.2);
        end
        allSimple{end + 1} = simplePath;
        simpleLen = simpleLen + calcPathLen(simplePath);

        smoothPathCont = simplePath;
        if simParams.enableSmooth
            smoothPathCont = SmoothPath(simplePath, 10);
        end
        allSmooth{end + 1} = smoothPathCont;
        smoothLen = smoothLen + calcPathLenCont(smoothPathCont);
    end

    robotTasks(r).allRawPaths = allRaw;
    robotTasks(r).allSimplePaths = allSimple;
    robotTasks(r).allSmoothPaths = allSmooth;
    robotTasks(r).rawLen = rawLen;
    robotTasks(r).simpleLen = simpleLen;
    robotTasks(r).smoothLen = smoothLen;

    totalRawLen = totalRawLen + rawLen;
    totalSimpleLen = totalSimpleLen + simpleLen;
    totalSmoothLen = totalSmoothLen + smoothLen;

    % 构建参考轨迹（连续坐标）及段边界索引
    if simParams.enableSmooth
        fullRef = [];
        segBounds = zeros(length(allSmooth), 2);
        idx = 1;
        for s = 1:length(allSmooth)
            nPts = size(allSmooth{s}, 1);
            fullRef = [fullRef; allSmooth{s}];                          %#ok<AGROW>
            segBounds(s, :) = [idx, idx + nPts - 1];
            idx = idx + nPts;
        end
    elseif simParams.enableSimplify
        fullRef = [];
        segBounds = zeros(length(allSimple), 2);
        idx = 1;
        for s = 1:length(allSimple)
            p = allSimple{s};
            pts = [p(:, 2) - 0.5, p(:, 1) - 0.5];
            nPts = size(pts, 1);
            fullRef = [fullRef; pts];                                   %#ok<AGROW>
            segBounds(s, :) = [idx, idx + nPts - 1];
            idx = idx + nPts;
        end
    else
        fullRef = [];
        segBounds = zeros(length(allRaw), 2);
        idx = 1;
        for s = 1:length(allRaw)
            p = allRaw{s};
            pts = [p(:, 2) - 0.5, p(:, 1) - 0.5];
            nPts = size(pts, 1);
            fullRef = [fullRef; pts];                                   %#ok<AGROW>
            segBounds(s, :) = [idx, idx + nPts - 1];
            idx = idx + nPts;
        end
    end
    robotTasks(r).fullRef = fullRef;
    robotTasks(r).segBounds = segBounds;

    % 访问序列（连续坐标）
    visitSeq = robotTasks(r).orderedPoints;
    robotTasks(r).visitContSeq = [visitSeq(:, 2) - 0.5, visitSeq(:, 1) - 0.5];
end
tPlan = toc(tPlanStart);

% --- 5. 可视化初始化 ---
hasViz = isfield(simParams, 'vizAxes') && ~isempty(simParams.vizAxes);
robotHs = gobjects(numRobots, 1);
trajHs = gobjects(numRobots, 1);
predHs = gobjects(numRobots, 1);

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

    % 起点（多机器人用不同颜色）
    for r = 1:numRobots
        color = plotTools('multiRobotColor', r);
        plot(ax, startPoints(r, 2) - 0.5, startPoints(r, 1) - 0.5, 'o', ...
            'MarkerFaceColor', color, 'MarkerEdgeColor', 'none', 'MarkerSize', 8);
        text(ax, startPoints(r, 2) - 0.2, startPoints(r, 1) - 0.2, ...
            sprintf('R%d', r), 'FontSize', 8, 'Color', color, 'FontWeight', 'bold');
    end

    % 目标点
    for t = 1:size(targets, 1)
        plot(ax, targets(t, 2) - 0.5, targets(t, 1) - 0.5, 'o', ...
            'MarkerFaceColor', plotTools('getColor', 'target'), ...
            'MarkerEdgeColor', 'none', 'MarkerSize', 8);
    end

    % 终点（多机器人各自颜色）
    for r = 1:numRobots
        gColor = plotTools('multiRobotColor', r);
        plot(ax, goalPoints(r, 2) - 0.5, goalPoints(r, 1) - 0.5, 'o', ...
            'MarkerFaceColor', gColor, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
        text(ax, goalPoints(r, 2) - 0.2, goalPoints(r, 1) - 0.2, ...
            sprintf('G%d', r), 'FontSize', 8, 'Color', gColor, 'FontWeight', 'bold');
    end

    % 原始路径
    if isfield(simParams, 'showRawPath') && simParams.showRawPath
        for r = 1:numRobots
            for s = 1:length(robotTasks(r).allRawPaths)
                p = robotTasks(r).allRawPaths{s};
                plot(ax, p(:, 2) - 0.5, p(:, 1) - 0.5, '-', ...
                    'Color', plotTools('getColor', 'globalPath'), 'LineWidth', 1);
            end
        end
    end

    % 简化路径
    if simParams.enableSimplify && isfield(simParams, 'showSimplePath') && simParams.showSimplePath
        for r = 1:numRobots
            for s = 1:length(robotTasks(r).allSimplePaths)
                p = robotTasks(r).allSimplePaths{s};
                plot(ax, p(:, 2) - 0.5, p(:, 1) - 0.5, '--', ...
                    'Color', plotTools('getColor', 'simplifiedPath'), 'LineWidth', 1);
            end
        end
    end

    % 平滑路径
    if simParams.enableSmooth && isfield(simParams, 'showSmoothPath') && simParams.showSmoothPath
        for r = 1:numRobots
            for s = 1:length(robotTasks(r).allSmoothPaths)
                p = robotTasks(r).allSmoothPaths{s};
                plot(ax, p(:, 1), p(:, 2), '-', ...
                    'Color', plotTools('getColor', 'smoothPath'), 'LineWidth', 1.5);
            end
        end
    end

    % 动态障碍物（速度=0 的为临时障碍物，显示蓝色）
    dynH = gobjects(length(map.dynamicObstacles), 1);
    for i = 1:length(map.dynamicObstacles)
        obs = map.dynamicObstacles(i);
        if obs.speed == 0
            faceColor = [0.3, 0.5, 1.0];  % 蓝色：临时障碍物
        else
            faceColor = plotTools('getColor', 'dynamicObs');
        end
        dynH(i) = rectangle(ax, 'Position', ...
            [obs.currentPos(1) - 0.5, obs.currentPos(2) - 0.5, 1, 1], ...
            'FaceColor', faceColor, 'EdgeColor', 'none');
    end

    % 机器人标记、轨迹、预测线
    for r = 1:numRobots
        c = plotTools('multiRobotColor', r);
        robotHs(r) = plot(ax, robots{r}.pos(1), robots{r}.pos(2), 'o', ...
            'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 10);
        if isfield(simParams, 'showTraj') && simParams.showTraj
            trajHs(r) = plot(ax, robots{r}.trajectory(:, 1), robots{r}.trajectory(:, 2), '-', ...
                'Color', c, 'LineWidth', 2);
        else
            trajHs(r) = gobjects(0);
        end
        predHs(r) = plot(ax, nan, nan, '--', 'Color', c);
    end

    drawnow;
end

% --- 6. 仿真执行循环 ---
dt = 0.1;
totalTime = 0;
collision = false;
arrivalThreshold = 0.3;
pauseDuration = 1.0;
lookAheadDist = 1.0;
localParams = struct('maxSpeed', simParams.robotMaxSpeed);

% 每机器人状态
robotState = struct();
for r = 1:numRobots
    robotState(r).currentTargetIdx = 1;
    robotState(r).targetReached = false;
    robotState(r).pauseTimer = 0;
    robotState(r).done = false;
    robotState(r).currentSegIdx = 1;     % 当前所在段索引（1-based，映射到 segBounds）
    robotState(r).prevIdx = robotTasks(r).segBounds(1, 1);  % findLookAhead 持久索引（初始化为第一段起始）
    robotState(r).latestPredictTraj = []; % 缓存最新预测轨迹
end

% 速度历史记录（预分配）
maxSteps = ceil(300 / dt);
speedHistory = cell(1, numRobots);
timeHistory = cell(1, numRobots);
for r = 1:numRobots
    speedHistory{r} = zeros(1, maxSteps);
    timeHistory{r} = zeros(1, maxSteps);
end
stepCount = 0;

active = true(1, numRobots);

while any(active)
    % ---- 每机器人控制 ----
    for r = 1:numRobots
        if ~active(r), continue; end
        rs = robotState(r);
        visitSeq = robotTasks(r).visitContSeq;

        if rs.currentTargetIdx > size(visitSeq, 1)
            active(r) = false;
            rs.done = true;
            robotState(r) = rs;
            continue;
        end

        localGoal = visitSeq(rs.currentTargetIdx, :);
        rob = robots{r};
        distToGoal = norm(rob.pos - localGoal);

        % 到达检测
        if distToGoal < arrivalThreshold && ~rs.targetReached
            rs.targetReached = true;
            rs.pauseTimer = 0;
        end

        if rs.targetReached
            rs.pauseTimer = rs.pauseTimer + dt;
            vx = 0; vy = 0;
            rob.vel = [0, 0];
            rs.latestPredictTraj = [];

            if rs.pauseTimer >= pauseDuration
                rs.currentTargetIdx = rs.currentTargetIdx + 1;
                if rs.currentTargetIdx > size(visitSeq, 1)
                    % 已到达终点，立即停用
                    active(r) = false;
                    rs.done = true;
                else
                    % 段追踪：切换到下一段，重置段内索引
                    rs.currentSegIdx = max(1, rs.currentTargetIdx - 1);
                    rs.prevIdx = robotTasks(r).segBounds(rs.currentSegIdx, 1);
                    rs.targetReached = false;
                end
            end
        else
            % 前瞻点（段约束：只在当前段内部搜索，防止路径自交叉时跳到其他段）
            segBounds = robotTasks(r).segBounds;
            searchEnd = segBounds(rs.currentSegIdx, 2);
            fullRef = robotTasks(r).fullRef;

            % 自适应前瞻距离：检测到临时障碍物时缩短
            effectiveLAD = lookAheadDist;
            if ~isempty(map.tempObstacles)
                for ti = 1:length(map.tempObstacles)
                    if map.tempObstacles(ti).isDetected(rob.pos, lookAheadDist + 1)
                        effectiveLAD = max(lookAheadDist * 0.4, 0.8);
                        break;
                    end
                end
            end

            if distToGoal < effectiveLAD + 0.5
                % 接近目标：直接使用目标点作为前瞻，但仍更新 prevIdx
                lookAheadPt = visitSeq(rs.currentTargetIdx, :);
                [~, newIdx] = findLookAheadExt(fullRef, rob.pos, effectiveLAD, rs.prevIdx, searchEnd);
                rs.prevIdx = newIdx;
            else
                [lookAheadPt, newIdx] = findLookAheadExt(fullRef, rob.pos, effectiveLAD, rs.prevIdx, searchEnd);
                rs.prevIdx = newIdx;
            end

            % 局部规划（传递所有机器人信息用于避碰）
            plannerParams = localParams;
            plannerParams.allRobots = robots;
            plannerParams.robotIdx = r;
            plannerParams.fullRef = fullRef;  % 传递完整路径用于偏离惩罚

            switch simParams.localAlgo
                case 'DWA'
                    [vx, vy, predictTraj] = DWAPlanner(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'DWA_v0'
                    [vx, vy, predictTraj] = DWAPlanner_v0(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'DWA_v1'
                    [vx, vy, predictTraj] = DWAPlanner_v1(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'DWA_v2'
                    [vx, vy, predictTraj] = DWAPlanner_v2(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'DWA_v3'
                    [vx, vy, predictTraj] = DWAPlanner_v3(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'TEB'
                    [vx, vy, predictTraj] = TEBPlanner(rob, lookAheadPt, map, [], dt, plannerParams);
                case 'MPC'
                    [vx, vy, predictTraj] = MPCPlanner(rob, lookAheadPt, map, [], dt, plannerParams);
                otherwise
                    [vx, vy, predictTraj] = DWAPlanner(rob, lookAheadPt, map, [], dt, plannerParams);
            end

            rob.applyVelocity(vx, vy, dt);
            rs.latestPredictTraj = predictTraj;
        end

        robotState(r) = rs;
    end

    % ---- 更新动态障碍物 ----
    map.updateDynamicObstacles(dt);

    % ---- 碰撞检测 ----
    for r = 1:numRobots
        if active(r) && checkCollision(robots{r}.pos, robots{r}.radius, map)
            collision = true;
            break;
        end
        % 边界检查
        rob = robots{r};
        if rob.pos(1) < rob.radius || rob.pos(1) > map.mapSize - rob.radius || ...
           rob.pos(2) < rob.radius || rob.pos(2) > map.mapSize - rob.radius
            collision = true;
            break;
        end
    end
    if ~collision && numRobots > 1 && checkRobotRobotCollision(robots)
        collision = true;
    end
    if collision, break; end

    totalTime = totalTime + dt;

    % 记录速度历史
    stepCount = stepCount + 1;
    for r = 1:numRobots
        speedHistory{r}(stepCount) = norm(robots{r}.vel);
        timeHistory{r}(stepCount) = totalTime;
    end

    % ---- 可视化更新 ----
    if hasViz
        for r = 1:numRobots
            rob = robots{r};
            set(robotHs(r), 'XData', rob.pos(1), 'YData', rob.pos(2));
            if ~isempty(trajHs(r))
                set(trajHs(r), 'XData', rob.trajectory(:, 1), ...
                    'YData', rob.trajectory(:, 2));
            end
            pred = robotState(r).latestPredictTraj;
            if ~isempty(pred)
                set(predHs(r), 'XData', pred(:, 1), 'YData', pred(:, 2));
            else
                set(predHs(r), 'XData', nan, 'YData', nan);
            end
        end
        for i = 1:length(map.dynamicObstacles)
            obs = map.dynamicObstacles(i);
            set(dynH(i), 'Position', ...
                [obs.currentPos(1) - 0.5, obs.currentPos(2) - 0.5, 1, 1]);
        end

        drawnow;
        pause(simParams.stepDelay);
    end

    if totalTime > 300
        break;
    end
end

% --- 7. 统计结果 ---
totalDist = 0;
robotDetails = struct();
for r = 1:numRobots
    traj = robots{r}.trajectory;
    dist = 0;
    if size(traj, 1) > 1
        for i = 2:size(traj, 1)
            dist = dist + norm(traj(i, :) - traj(i - 1, :));
        end
    end
    totalDist = totalDist + dist;
    robotDetails(r).totalDistance = dist;
    robotDetails(r).visitOrder = robotTasks(r).orderedPoints;
    robotDetails(r).assignedTargets = robotTasks(r).assignedTargets;
    robotDetails(r).goalPoint = robotTasks(r).goalPoint;
    robotDetails(r).trajectory = traj;
    robotDetails(r).rawLen = robotTasks(r).rawLen;
    robotDetails(r).simpleLen = robotTasks(r).simpleLen;
    robotDetails(r).smoothLen = robotTasks(r).smoothLen;
    robotDetails(r).speedHistory = speedHistory{r}(1:stepCount);
    robotDetails(r).timeHistory = timeHistory{r}(1:stepCount);
end

% 汇总全部路径用于向后兼容的字段
allRawPaths = {}; allSimplePaths = {}; allSmoothPaths = {};
allTrajectories = {};
for r = 1:numRobots
    allRawPaths = [allRawPaths, robotTasks(r).allRawPaths];
    allSimplePaths = [allSimplePaths, robotTasks(r).allSimplePaths];
    allSmoothPaths = [allSmoothPaths, robotTasks(r).allSmoothPaths];
    allTrajectories{r} = robots{r}.trajectory;
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
    'robotTrajectory', robots{1}.trajectory, ...  % 向后兼容
    'allRawPaths', {allRawPaths}, ...
    'allSimplePaths', {allSimplePaths}, ...
    'allSmoothPaths', {allSmoothPaths}, ...
    'robotDetails', robotDetails, ...
    'numRobots', numRobots);
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

function [pt, newIdx] = findLookAheadExt(refPath, robotPos, lookAheadDist, prevIdx, searchEnd)
    % 前瞻点查找（无持久变量版本，支持多机器人，支持段窗口约束）
    %   searchEnd: 可选，搜索上限索引（默认 refPath 末尾）。
    %              传入当前段的 endIdx，防止路径自交叉时跳到其他段。
    if nargin < 5
        searchEnd = size(refPath, 1);
    end
    if size(refPath, 1) < 2
        pt = robotPos;
        newIdx = 1;
        return;
    end

    startIdx = min(prevIdx, searchEnd);
    minDist = inf;
    closestIdx = startIdx;
    for i = startIdx:searchEnd
        d = norm(refPath(i, :) - robotPos);
        if d < minDist
            minDist = d;
            closestIdx = i;
        end
    end

    for i = closestIdx:searchEnd
        if norm(refPath(i, :) - robotPos) >= lookAheadDist
            pt = refPath(i, :);
            newIdx = closestIdx;
            return;
        end
    end
    pt = refPath(searchEnd, :);
    newIdx = closestIdx;
end

function collision = checkRobotRobotCollision(robots)
    collision = false;
    n = length(robots);
    for i = 1:n
        for j = i + 1:n
            dist = norm(robots{i}.pos - robots{j}.pos);
            if dist < robots{i}.radius + robots{j}.radius
                collision = true;
                return;
            end
        end
    end
end

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
