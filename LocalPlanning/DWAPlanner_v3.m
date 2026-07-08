function [vx, vy, predictTraj] = DWAPlanner_v3(robot, localGoal, map, ~, dt, params)
%DWAPLANNER_V3 局部规划器（DWA + 动态障碍物速度预测 + TTC避碰 + 自适应速度限制）
%   [vx, vy, predictTraj] = DWAPlanner_v3(robot, localGoal, map, ~, dt, params)
%
%   核心改进（相比 V2）：
%     (1) 速度预测 — 从 DynamicObstacle.getVelocity() 获取障碍物速度向量，
%         用 线性预测 替代 V2 的 ping-pong 轨迹模拟
%     (2) TTC 动态威胁项 — 代价函数新增 Time-to-Collision 评价：
%         G = α·heading + β·obstacle + γ·velocity + δ·dynamicThreat + ε·boundary
%         TTC 越小威胁越大，迫使机器人提前避让
%     (3) 自适应速度限制与膨胀半径 —
%         障碍物速度越快，膨胀半径越大（inflationRadius = baseRadius + k·obsSpeed）
%         检测到快速接近时，动态缩小速度窗口
%
%   主要可调参数：
%     动态威胁: wDynThreat(3.0) / ttcWarning(2.0) — TTC 低于此值开始警告
%     膨胀系数: inflationK(0.3) — 障碍物速度对膨胀半径的缩放因子
%     速度限制: speedLimitK(0.5) — TTC 对速度限制的缩放因子
%   params 可选字段: maxSpeed, maxAccel, allRobots, robotIdx

% ===== 可调参数 ============================================================

% --- 采样参数 ---
numSamples   = 7;
predictSteps = 20;

% --- 代价权重 ---
wHeading    = 2.0;
wObstacle   = 2.0;
wBoundary   = 1.0;
wRobot      = 0.8;
wSpeed      = 1.0;
wSmoothness = 0.0;
wDynThreat  = 2.0;   % 动态威胁（TTC）权重

% --- 静态障碍物避碰参数 ---
staticSafeDist    = 0.4;
staticRepulseRange = 1.2;

% --- 动态障碍物避碰参数 ---
dynSafeDist    = 1.0;
dynRepulseRange = 1.8;

% --- 自适应参数 ---
inflationK = 0.3;    % 膨胀半径系数：inflationRadius = baseRadius + k * obsSpeed
ttcWarning = 6.0;    % TTC 低于此值触发自适应速度限制 + 朝向调制（提前反应）
speedLimitK = 0.3;   % 速度限制缩放：speedScale = max(ttc/ttcWarning, speedLimitK)

% --- 边界 / 机器人间距离 ---
boundarySafeDist = 0.5;
robotSafeDist    = 0.8;
robotReactDist   = 3.6;

% --- 传感器参数 ---
sensorRange  = 5;    % 动态障碍物检测范围（栅格数）

% --- 减速参数 ---
decelDist = 1.0;

% --- 局部极小值逃逸参数 ---
stuckWindow         = 15;
stuckThresh         = 0.15;
escapeMinSteps      = 20;
escapeProgressThresh = 0.5;
wEscape             = 4.0;

% --- 速度参数 ---
maxSpeed = 1.0;
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
maxAccel = 2.0;
if isfield(params, 'maxAccel'), maxAccel = params.maxAccel; end
% =========================================================================

% ===== 1. 持久状态管理 =====
persistent stateMap
if isempty(stateMap)
    stateMap = containers.Map('KeyType', 'double', 'ValueType', 'any');
end

robotIdx = 0;
if isfield(params, 'robotIdx'), robotIdx = params.robotIdx; end

if stateMap.isKey(robotIdx)
    st = stateMap(robotIdx);
else
    st = initState(stuckWindow);
end

% ===== 2. 目标到达检查 =====
goalVec = localGoal - robot.pos;
goalDist = norm(goalVec);
if goalDist < 1e-6
    vx = 0; vy = 0; predictTraj = [];
    stateMap(robotIdx) = st;
    return;
end
goalDir = goalVec / goalDist;

% ===== 3. 预计算共享数据 =====
occGrid = map.getLocalOccGrid(robot.pos, sensorRange);
n = map.mapSize;

% 静态障碍物坐标列表
staticGrid = double(map.grid == 1);
[occRows, occCols] = find(staticGrid);
occCoords = [occCols(:) - 0.5, occRows(:) - 0.5];

% ===== 3a. 动态障碍物速度预测 + 自适应参数 =====
dynObs = map.dynamicObstacles;
nDynObs = length(dynObs);

% 预测各动态障碍物轨迹（基于速度向量线性预测）
dynObsTrajs = cell(nDynObs, 1);
dynObsVels = cell(nDynObs, 1);
dynObsSpeeds = zeros(nDynObs, 1);
minTTC = inf;  % 最近碰撞时间

for o = 1:nDynObs
    obs = dynObs(o);
    vel = obs.getVelocity();
    obsSpeed = norm(vel);
    dynObsSpeeds(o) = obsSpeed;
    dynObsVels{o} = vel;

    % 线性预测轨迹：futurePos = currentPos + velocity * t
    traj = zeros(predictSteps, 2);
    for k = 1:predictSteps
        traj(k, :) = obs.currentPos + vel * k * dt;
    end
    dynObsTrajs{o} = traj;

    % 计算 TTC（使用相对速度：机器人+障碍物的接近速度）
    relPos = obs.currentPos - robot.pos;
    relDist = norm(relPos);
    % 相对速度 = 障碍物速度 - 机器人速度
    relVel = vel - robot.vel;
    % 接近速度 = 相对速度在连线方向的分量（正值=接近）
    approachSpeed = -dot(relVel, relPos / max(relDist, 1e-6));
    if approachSpeed > 0.01
        ttc = relDist / approachSpeed;
        if ttc < minTTC
            minTTC = ttc;
        end
    end
end

% 自适应速度限制
effectiveMaxSpeed = maxSpeed;
if minTTC < ttcWarning
    speedScale = max(minTTC / ttcWarning, speedLimitK);
    effectiveMaxSpeed = maxSpeed * speedScale;
end

% 自适应膨胀半径（取所有动态障碍物的最大膨胀）
maxInflation = 0;
for o = 1:nDynObs
    inflation = inflationK * dynObsSpeeds(o);
    if inflation > maxInflation
        maxInflation = inflation;
    end
end
% 动态膨胀应用于动态障碍物安全距离
adaptiveDynSafeDist = dynSafeDist + maxInflation;

% 计算正面威胁因子（障碍物在正前方 → 值小 → 朝向权重大幅降低）
frontalThreatFactor = 1.0;
if minTTC < ttcWarning
    for o = 1:nDynObs
        lateralOffset = abs(dynObs(o).currentPos(1) - robot.pos(1));
        if lateralOffset < 1.5
            frontalThreatFactor = min(frontalThreatFactor, lateralOffset / 1.5);
        end
    end
end

% 收集其他机器人
otherRobots = cell(0, 1);
if isfield(params, 'allRobots') && ~isempty(params.allRobots) ...
        && isfield(params, 'robotIdx') && robotIdx > 0
    allRobots = params.allRobots;
    for j = 1:length(allRobots)
        if j ~= robotIdx
            otherRobots{end+1} = allRobots{j}; %#ok<AGROW>
        end
    end
end

% ===== 4. 卡住检测 =====
st.posHistory(st.posHistIdx, :) = robot.pos;
st.posHistIdx = st.posHistIdx + 1;
if st.posHistIdx > stuckWindow
    st.posHistIdx = 1;
    st.posHistFilled = true;
end

if st.posHistFilled && ~st.escapeActive
    totalDisp = computeRingBufferDisp(st.posHistory);
    if totalDisp < stuckThresh
        st.escapeActive = true;
        st.escapeSteps  = 0;
        st.stuckPos     = robot.pos;
        st.escapeDir    = computeEscapeDir(robot.pos, goalDir, occGrid, n, sensorRange);
    end
end

% ===== 5. 逃逸模式维护 =====
if st.escapeActive
    st.escapeSteps = st.escapeSteps + 1;
    if st.escapeSteps > escapeMinSteps
        progress = norm(robot.pos - st.stuckPos);
        if progress > escapeProgressThresh
            st.escapeActive = false;
        end
    end
end

% ===== 6. 动态窗口构造（使用自适应速度限制）=====
vMin = robot.vel - maxAccel * dt;
vMax = robot.vel + maxAccel * dt;
vMin = max(vMin, -effectiveMaxSpeed);
vMax = min(vMax,  effectiveMaxSpeed);

vxSamples = linspace(vMin(1), vMax(1), numSamples);
vySamples = linspace(vMin(2), vMax(2), numSamples);

% ===== 7. 候选评估主循环 =====
bestCost = inf;
bestVx   = 0;
bestVy   = 0;
bestTraj = zeros(predictSteps, 2);

for ix = 1:numSamples
    for iy = 1:numSamples
        vxCand = vxSamples(ix);
        vyCand = vySamples(iy);
        vCand  = [vxCand, vyCand];
        vMag   = norm(vCand);

        traj = forwardSimulate(robot.pos, vCand, dt, predictSteps);

        cost = 0;

        % 朝向代价（有正面威胁时降低权重，允许更早侧向偏移）
        effectiveWHeading = wHeading;
        if minTTC < ttcWarning
            ttcRatio = minTTC / ttcWarning;  % 1.0→0.0，越小威胁越大
            effectiveWHeading = wHeading * max(ttcRatio * frontalThreatFactor, 0.2);
        end
        cost = cost + effectiveWHeading * headingCost(vCand, goalDir);

        % 障碍物代价（静态 + 动态，使用自适应膨胀半径）
        cost = cost + wObstacle * obstacleCost(traj, occCoords, ...
            staticRepulseRange, staticSafeDist, robot.radius, ...
            dynObsTrajs, dynRepulseRange, adaptiveDynSafeDist);

        % 边界代价
        cost = cost + wBoundary * boundaryCost(traj, n, ...
            boundarySafeDist, robot.radius);

        % 机器人间代价
        cost = cost + wRobot * robotCost(traj, otherRobots, dt, ...
            robotSafeDist, robotReactDist, goalDir);

        % 速度代价（有动态威胁时降低权重，允许自由减速+转向）
        effectiveWSpeed = wSpeed;
        if minTTC < ttcWarning
            effectiveWSpeed = wSpeed * (minTTC / ttcWarning);  % 威胁越大权重越低
        end
        cost = cost + effectiveWSpeed * speedCost(vMag, effectiveMaxSpeed, goalDist, decelDist);

        % 平滑代价
        cost = cost + wSmoothness * smoothnessCost(vCand, robot.vel, maxAccel, dt);

        % ===== 动态威胁代价（TTC）=====
        cost = cost + wDynThreat * dynThreatCost(traj, vCand, dynObsTrajs, ...
            dynObsVels, dt, robot.radius);

        % ===== 滞留惩罚（有威胁时禁止停死）=====
        % 当 TTC < ttcWarning 时，速度越低惩罚越大
        % 迫使机器人保持运动（即使是横向运动），避免"停死等障碍物通过"
        if minTTC < ttcWarning && vMag < effectiveMaxSpeed * 0.5
            stagnationPenalty = (1 - vMag / (effectiveMaxSpeed * 0.5)) * 3.0;
            cost = cost + stagnationPenalty;
        end

        % 逃逸代价
        if st.escapeActive
            cost = cost + wEscape * escapeCost(vCand, st.escapeDir);
        end

        if cost < bestCost
            bestCost = cost;
            bestVx   = vxCand;
            bestVy   = vyCand;
            bestTraj = traj;
        end
    end
end

% ===== 8. 输出 =====
vMag = norm([bestVx, bestVy]);
if vMag > effectiveMaxSpeed
    bestVx = bestVx / vMag * effectiveMaxSpeed;
    bestVy = bestVy / vMag * effectiveMaxSpeed;
end
vx = bestVx;
vy = bestVy;
predictTraj = bestTraj;

% ===== 9. 持久化状态 =====
stateMap(robotIdx) = st;
end

% =========================================================================
%  辅助函数
% =========================================================================

function st = initState(stuckWindow)
st = struct(...
    'posHistory',     zeros(stuckWindow, 2), ...
    'posHistIdx',     1, ...
    'posHistFilled',  false, ...
    'escapeActive',   false, ...
    'escapeSteps',    0, ...
    'escapeDir',      [0, 0], ...
    'stuckPos',       [0, 0]);
end

function totalDisp = computeRingBufferDisp(posHistory)
stuckWindow = size(posHistory, 1);
totalDisp = 0;
for k = 1:(stuckWindow - 1)
    totalDisp = totalDisp + norm(posHistory(k+1, :) - posHistory(k, :));
end
totalDisp = totalDisp + norm(posHistory(1, :) - posHistory(stuckWindow, :));
end

function traj = forwardSimulate(startPos, vel, dt, steps)
traj = zeros(steps, 2);
px = startPos(1);
py = startPos(2);
for k = 1:steps
    px = px + vel(1) * dt;
    py = py + vel(2) * dt;
    traj(k, :) = [px, py];
end
end

% =========================================================================
%  代价函数
% =========================================================================

function cost = headingCost(vCand, goalDir)
vMag = norm(vCand);
if vMag < 1e-6
    cost = 2.0;
else
    vDir = vCand / vMag;
    cost = 1.0 - dot(vDir, goalDir);
end
end

function cost = obstacleCost(traj, occCoords, staticRange, staticSafe, radius, ...
        dynObsTrajs, dynRange, dynSafe)
staticMargin = staticSafe + radius;
dynMargin    = dynSafe + radius;
nSteps = size(traj, 1);
totalPenalty = 0;

for k = 1:nSteps
    p = traj(k, :);

    % 静态障碍物
    minD = minDistToCoords(p, occCoords);
    totalPenalty = totalPenalty + distPenalty(minD, staticMargin, staticRange);

    % 动态障碍物（时空预测碰撞检测）
    for o = 1:length(dynObsTrajs)
        predPos = dynObsTrajs{o}(k, :);
        dDyn = norm(p - predPos);
        totalPenalty = totalPenalty + distPenalty(dDyn, dynMargin, dynRange);
    end
end
cost = totalPenalty / nSteps;
end

function penalty = distPenalty(d, margin, repulseRange)
if d < margin
    penalty = (margin / max(d, 0.01))^2;
elseif d < repulseRange
    alpha = (repulseRange - d) / (repulseRange - margin);
    penalty = alpha;
else
    penalty = 0;
end
end

function cost = dynThreatCost(traj, robotVelCand, dynObsTrajs, dynObsVels, ~, robotRadius)
%   TTC 动态威胁代价（方向+速度感知版）
%   - 只惩罚朝障碍物方向的速度分量（横向绕行零惩罚）
%   - 与速度大小成正比（静止时无威胁，避免局部极小值）
nSteps = size(traj, 1);
nObs = length(dynObsTrajs);
totalThreat = 0;
vMag = norm(robotVelCand);
if vMag < 1e-6
    cost = 0;
    return;
end

for k = 1:nSteps
    robotPos = traj(k, :);

    for o = 1:nObs
        obsPos = dynObsTrajs{o}(k, :);
        relPos = obsPos - robotPos;
        d = norm(relPos);
        obsVel = dynObsVels{o};

        if d < 1e-6, continue; end
        obsToRobotDir = -relPos / d;

        % 朝障碍物方向的速度分量（正值=接近）
        approachComponent = dot(robotVelCand, obsToRobotDir);

        if approachComponent > 0.01
            threatRadius = robotRadius + 0.3 + norm(obsVel) * 0.5;
            if d < threatRadius * 4
                relApproach = approachComponent - dot(obsVel, obsToRobotDir);
                if relApproach > 0.01
                    ttc = d / relApproach;
                    if ttc < 3.0
                        % 威胁 = (1/TTC) × (接近速度比例) × (距离因子)
                        % 接近速度比例：approachComponent/vMag ∈ (0,1]
                        %   横向运动 → 接近0，直冲障碍物 → 接近1
                        approachRatio = approachComponent / vMag;
                        threat = (1.0 / max(ttc, 0.1)) * approachRatio * (threatRadius / max(d, 0.01));
                        totalThreat = totalThreat + threat;
                    end
                end
            end
        end
    end
end
cost = totalThreat / nSteps;
end

function cost = boundaryCost(traj, n, safeDist, radius)
margin = radius + safeDist;
totalPenalty = 0;
for k = 1:size(traj, 1)
    p = traj(k, :);
    d_left   = p(1) - margin;
    d_right  = n - p(1) - margin;
    d_bottom = p(2) - margin;
    d_top    = n - p(2) - margin;
    dists = [d_left, d_right, d_bottom, d_top];
    for d = dists
        if d > 0 && d < safeDist
            totalPenalty = totalPenalty + (safeDist / d)^2;
        elseif d <= 0
            totalPenalty = totalPenalty + 100;
        end
    end
end
cost = totalPenalty / size(traj, 1);
end

function cost = robotCost(traj, otherRobots, dt, safeDist, reactDist, goalDir)
if isempty(otherRobots)
    cost = 0;
    return;
end

predictSteps = size(traj, 1);
startPos = traj(1, :);
vMag = norm(traj(2, :) - traj(1, :)) / max(dt, 1e-6);

numOthers = length(otherRobots);
otherTrajs = cell(numOthers, 1);
for r = 1:numOthers
    otherTrajs{r} = forwardSimulate(otherRobots{r}.pos, ...
        otherRobots{r}.vel, dt, predictSteps);
end

perpLeft = [-goalDir(2), goalDir(1)];
urgentDist = safeDist;
criticalDist = safeDist * 1.0;
sepDist    = safeDist * 0.5;

cost = 0;

for r = 1:numOthers
    otherPos = otherRobots{r}.pos;
    otherVel = otherRobots{r}.vel;
    rel = otherPos - startPos;
    dRel = norm(rel);
    if dRel < 1e-4, continue; end
    relDir = rel / dRel;

    cross_me = goalDir(1) * relDir(2) - goalDir(2) * relDir(1);
    otherSpeed = norm(otherVel);
    if otherSpeed > 0.01
        otherDir = otherVel / otherSpeed;
        cross_other = otherDir(2) * relDir(1) - otherDir(1) * relDir(2);
    else
        cross_other = -cross_me;
    end

    isClassicYield    = (cross_me < -0.05) && (cross_other >  0.05);
    isClassicPriority = (cross_me >  0.05) && (cross_other < -0.05);

    if ~isClassicYield && ~isClassicPriority
        isClassicYield = (startPos(1) + startPos(2)) < (otherPos(1) + otherPos(2));
    end

    minPredDist = inf;
    for k = 1:predictSteps
        d = norm(traj(k, :) - otherTrajs{r}(k, :));
        if d < minPredDist, minPredDist = d; end
    end

    totalLateral = 0;
    for k = 1:predictSteps
        totalLateral = totalLateral + dot(traj(k, :) - startPos, perpLeft);
    end
    avgLateral = totalLateral / predictSteps;

    if dRel < sepDist
        pushDir = -relDir;
        lateralPush = dot(pushDir, perpLeft);
        cost = cost + abs(lateralPush - avgLateral) * 3.0;
    end

    if isClassicYield
        if dRel < reactDist
            proximity = (reactDist - dRel) / (reactDist - safeDist * 0.4);
            proximity = max(0, min(1, proximity));
            cost = cost + vMag * proximity * 5.0;
        end
        if minPredDist < urgentDist
            cost = cost + (urgentDist / max(minPredDist, 0.05)) * 0.8;
            if avgLateral > 0
                cost = cost - avgLateral * 1.0;
            else
                cost = cost + abs(avgLateral) * 2.0;
            end
        end
    else
        if dRel < reactDist && dRel > safeDist * 0.4
            courage = (dRel - safeDist * 0.4) / (reactDist - safeDist * 0.4);
            courage = max(0, min(1, courage));
            cost = cost - vMag * 0.8 * courage;
        end
        if dRel < reactDist
            proximity = (reactDist - dRel) / reactDist;
            if avgLateral < 0
                cost = cost + avgLateral * proximity * 1.5;
            elseif avgLateral > 0
                cost = cost + avgLateral * proximity * 2.0;
            end
        end
        if minPredDist < criticalDist
            cost = cost + (criticalDist / max(minPredDist, 0.05)) * 0.4;
        end
    end
end
end

function cost = speedCost(vMag, maxSpeed, goalDist, decelDist)
if goalDist > decelDist
    desiredSpeed = maxSpeed;
else
    desiredSpeed = maxSpeed * (goalDist / decelDist);
end
cost = abs(vMag - desiredSpeed) / maxSpeed;
end

function cost = smoothnessCost(vCand, vCurrent, maxAccel, dt)
dv = norm(vCand - vCurrent);
maxDV = maxAccel * dt;
if maxDV < 1e-6
    cost = 0;
else
    cost = dv / maxDV;
end
end

function cost = escapeCost(vCand, escapeDir)
vMag = norm(vCand);
if vMag < 1e-6
    cost = 2.0;
else
    vDir = vCand / vMag;
    cost = 1.0 - dot(vDir, escapeDir);
end
end

% =========================================================================
%  距离查询 / 逃逸计算
% =========================================================================

function minD = minDistToCoords(point, occCoords)
if isempty(occCoords)
    minD = inf;
    return;
end
diffs = occCoords - point;
dists = sqrt(diffs(:,1).^2 + diffs(:,2).^2);
minD = min(dists);
end

function escapeDir = computeEscapeDir(robotPos, goalDir, occGrid, n, sensorRange)
nearFieldR = ceil(sensorRange * 0.7);
r0 = round(robotPos(2));
c0 = round(robotPos(1));
obstacleDirs = zeros(0, 2);
for dr = -nearFieldR:nearFieldR
    for dc = -nearFieldR:nearFieldR
        r = r0 + dr;
        c = c0 + dc;
        if r < 1 || r > n || c < 1 || c > n
            continue;
        end
        if occGrid(r, c)
            gx = c - 0.5;
            gy = r - 0.5;
            diff = robotPos - [gx, gy];
            d = norm(diff);
            if d > 1e-4
                obstacleDirs(end+1, :) = diff / d; %#ok<AGROW>
            end
        end
    end
end

if isempty(obstacleDirs)
    tan1 = [-goalDir(2), goalDir(1)];
    tan2 = [goalDir(2), -goalDir(1)];
    if rand() > 0.5
        escapeDir = tan1;
    else
        escapeDir = tan2;
    end
else
    meanObstDir = mean(obstacleDirs, 1);
    meanObstDir = meanObstDir / (norm(meanObstDir) + 1e-6);
    tan1 = [-meanObstDir(2), meanObstDir(1)];
    tan2 = [meanObstDir(2), -meanObstDir(1)];
    if dot(tan1, goalDir) > dot(tan2, goalDir)
        escapeDir = tan1;
    else
        escapeDir = tan2;
    end
end
escapeDir = escapeDir / (norm(escapeDir) + 1e-6);
end
