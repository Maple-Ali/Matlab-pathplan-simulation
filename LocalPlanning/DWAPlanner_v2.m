function [vx, vy, predictTraj] = DWAPlanner_v2(robot, localGoal, map, ~, dt, params)
%DWAPLANNER_V2 局部规划器（DWA 速度采样 + 代价评估 + 局部极小值逃逸）
%   [vx, vy, predictTraj] = DWAPlanner_v2(robot, localGoal, map, ~, dt, params)
%
%   核心改进（相比 V1 的势场力合成法）：
%     (1) 多轨迹采样 + 代价评估 — 在动态窗口内采样候选速度，
%         每个候选前向模拟轨迹，加权代价函数评分选最优
%     (2) 局部极小值逃逸 — 检测卡住状态，触发沿墙绕行模式
%
%   V1 特性保留（从力向量映射为代价项）：
%     - 障碍物/边界/机器人间距离 → 各代价项中的距离惩罚
%     - 侧向避让 → 绕行轨迹自然得低障碍分（隐式）
%     - 接近减速 → speedCost 中 desiredSpeed 随距离缩放
%     - 加速度限幅 → 动态窗口边界
%
%   params 可选字段: maxSpeed, maxAccel, allRobots, robotIdx

% ===== 可调参数 ============================================================

% --- 采样参数 ---
numSamples   = 7;    % 每速度轴采样数（总数 = 7×7 = 49）
predictSteps = 10;   % 每条候选轨迹的前向模拟步数（dt=0.1时覆盖2.5单位，
                     %   确保追上 repulseRange，障碍物在可视范围内）

% --- 代价权重 ---
wHeading    = 2.0;   % 朝向目标对齐权重
wObstacle   = 2.0;   % 障碍物避碰权重（安全项中最高）
wBoundary   = 1.0;   % 边界距离权重
wRobot      = 0.8;   % 机器人间距离权重（看到危险后反应强烈程度）
wSpeed      = 1.0;   % 速度偏好权重
wSmoothness = 0.0;   % 速度平滑权重。置零原因：动态窗口 v∈[vel±maxAccel×dt]
                     %   已硬约束加速度，加上此项会导致速度Cost（~0.5v）
                     %   被平滑Cost（~1.5v）压制，机器人拒绝加速

% --- 安全距离阈值 ---
obsSafeDist      = 0.5;  % 期望障碍物距离（低于此值代价激增）
boundarySafeDist = 0.5;  % 期望边界距离
robotSafeDist    = 0.8;  % 期望机器人间距离（大于障碍物安全距）

% --- 传感器参数 ---
sensorRange  = 4;    % 障碍物搜索半径（栅格数）
repulseRange = 2.5;  % 斥力作用范围（连续单位），障碍物在此范围内产生渐进代价
                     %   与 V1 的 repulseRange 一致，保证提前预警距离

% --- 减速参数（V1 保留）---
decelDist = 1.5;    % 接近目标此距离内开始线性减速

% --- 局部极小值逃逸参数 ---
stuckWindow         = 15;   % 卡住检测窗口（步数）
stuckThresh         = 0.15; % 窗口内累计位移低于此值判定为卡住
escapeMinSteps      = 20;   % 最小逃逸持续步数（防止模式震荡）
escapeProgressThresh = 0.5;  % 距卡住位置位移超过此值可退出逃逸
wEscape             = 4.0;  % 逃逸方向对齐权重（覆盖正常朝向代价）

% --- 速度参数（可从 params 覆盖）---
maxSpeed = 1.0;
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
maxAccel = 2.0;
if isfield(params, 'maxAccel'), maxAccel = params.maxAccel; end
% =========================================================================

% ===== 1. 持久状态管理（按 robotIdx 隔离，参照 MPC 模式）=====
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
occGrid = map.getOccupancyGrid();
n = map.mapSize;

% 预计算占用栅格连续坐标列表（避免 obstacleCost 内重复遍历网格）
[occRows, occCols] = find(occGrid);
occCoords = [occCols(:) - 0.5, occRows(:) - 0.5];  % N×2 [x, y] 连续坐标

% 收集其他机器人的位置和速度（用于运动预测）
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
% 更新位置环形缓冲区
st.posHistory(st.posHistIdx, :) = robot.pos;
st.posHistIdx = st.posHistIdx + 1;
if st.posHistIdx > stuckWindow
    st.posHistIdx = 1;
    st.posHistFilled = true;
end

% 检测卡住
if st.posHistFilled && ~st.escapeActive
    totalDisp = computeRingBufferDisp(st.posHistory);
    if totalDisp < stuckThresh
        % 触发逃逸模式
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

% ===== 6. 动态窗口构造 =====
vMin = robot.vel - maxAccel * dt;
vMax = robot.vel + maxAccel * dt;
vMin = max(vMin, -maxSpeed);
vMax = min(vMax,  maxSpeed);

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

        % 7a. 前向模拟轨迹
        traj = forwardSimulate(robot.pos, vCand, dt, predictSteps);

        % 7b. 逐项计算代价
        cost = 0;

        % 朝向代价
        cost = cost + wHeading * headingCost(vCand, goalDir);

        % 障碍物代价
        cost = cost + wObstacle * obstacleCost(traj, occCoords, ...
            repulseRange, obsSafeDist, robot.radius);

        % 边界代价
        cost = cost + wBoundary * boundaryCost(traj, n, ...
            boundarySafeDist, robot.radius);

        % 机器人间代价（传入速度用于运动预测 + 方向用于对称打破）
        cost = cost + wRobot * robotCost(traj, otherRobots, dt, ...
            robotSafeDist, goalDir);

        % 速度代价
        cost = cost + wSpeed * speedCost(vMag, maxSpeed, goalDist, decelDist);

        % 平滑代价
        cost = cost + wSmoothness * smoothnessCost(vCand, robot.vel, maxAccel, dt);

        % 逃逸代价（仅逃逸模式）
        if st.escapeActive
            cost = cost + wEscape * escapeCost(vCand, st.escapeDir);
        end

        % 7c. 保留最优
        if cost < bestCost
            bestCost = cost;
            bestVx   = vxCand;
            bestVy   = vyCand;
            bestTraj = traj;
        end
    end
end

% ===== 8. 输出 =====
% 矢量模限幅（与V1一致）：逐分量限幅无法保证 |v|≤maxSpeed
vMag = norm([bestVx, bestVy]);
if vMag > maxSpeed
    bestVx = bestVx / vMag * maxSpeed;
    bestVy = bestVy / vMag * maxSpeed;
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
% 初始化持久状态结构体
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
% 计算环形缓冲区中连续点之间的累计位移
stuckWindow = size(posHistory, 1);
totalDisp = 0;
for k = 1:(stuckWindow - 1)
    totalDisp = totalDisp + norm(posHistory(k+1, :) - posHistory(k, :));
end
% 环形连接（最新点到最早点）
totalDisp = totalDisp + norm(posHistory(1, :) - posHistory(stuckWindow, :));
end

function traj = forwardSimulate(startPos, vel, dt, steps)
% 匀速前向模拟轨迹
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
% 朝向对齐代价：速度方向与目标方向偏差
vMag = norm(vCand);
if vMag < 1e-6
    cost = 2.0;  % 静止无方向性，给最大惩罚
else
    vDir = vCand / vMag;
    cost = 1.0 - dot(vDir, goalDir);  % [0, 2]，0=完美对齐
end
end

function cost = obstacleCost(traj, occCoords, repulseRange, safeDist, radius)
% 障碍物避碰代价（渐进式）
%   硬危险区（d < margin）：二次惩罚 (margin/d)²
%   软警戒区（margin ≤ d < repulseRange）：线性衰减至0
%   范围外（d ≥ repulseRange）：无代价
%   与 V1 的 1/d² 斥力语义一致——远距离提供提前预警，近距离强力排斥
margin = safeDist + radius;
totalPenalty = 0;
for k = 1:size(traj, 1)
    p = traj(k, :);
    minD = minDistToCoords(p, occCoords);
    if minD < margin
        % 硬危险区：二次惩罚（与 V1 1/d² 力等效）
        totalPenalty = totalPenalty + (margin / max(minD, 0.01))^2;
    elseif minD < repulseRange
        % 软警戒区：线性衰减 [1, 0]，确保在 margin 处连续
        alpha = (repulseRange - minD) / (repulseRange - margin);
        totalPenalty = totalPenalty + alpha;
    end
end
cost = totalPenalty / size(traj, 1);  % 归一化
end

function cost = boundaryCost(traj, n, safeDist, radius)
% 边界距离代价：四边距离惩罚
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
            totalPenalty = totalPenalty + 100;  % 严重越界
        end
    end
end
cost = totalPenalty / size(traj, 1);
end

function cost = robotCost(traj, otherRobots, dt, safeDist, goalDir)
% 机器人间避碰代价（预测运动 + 线性代价 + 路径偏离 + 右舷让行）
%   - 预测它机未来位置（匀速假设）
%   - 线性代价 safeDist/d（非二次），避免 6400× 悬崖导致转向绝对优先
%   - 嵌路径偏离项：偏离目标线越多代价越高，鼓励减速而非绕行
%   - 右舷规则打破对称：它机在右侧→让行（惩罚速度）
if isempty(otherRobots)
    cost = 0;
    return;
end
predictSteps = size(traj, 1);

% 预计算所有它机的未来轨迹
numOthers = length(otherRobots);
otherTrajs = cell(numOthers, 1);
for r = 1:numOthers
    otherTrajs{r} = forwardSimulate(otherRobots{r}.pos, ...
        otherRobots{r}.vel, dt, predictSteps);
end

% 沿轨迹计算最接近点代价（线性，非二次）
maxPenalty = 0;
minPredictedDist = inf;
for k = 1:predictSteps
    p = traj(k, :);
    for r = 1:numOthers
        otherP = otherTrajs{r}(k, :);
        d = norm(p - otherP);
        if d < minPredictedDist
            minPredictedDist = d;
        end
        if d < safeDist
            penalty = safeDist / max(d, 0.01);  % 线性（非二次），d=0.01→80×
            if penalty > maxPenalty
                maxPenalty = penalty;
            end
        end
    end
end
cost = maxPenalty;

% ---- 右舷让行 + 路径偏离（仅碰撞风险时激活）----
if minPredictedDist < safeDist * 2.0
    % 路径偏离项：轨迹各点偏离 start→goal 直线的平均距离
    startPos = traj(1, :);
    totalLateral = 0;
    for k = 1:predictSteps
        p = traj(k, :);
        vec = p - startPos;
        projDist = dot(vec, goalDir);
        if projDist > 0
            lateral = norm(vec - goalDir * projDist);
        else
            lateral = norm(vec);
        end
        totalLateral = totalLateral + lateral;
    end
    avgLateral = totalLateral / predictSteps;
    cost = cost + avgLateral * 2.0;  % 偏离代价：鼓励沿路径减速而非绕行

    % 右舷让行规则
    yieldPenalty = 0;
    for r = 1:numOthers
        rel = otherRobots{r}.pos - startPos;
        dRel = norm(rel);
        if dRel < 1e-4, continue; end
        relDir = rel / dRel;
        crossVal = goalDir(1) * relDir(2) - goalDir(2) * relDir(1);
        if crossVal < -0.05  % 它机在右侧 → 自身让行
            vMag = norm(traj(2, :) - traj(1, :)) / max(dt, 1e-6);
            yieldPenalty = max(yieldPenalty, vMag * 1.0);  % 增至 1.0
        end
    end
    cost = cost + yieldPenalty;
end
end

function cost = speedCost(vMag, maxSpeed, goalDist, decelDist)
% 速度偏好代价：与期望速度的偏差（期望速度在近目标时降低）
if goalDist > decelDist
    desiredSpeed = maxSpeed;
else
    desiredSpeed = maxSpeed * (goalDist / decelDist);
end
cost = abs(vMag - desiredSpeed) / maxSpeed;  % [0, 1]
end

function cost = smoothnessCost(vCand, vCurrent, maxAccel, dt)
% 速度平滑代价：与当前速度的偏差
dv = norm(vCand - vCurrent);
maxDV = maxAccel * dt;
if maxDV < 1e-6
    cost = 0;
else
    cost = dv / maxDV;  % 0=无变化，1=最大允许变化
end
end

function cost = escapeCost(vCand, escapeDir)
% 逃逸方向对齐代价
vMag = norm(vCand);
if vMag < 1e-6
    cost = 2.0;
else
    vDir = vCand / vMag;
    cost = 1.0 - dot(vDir, escapeDir);  % [0, 2]，0=完美对齐逃逸方向
end
end

% =========================================================================
%  距离查询 / 逃逸计算
% =========================================================================

function minD = minDistToCoords(point, occCoords)
% 计算点到最近障碍物的距离（使用预计算的占用栅格坐标列表）
if isempty(occCoords)
    minD = inf;
    return;
end
diffs = occCoords - point;              % N×2 差值矩阵
dists = sqrt(diffs(:,1).^2 + diffs(:,2).^2);  % 向量化距离计算
minD = min(dists);
end

function escapeDir = computeEscapeDir(robotPos, goalDir, occGrid, n, sensorRange)
% 计算逃逸方向：障碍物主导方向的切向，偏向目标侧
% 收集近场障碍物方向
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
            diff = robotPos - [gx, gy];  % 从障碍物指向机器人
            d = norm(diff);
            if d > 1e-4
                obstacleDirs(end+1, :) = diff / d; %#ok<AGROW>
            end
        end
    end
end

if isempty(obstacleDirs)
    % 无近场障碍：目标方向的垂直方向（随机选一侧）
    tan1 = [-goalDir(2), goalDir(1)];
    tan2 = [goalDir(2), -goalDir(1)];
    % 随机选择一侧
    if rand() > 0.5
        escapeDir = tan1;
    else
        escapeDir = tan2;
    end
else
    % 平均障碍物方向
    meanObstDir = mean(obstacleDirs, 1);
    meanObstDir = meanObstDir / (norm(meanObstDir) + 1e-6);

    % 两个切向
    tan1 = [-meanObstDir(2), meanObstDir(1)];
    tan2 = [meanObstDir(2), -meanObstDir(1)];

    % 选与目标方向投影更大的切向
    if dot(tan1, goalDir) > dot(tan2, goalDir)
        escapeDir = tan1;
    else
        escapeDir = tan2;
    end
end
escapeDir = escapeDir / (norm(escapeDir) + 1e-6);
end
