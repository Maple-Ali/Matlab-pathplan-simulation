function [vx, vy, predictTraj] = DWAPlanner_v2(robot, localGoal, map, ~, dt, params)
%DWAPLANNER_V2 局部规划器（DWA 速度采样 + 代价评估 + 局部极小值逃逸）
%   [vx, vy, predictTraj] = DWAPlanner_v2(robot, localGoal, map, ~, dt, params)
%
%   核心改进（相比 V1 的势场力合成法）：
%     (1) 多轨迹采样 + 代价评估 — 在动态窗口内采样候选速度，
%         每个候选前向模拟轨迹，加权代价函数评分选最优
%     (2) 局部极小值逃逸 — 检测卡住状态，触发沿墙绕行模式
%     (3) 动静态障碍物独立参数 — 动态障碍物时空预测 + 更大预警范围
%
%   V1 特性保留（从力向量映射为代价项）：
%     - 障碍物/边界/机器人间距离 → 各代价项中的距离惩罚
%     - 侧向避让 → 绕行轨迹自然得低障碍分（隐式）
%     - 接近减速 → speedCost 中 desiredSpeed 随距离缩放
%     - 加速度限幅 → 动态窗口边界
%
%   主要可调参数（文件顶部参数区块）：
%     静态障碍物: staticSafeDist(0.4) / staticRepulseRange(1.2) — 越小贴得越近
%     动态障碍物: dynSafeDist(0.8) / dynRepulseRange(2.0) — 越大预警越早
%     整体避障强度: wObstacle(2.0) — 越大越保守
%     margin = safeDist + robot.radius(≈0.2)
%   params 可选字段: maxSpeed, maxAccel, allRobots, robotIdx

% ===== 可调参数 ============================================================

% --- 采样参数 ---
numSamples   = 7;    % 每速度轴采样数（总数 = 7×7 = 49）
predictSteps = 20;   % 每条候选轨迹的前向模拟步数（dt=0.1时覆盖2.0单位，
                     %   配合 dynRepulseRange 实现 ~4 单位动态预警距离）

% --- 代价权重 ---
wHeading    = 2.0;   % 朝向目标对齐权重
wObstacle   = 2.0;   % 障碍物避碰权重（安全项中最高）
wBoundary   = 1.0;   % 边界距离权重
wRobot      = 0.8;   % 机器人间距离权重（看到危险后反应强烈程度）
wSpeed      = 1.0;   % 速度偏好权重
wSmoothness = 0.0;   % 速度平滑权重。置零原因：动态窗口 v∈[vel±maxAccel×dt]
                     %   已硬约束加速度，加上此项会导致速度Cost（~0.5v）
                     %   被平滑Cost（~1.5v）压制，机器人拒绝加速

% --- 静态障碍物避碰参数（可独立调整）---
%  调参: 减小 safeDist/repulseRange → 允许贴得更近（窄道通行更激进）
%        增大 safeDist/repulseRange → 更远离静态障碍物
staticSafeDist    = 0.4;  % 静态障碍物期望距离
                          %   margin = 0.4 + robot.radius ≈ 0.6
staticRepulseRange = 1.2; % 静态障碍物斥力作用范围，超过此距离完全无视
                          %   硬危险区(<0.6) → 软警戒区(0.6~1.2) → 安全区(>1.2)

% --- 动态障碍物避碰参数（可独立调整）---
%  调参: 增大 safeDist/repulseRange → 更早预警、更强规避
%        速度越快的动态障碍物，repulseRange 应越大（保证反应时间）
dynSafeDist    = 1.0;  % 动态障碍物期望距离（比静态大：运动物体需提前预警）
                       %   margin = 0.8 + robot.radius ≈ 1.0
dynRepulseRange = 1.8; % 动态障碍物斥力作用范围（比静态大：速度越快范围应越大）
                       %   硬危险区(<1.0) → 软警戒区(1.0~2.0) → 安全区(>2.0)

% --- 边界 / 机器人间距离 ---
boundarySafeDist = 0.5;  % 期望边界距离
robotSafeDist    = 0.8;  % 期望机器人间距离（碰撞安全阈值）
robotReactDist   = 3.6;  % 机器人间让行/优先判定距离（决定多早开始协商）
                          %   增大 → 更早判定谁让谁行，避免十字交叉时的混乱避让
                          %   减小 → 更接近时才判定（更激进）
                          %   典型值: 2.0=保守(原值), 3.5=推荐, 5.0=很远预警

% --- 传感器参数 ---
sensorRange  = 3;    % 障碍物搜索半径（栅格数），用于局部极小值逃逸方向计算

% --- 减速参数（V1 保留）---
decelDist = 1.0;    % 接近目标此距离内开始线性减速

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
occGrid = map.getOccupancyGrid();  % 含静态+动态（用于逃逸方向计算）
n = map.mapSize;

% 预计算静态障碍物栅格连续坐标列表（仅静态，动态障碍物由独立轨迹预测处理）
staticGrid = double(map.grid == 1);
[occRows, occCols] = find(staticGrid);
occCoords = [occCols(:) - 0.5, occRows(:) - 0.5];  % N×2 [x, y] 连续坐标

% 预计算动态障碍物未来轨迹（逐时间步预测位置，用于时空避碰）
dynObs = map.dynamicObstacles;
nDynObs = length(dynObs);
dynObsTrajs = cell(nDynObs, 1);
for o = 1:nDynObs
    dynObsTrajs{o} = predictDynObsTraj(dynObs(o), dt, predictSteps);
end

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

        % 障碍物代价（静态 + 动态，各自独立参数）
        cost = cost + wObstacle * obstacleCost(traj, occCoords, ...
            staticRepulseRange, staticSafeDist, robot.radius, ...
            dynObsTrajs, dynRepulseRange, dynSafeDist);

        % 边界代价
        cost = cost + wBoundary * boundaryCost(traj, n, ...
            boundarySafeDist, robot.radius);

        % 机器人间代价（传入速度用于运动预测 + 方向用于对称打破）
        cost = cost + wRobot * robotCost(traj, otherRobots, dt, ...
            robotSafeDist, robotReactDist, goalDir);

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

function cost = obstacleCost(traj, occCoords, staticRange, staticSafe, radius, ...
        dynObsTrajs, dynRange, dynSafe)
% 障碍物避碰代价（静态 + 动态障碍物时空预测，各自独立参数）
%   - 静态障碍物：各轨迹点检查与占栅格最近距离，用 staticRange/staticSafe
%   - 动态障碍物：各轨迹点 k 检查与动态障碍物在时刻 k*dt 的预测位置距离，用 dynRange/dynSafe
%   硬危险区（d < margin）：二次惩罚 (margin/d)²
%   软警戒区（margin ≤ d < repulseRange）：线性衰减至0
%   范围外（d ≥ repulseRange）：无代价
staticMargin = staticSafe + radius;
dynMargin    = dynSafe + radius;
nSteps = size(traj, 1);
totalPenalty = 0;

for k = 1:nSteps
    p = traj(k, :);

    % 静态障碍物（使用静态参数：允许较近距离通过）
    minD = minDistToCoords(p, occCoords);
    totalPenalty = totalPenalty + distPenalty(minD, staticMargin, staticRange);

    % 动态障碍物（使用动态参数：更大的预警范围和期望距离）
    for o = 1:length(dynObsTrajs)
        predPos = dynObsTrajs{o}(k, :);
        dDyn = norm(p - predPos);
        totalPenalty = totalPenalty + distPenalty(dDyn, dynMargin, dynRange);
    end
end
cost = totalPenalty / nSteps;  % 归一化
end

function penalty = distPenalty(d, margin, repulseRange)
% 单点距离→代价映射
if d < margin
    % 硬危险区：二次惩罚（与 V1 1/d² 力等效）
    penalty = (margin / max(d, 0.01))^2;
elseif d < repulseRange
    % 软警戒区：线性衰减 [1, 0]，确保在 margin 处连续
    alpha = (repulseRange - d) / (repulseRange - margin);
    penalty = alpha;
else
    penalty = 0;
end
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

function cost = robotCost(traj, otherRobots, dt, safeDist, reactDist, goalDir)
% 机器人间避碰代价（互验左右 + 哈希破缺 + 紧急分离）
%   解决纯自我中心左右判定的对称性破缺问题。
%
%   互验左右判定：
%     cross_me  = goalDir × relDir        → 它机在我哪侧
%     cross_other = otherDir × (-relDir)  → 我在它机哪侧（用其速度方向近似）
%     经典非对称 → 按左右规则正常处理
%     模糊（both-right/both-left）→ 位置哈希破缺保证恰好一车让行
%
%   紧急分离：当前距离 < sepDist 时强制侧向远离，不受左右分类约束
%
%   静止机器人 fallback：otherSpeed < 0.01 时退化为 -cross_me
%
%   参数说明：
%     safeDist  — 碰撞安全距离，控制紧急避碰阈值
%     reactDist — 让行/优先判定距离，控制多早开始协商（增大=更早判定）
if isempty(otherRobots)
    cost = 0;
    return;
end

predictSteps = size(traj, 1);
startPos = traj(1, :);

% 候选速度大小
vMag = norm(traj(2, :) - traj(1, :)) / max(dt, 1e-6);

% 预计算所有它机未来轨迹
numOthers = length(otherRobots);
otherTrajs = cell(numOthers, 1);
for r = 1:numOthers
    otherTrajs{r} = forwardSimulate(otherRobots{r}.pos, ...
        otherRobots{r}.vel, dt, predictSteps);
end

% 垂直于前进方向的单位向量（指向左侧）
perpLeft = [-goalDir(2), goalDir(1)];

% 距离阈值（由显式参数控制）
urgentDist = safeDist;                % 紧急碰撞距离
criticalDist = safeDist * 1.0;        % 左侧车避让阈值
sepDist    = safeDist * 0.5;          % 强制分离距离

cost = 0;

for r = 1:numOthers
    otherPos = otherRobots{r}.pos;
    otherVel = otherRobots{r}.vel;
    rel = otherPos - startPos;
    dRel = norm(rel);
    if dRel < 1e-4, continue; end
    relDir = rel / dRel;

    % ===== 互验左右判定 =====
    % cross_me < 0 → 它机在我右侧（我该让行）
    cross_me = goalDir(1) * relDir(2) - goalDir(2) * relDir(1);

    % cross_other < 0 → 我在它机右侧（它也该让行）
    otherSpeed = norm(otherVel);
    if otherSpeed > 0.01
        otherDir = otherVel / otherSpeed;
        % 从它机视角：rel_other = -relDir
        % cross_other = otherDir × (-relDir) = otherDir(2)*relDir(1) - otherDir(1)*relDir(2)
        cross_other = otherDir(2) * relDir(1) - otherDir(1) * relDir(2);
    else
        % 静止机器人 fallback：强制与 cross_me 反号，走经典非对称
        cross_other = -cross_me;
    end

    % 四种情况分类：
    %   cross_me<0 & cross_other>0  → 经典：我让行，它优先
    %   cross_me>0 & cross_other<0  → 经典：我优先，它让行
    %   cross_me<0 & cross_other<0  → 模糊 both-right（双方都判定让行）
    %   cross_me>0 & cross_other>0  → 模糊 both-left （双方都判定优先）
    isClassicYield    = (cross_me < -0.05) && (cross_other >  0.05);
    isClassicPriority = (cross_me >  0.05) && (cross_other < -0.05);

    % 模糊情况：位置哈希破缺 — 全局一致地决定谁让行
    if ~isClassicYield && ~isClassicPriority
        % 比较两车绝对位置之和（双方视角结果相反，保证唯一性）
        % 只设 isClassicYield：true=让行，false=优先（由后续else分支处理）
        isClassicYield = (startPos(1) + startPos(2)) < (otherPos(1) + otherPos(2));
    end

    % 计算与该它机预测轨迹的最小距离
    minPredDist = inf;
    for k = 1:predictSteps
        d = norm(traj(k, :) - otherTrajs{r}(k, :));
        if d < minPredDist, minPredDist = d; end
    end

    % 计算轨迹相对于前进方向的平均侧向偏移
    %   avgLateral > 0: 偏左, avgLateral < 0: 偏右
    totalLateral = 0;
    for k = 1:predictSteps
        totalLateral = totalLateral + dot(traj(k, :) - startPos, perpLeft);
    end
    avgLateral = totalLateral / predictSteps;

    % ===== 紧急分离（最后防线，不受左右分类约束）=====
    %   当前距离低于 sepDist 时强制侧向远离，解决两车低速接近时无绕行动作的问题
    if dRel < sepDist
        pushDir = -relDir;                       % 远离它机的方向
        lateralPush = dot(pushDir, perpLeft);    % 排斥方向的侧向分量
        cost = cost + abs(lateralPush - avgLateral) * 3.0;  % 鼓励匹配排斥方向
    end

    if isClassicYield
        % ============================================================
        %  让行模式：减速优先 + 紧急绕行
        % ============================================================

        % ① 减速代价（主导）：距离越近减速越强
        %     proximity: 0 at reactDist → 1 at safeDist*0.4
        if dRel < reactDist
            proximity = (reactDist - dRel) / (reactDist - safeDist * 0.4);
            proximity = max(0, min(1, proximity));
            cost = cost + vMag * proximity * 5.0;
        end

        % ② 紧急绕行（辅助）：仅预测碰撞迫近时激活
        if minPredDist < urgentDist
            cost = cost + (urgentDist / max(minPredDist, 0.05)) * 0.8;
            if avgLateral > 0
                cost = cost - avgLateral * 1.0;   % 偏左（远离右侧车）
            else
                cost = cost + abs(avgLateral) * 2.0; % 偏右（靠近右侧车）
            end
        end

    else
        % ============================================================
        %  优先模式：保持速度 + 适度避让
        % ============================================================

        % ① 鼓励保持速度（我有优先权），距离越远越有信心快速通过
        %    越近勇气越低：full at reactDist → 0 at safeDist*0.4
        if dRel < reactDist && dRel > safeDist * 0.4
            courage = (dRel - safeDist * 0.4) / (reactDist - safeDist * 0.4);
            courage = max(0, min(1, courage));
            cost = cost - vMag * 0.8 * courage;
        end

        % ② 连续侧向引导（基于当前距离渐变，无阈值悬崖）
        %    用 dRel 连续渐变 — 越近引导越强，远距离引导趋零。
        %    机器人无需偏航回避阈值，headingCost 自然拉回直行。
        if dRel < reactDist
            proximity = (reactDist - dRel) / reactDist;  % [0,1]，0 at reactDist
            if avgLateral < 0
                cost = cost + avgLateral * proximity * 1.5;   % 偏右（远离）奖励
            elseif avgLateral > 0
                cost = cost + avgLateral * proximity * 2.0;   % 偏左（靠近）惩罚
            end
        end

        % ③ 紧急碰撞保护（仅极近时激活，配合连续引导）
        if minPredDist < criticalDist
            cost = cost + (criticalDist / max(minPredDist, 0.05)) * 0.4;
        end
    end
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

function traj = predictDynObsTraj(obs, dt, steps)
% 预测动态障碍物未来 steps 步的轨迹（不修改障碍物状态）
%   模拟 DynamicObstacle.update() 的运动逻辑（线段往复）
%   返回 steps×2 轨迹坐标
startCont = [obs.startPos(2) - 0.5, obs.startPos(1) - 0.5];
endCont   = [obs.endPos(2) - 0.5, obs.endPos(1) - 0.5];
dirVec = endCont - startCont;
segLen = norm(dirVec);
if segLen < 1e-6
    traj = repmat(obs.currentPos, steps, 1);
    return;
end
unitDir = dirVec / segLen;

curPos = obs.currentPos;
curDir = obs.direction;
stepDist = obs.speed * dt;
traj = zeros(steps, 2);

for k = 1:steps
    remaining = stepDist;
    while remaining > 0
        if curDir > 0
            distToEnd = dot(endCont - curPos, unitDir);
        else
            distToEnd = dot(curPos - startCont, unitDir);
        end
        if remaining <= distToEnd
            curPos = curPos + unitDir * curDir * remaining;
            remaining = 0;
        else
            curPos = curPos + unitDir * curDir * distToEnd;
            remaining = remaining - distToEnd;
            curDir = -curDir;  % 到达端点，反弹
        end
    end
    traj(k, :) = curPos;
end
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
