function [vx, vy, predictTraj] = DWAPlanner_v1(robot, localGoal, map, ~, dt, params)
%DWAPLANNER_V1 局部规划器（引力+斥力势场模型，增强版）
%   [vx, vy, predictTraj] = DWAPlanner_v1(robot, localGoal, map, ~, dt, params)
%
%   在原始势场模型基础上新增：
%     (1) 边界斥力 — 靠近地图边缘产生排斥力
%     (2) 侧向避让 — 前方障碍物产生横向推力绕行
%     (3) 机器人间斥力 — 多机器人场景互相排斥 + 对向避让
%     (4) 接近目标减速 — 局部目标附近线性减速
%
%   params 可选字段: maxSpeed, maxAccel, allRobots, robotIdx

% ===== 可调参数 ============================================================
% 引力参数
attrGain  = 1.0;     % 引力增益，增大使机器人更积极靠近目标

% 斥力参数（障碍物）
repulseGain  = 0.5;  % 障碍物斥力增益，增大使机器人更强烈避开障碍
                     %   公式: repulseGain / d^2，d为机器人到障碍物距离
                     %   参考：d=0.5时力=2.0，d=0.7时力≈1.0，d=1.0时力=0.5
                     %   过大会在窄通道中卡住，过小则避障不灵敏
repulseRange = 2.5;  % 斥力作用范围（连续单位），障碍物在此范围内才产生斥力
                     %   增大使机器人提前规避，但过大在复杂环境中难以通行

% 传感器参数
sensorRange  = 4;    % 障碍物搜索半径（栅格数），决定检测多远处的障碍物
                     %   增大可提前发现动态障碍，对避障效果配合 repulseRange 使用

% 边界斥力参数（V1 新增）
boundaryGain   = 0.5;   % 边界斥力增益，增大使机器人更远离边界
boundaryRange  = 0.5;   % 边界斥力作用范围（连续单位），距边界此范围内产生斥力
boundaryMargin = 0.3;   % 边界附加安全距离（叠加 robot.radius）

% 速度参数
maxSpeed = 1.0;      % 机器人最大速度
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
maxAccel = 2.0;      % 最大加速度，限制每步速度变化量
if isfield(params, 'maxAccel'), maxAccel = params.maxAccel; end

% 侧向避让参数（V1 新增）
tangentGain  = 0.8;  % 侧向避让增益，增大使机器人更主动绕行前方障碍
                     %   与径向斥力协同：斥力推开障碍，侧向力让机器人绕过去

% 减速参数（V1 新增）
decelDist = 1.0;     % 接近目标此距离内开始线性减速

% 其他机器人斥力参数（V1 新增）
robotRepulseGain  = 0.5;   % 机器人间斥力增益，需大于障碍物斥力以保证避碰
                            %   碰撞风险高于静态障碍，需要更强排斥
robotRepulseRange = 2.5;   % 机器人间斥力作用范围（连续单位）
                            %   比障碍物斥力范围大，提前规避对向/侧向机器人
% =========================================================================

occGrid = map.getOccupancyGrid();
n = map.mapSize;

% ---- 引力：指向局部目标 ----
goalVec = localGoal - robot.pos;
goalDist = norm(goalVec);
if goalDist < 1e-6
    vx = 0; vy = 0; predictTraj = []; return;
end
attrDir = goalVec / goalDist;

% 接近目标时减速（线性缩放：距离 decelDist 内 scale = goalDist/decelDist）[V1 新增]
speedScale = min(1.0, goalDist / decelDist);
desiredSpeed = maxSpeed * speedScale * attrGain;

% ---- 斥力：障碍物 + 边界 ----
repVec = [0, 0];

% 边界斥力（坐标系：原点左下，x右 y上）[V1 新增]
margin = robot.radius + boundaryMargin;
bottomDist = robot.pos(2) - margin;
if bottomDist < boundaryRange && bottomDist > 0
    repVec(2) = repVec(2) + boundaryGain / max(bottomDist, 0.01)^2;
end
topDist = n - robot.pos(2) - margin;
if topDist < boundaryRange && topDist > 0
    repVec(2) = repVec(2) - boundaryGain / max(topDist, 0.01)^2;
end
leftDist = robot.pos(1) - margin;
if leftDist < boundaryRange && leftDist > 0
    repVec(1) = repVec(1) + boundaryGain / max(leftDist, 0.01)^2;
end
rightDist = n - robot.pos(1) - margin;
if rightDist < boundaryRange && rightDist > 0
    repVec(1) = repVec(1) - boundaryGain / max(rightDist, 0.01)^2;
end

% 障碍物斥力（遍历传感器范围内的占用栅格）
r0 = ceil(robot.pos(2));
c0 = ceil(robot.pos(1));
evadeVec = [0, 0];  % 侧向避让累计量 [V1 新增]
for dr = -sensorRange:sensorRange
    for dc = -sensorRange:sensorRange
        r = r0 + dr; c = c0 + dc;
        if r < 1 || r > n || c < 1 || c > n, continue; end
        if occGrid(r, c)
            gx = c - 0.5; gy = r - 0.5;            % 障碍栅格中心连续坐标
            diff = robot.pos - [gx, gy];           % 机器人→障碍物向量
            d = norm(diff);
            if d < 1e-2, d = 1e-2; end             % 避免除零
            if d < repulseRange
                % 径向斥力
                repVec = repVec + (diff / d) * (repulseGain / d^2);
                % 侧向避让：障碍物在前方时产生横向推力绕行 [V1 新增]
                obstDir = -diff / d;                % 机器人→障碍物方向
                forwardProj = dot(obstDir, attrDir);
                if forwardProj > 0.3                % 障碍物在前进方向内
                    lateral = obstDir - forwardProj * attrDir;
                    latNorm = norm(lateral);
                    if latNorm > 0.05
                        evadeDir = -lateral / latNorm;
                    else
                        evadeDir = [-attrDir(2), attrDir(1)];
                    end
                    evadeVec = evadeVec + evadeDir * (tangentGain * repulseGain / d^2) * forwardProj;
                end
            end
        end
    end
end

% ---- 其他机器人斥力 [V1 新增] ----
robotRepVec = [0, 0];
if isfield(params, 'allRobots') && ~isempty(params.allRobots) ...
        && isfield(params, 'robotIdx') && params.robotIdx > 0
    allRobots = params.allRobots;
    myIdx = params.robotIdx;
    for j = 1:length(allRobots)
        if j == myIdx, continue; end
        other = allRobots{j};
        diff = robot.pos - other.pos;           % 自身→对方向量（排斥方向）
        d = norm(diff);
        if d < 1e-2, d = 1e-2; end
        if d < robotRepulseRange
            % 径向斥力：距离越近斥力越强，与障碍物斥力公式一致但增益更大
            robotRepVec = robotRepVec + (diff / d) * (robotRepulseGain / d^2);
            % 对向接近时额外侧向避让
            relVel = robot.vel - other.vel;
            closingSpeed = -dot(relVel, diff / d);  % 正=正在靠近
            if closingSpeed > 0.1
                % 横向偏转：选择与 attrDir 投影更大的侧向
                lateral1 = [-diff(2), diff(1)] / d;
                lateral2 = [diff(2), -diff(1)] / d;
                if abs(dot(lateral1, attrDir)) > abs(dot(lateral2, attrDir))
                    evadeDir = lateral1 * sign(dot(lateral1, attrDir) + 1e-6);
                else
                    evadeDir = lateral2 * sign(dot(lateral2, attrDir) + 1e-6);
                end
                robotRepVec = robotRepVec + evadeDir * (robotRepulseGain * closingSpeed / d^2);
            end
        end
    end
end

% ---- 合成速度 ----
desiredV = attrDir * desiredSpeed + repVec + evadeVec + robotRepVec;
vMag = norm(desiredV);
if vMag > maxSpeed
    desiredV = desiredV / vMag * maxSpeed;
end

% ---- 加速度限幅（防止速度突变）----
dv = desiredV - robot.vel;
dvMag = norm(dv);
maxDV = maxAccel * dt;     % 一个时间步内允许的最大速度变化量
if dvMag > maxDV
    dv = dv / dvMag * maxDV;
end
vx = robot.vel(1) + dv(1);
vy = robot.vel(2) + dv(2);

% ---- 预测轨迹（用于可视化）----
if nargout > 2
    steps = 10;
    predictTraj = zeros(steps, 2);
    px = robot.pos(1); py = robot.pos(2);
    for k = 1:steps
        px = px + vx * dt; py = py + vy * dt;
        predictTraj(k, :) = [px, py];
    end
end
end
