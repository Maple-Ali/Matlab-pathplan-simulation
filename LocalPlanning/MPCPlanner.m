function [vx, vy, predictTraj] = MPCPlanner(robot, localGoal, map, ~, dt, params)
%MPCPLANNER 模型预测控制局部规划器（简化版框架）
%   当前版本使用纯比例导引 + 速度平滑简化实现
%   保留 MPC 框架结构，内部可用 fmincon 或二次规划替换
%
%   MPC 核心思想：基于机器人运动模型，在有限预测时域内
%   求解最优控制序列，最小化跟踪误差和控制代价。
%   当前简化实现：
%     1. 计算期望速度方向（比例导引）
%     2. 对速度变化率进行平滑（模拟 MPC 的控制平滑效果）
%     3. 施加障碍物约束（软约束）

persistent prevVelMap
if isempty(prevVelMap)
    prevVelMap = containers.Map('KeyType', 'double', 'ValueType', 'any');
end

% 获取机器人索引（多机器人时用于隔离状态）
idx = 0;
if isfield(params, 'robotIdx'), idx = params.robotIdx; end

if prevVelMap.isKey(idx)
    prevV = prevVelMap(idx);
else
    prevV = [0, 0];
end

% --- 参数 ---
maxSpeed = 1.0;
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
smoothFactor = 0.3;  % 平滑系数（模拟 MPC 控制代价）
if isfield(params, 'smoothFactor'), smoothFactor = params.smoothFactor; end
predictHorizon = 10;  % MPC 预测步数
if isfield(params, 'predictHorizon'), predictHorizon = params.predictHorizon; end
sensorRange = 2.5;
if isfield(params, 'sensorRange'), sensorRange = params.sensorRange; end

% --- 比例导引：计算期望速度 ---
goalVec = localGoal - robot.pos;
goalDist = norm(goalVec);
if goalDist < 1e-6
    vx = 0; vy = 0;
    prevVelMap(idx) = [0, 0];
    predictTraj = [];
    return;
end

desiredV = goalVec / goalDist * maxSpeed;

% --- 障碍物软约束 ---
occGrid = map.getLocalOccGrid(robot.pos, sensorRange);
n = map.mapSize;
robotR = ceil(robot.pos(2));
robotC = ceil(robot.pos(1));
checkRange = ceil(sensorRange);

repV = [0, 0];
for dr = -checkRange:checkRange
    for dc = -checkRange:checkRange
        r = robotR + dr;
        c = robotC + dc;
        if r < 1 || r > n || c < 1 || c > n
            continue;
        end
        if occGrid(r, c)
            gridCX = c - 0.5;
            gridCY = r - 0.5;
            diff = robot.pos - [gridCX, gridCY];
            d = norm(diff);
            if d < 1e-6, d = 1e-6; end
            if d < sensorRange
                repV = repV + (diff / d) * (1.0 / d);
            end
        end
    end
end

desiredV = desiredV + repV;
if norm(desiredV) > maxSpeed
    desiredV = desiredV / norm(desiredV) * maxSpeed;
end

% --- 速度平滑（类似 MPC 控制代价）---
rawVx = (1 - smoothFactor) * prevV(1) + smoothFactor * desiredV(1);
rawVy = (1 - smoothFactor) * prevV(2) + smoothFactor * desiredV(2);

vx = rawVx;
vy = rawVy;
prevVelMap(idx) = [vx, vy];

% --- MPC 预测轨迹 ---
if nargout > 2
    predictTraj = zeros(predictHorizon, 2);
    px = robot.pos(1);
    py = robot.pos(2);
    for k = 1:predictHorizon
        px = px + vx * dt;
        py = py + vy * dt;
        predictTraj(k, :) = [px, py];
    end
end
end
