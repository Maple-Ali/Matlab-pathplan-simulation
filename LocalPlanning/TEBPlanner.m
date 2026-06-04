function [vx, vy, predictTraj] = TEBPlanner(robot, localGoal, map, ~, dt, params)
%TEBPLANNER 时间弹性带局部规划器（简化版框架）
%   当前版本使用纯比例导引 + 斥力场简化实现
%   保留 TEB 框架结构，内部优化可用 CasADi/fmincon 替换
%
%   TEB 核心思想：在全局路径上优化一系列机器人位姿序列，
%   同时考虑时间最优性、避障约束、运动学约束等。
%   当前简化实现：
%     1. 计算引力方向（朝向局部目标）
%     2. 计算斥力方向（远离障碍物）
%     3. 合力作为速度指令

% --- 参数 ---
maxSpeed = 1.0;
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
repulsiveGain = 0.5;
if isfield(params, 'repulsiveGain'), repulsiveGain = params.repulsiveGain; end
sensorRange = 2.5;
if isfield(params, 'sensorRange'), sensorRange = params.sensorRange; end

% --- 引力：朝向局部目标 ---
goalVec = localGoal - robot.pos;
goalDist = norm(goalVec);
if goalDist > 1e-6
    attrForce = goalVec / goalDist * maxSpeed;
else
    vx = 0; vy = 0; predictTraj = []; return;
end

% --- 斥力：远离障碍物 ---
occGrid = map.getOccupancyGrid();
n = map.mapSize;
repForce = [0, 0];

% 检查机器人周围栅格
robotR = ceil(robot.pos(2));
robotC = ceil(robot.pos(1));
checkRange = ceil(sensorRange);

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
            obsVec = [gridCX, gridCY];
            diff = robot.pos - obsVec;
            d = norm(diff);
            if d < 1e-6, d = 1e-6; end
            if d < sensorRange
                repForce = repForce + (diff / d) * (repulsiveGain / d^2);
            end
        end
    end
end

% --- 合力 ---
totalForce = attrForce + repForce;

% --- 限幅 ---
forceMag = norm(totalForce);
if forceMag > maxSpeed
    totalForce = totalForce / forceMag * maxSpeed;
end

vx = totalForce(1);
vy = totalForce(2);

% --- 预测轨迹（简化线性外推）---
if nargout > 2
    tHorizon = 1.0;
    steps = ceil(tHorizon / dt);
    predictTraj = zeros(steps, 2);
    for i = 1:steps
        predictTraj(i, :) = robot.pos + totalForce * (dt * i);
    end
end
end
