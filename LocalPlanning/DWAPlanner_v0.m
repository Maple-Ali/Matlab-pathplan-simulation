function [vx, vy, predictTraj] = DWAPlanner_v0(robot, localGoal, map, ~, dt, params)
%DWAPLANNER_V0 局部规划器（引力+斥力势场模型，原始版）
%   [vx, vy, predictTraj] = DWAPlanner_v0(robot, localGoal, map, ~, dt, params)
%
%   仅包含引力（目标）和斥力（障碍物）的基础势场模型。
%   不含 V1 的扩展特性：边界斥力、侧向避让、机器人间斥力、接近减速。
%   用于与 V1 版本对比评估各扩展特性的效果。
%
%   params 可选字段: maxSpeed, maxAccel

% ===== 可调参数 ============================================================
% 引力参数
attrGain  = 1.0;     % 引力增益，增大使机器人更积极靠近目标

% 斥力参数（障碍物）
repulseGain  = 0.5;  % 障碍物斥力增益，增大使机器人更强烈避开障碍
                     %   公式: repulseGain / d^2，d为机器人到障碍物距离
repulseRange = 2.5;  % 斥力作用范围（连续单位），障碍物在此范围内才产生斥力

% 传感器参数
sensorRange  = 4;    % 障碍物搜索半径（栅格数），决定检测多远处的障碍物

% 速度参数
maxSpeed = 1.0;      % 机器人最大速度
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
maxAccel = 2.0;      % 最大加速度，限制每步速度变化量
if isfield(params, 'maxAccel'), maxAccel = params.maxAccel; end
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
desiredSpeed = maxSpeed * attrGain;

% ---- 斥力：障碍物 ----
repVec = [0, 0];
r0 = ceil(robot.pos(2));
c0 = ceil(robot.pos(1));
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
                repVec = repVec + (diff / d) * (repulseGain / d^2);
            end
        end
    end
end

% ---- 合成速度 ----
desiredV = attrDir * desiredSpeed + repVec;
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
