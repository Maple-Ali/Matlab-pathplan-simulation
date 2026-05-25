function [vx, vy, predictTraj] = DWAPlanner(robot, localGoal, map, ~, dt, params)
%DWAPLANNER 局部规划器
%   引力+斥力模型，带加速度限幅和接近减速

maxSpeed = 1.0;
if isfield(params, 'maxSpeed'), maxSpeed = params.maxSpeed; end
maxAccel = 2.0;
if isfield(params, 'maxAccel'), maxAccel = params.maxAccel; end

occGrid = map.getOccupancyGrid();
n = map.mapSize;

% 引力方向
goalVec = localGoal - robot.pos;
goalDist = norm(goalVec);
if goalDist < 1e-6
    vx = 0; vy = 0; predictTraj = []; return;
end
attrDir = goalVec / goalDist;

% 接近目标时减速
speedScale = min(1.0, goalDist / 2.0);
desiredSpeed = maxSpeed * speedScale;

% 斥力：来自障碍物和边界
repVec = [0, 0];

% 边界斥力
margin = robot.radius + 0.3;
leftDist = robot.pos(1) - margin;
if leftDist < 0.5 && leftDist > 0
    repVec(1) = repVec(1) + 0.5 / max(leftDist, 0.01)^2;
end
rightDist = n - robot.pos(1) - margin;
if rightDist < 0.5 && rightDist > 0
    repVec(1) = repVec(1) - 0.5 / max(rightDist, 0.01)^2;
end
topDist = robot.pos(2) - margin;
if topDist < 0.5 && topDist > 0
    repVec(2) = repVec(2) + 0.5 / max(topDist, 0.01)^2;
end
bottomDist = n - robot.pos(2) - margin;
if bottomDist < 0.5 && bottomDist > 0
    repVec(2) = repVec(2) - 0.5 / max(bottomDist, 0.01)^2;
end

% 障碍物斥力
r0 = ceil(robot.pos(2));
c0 = ceil(robot.pos(1));
sr = 3;
for dr = -sr:sr
    for dc = -sr:sr
        r = r0 + dr; c = c0 + dc;
        if r < 1 || r > n || c < 1 || c > n, continue; end
        if occGrid(r, c)
            gx = c - 0.5; gy = r - 0.5;
            diff = robot.pos - [gx, gy];
            d = norm(diff);
            if d < 1e-2, d = 1e-2; end
            if d < 2.0
                repVec = repVec + (diff / d) * (0.3 / d^2);
            end
        end
    end
end

% 合成速度
desiredV = attrDir * desiredSpeed + repVec;
vMag = norm(desiredV);
if vMag > maxSpeed
    desiredV = desiredV / vMag * maxSpeed;
end

% 加速度限幅
dv = desiredV - robot.vel;
dvMag = norm(dv);
maxDV = maxAccel * dt;
if dvMag > maxDV
    dv = dv / dvMag * maxDV;
end
vx = robot.vel(1) + dv(1);
vy = robot.vel(2) + dv(2);

% 预测轨迹
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
