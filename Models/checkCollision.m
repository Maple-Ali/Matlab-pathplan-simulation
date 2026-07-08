function collision = checkCollision(robotPos, robotRadius, map)
%CHECKCOLLISION 碰撞检测
%   检查机器人圆域是否与障碍物重叠（含静态和动态障碍物）
%   robotPos: [x, y] 连续坐标
%   robotRadius: 机器人半径
%   map: Map 对象

collision = false;

% --- 1. 静态障碍物：基于占用栅格 ---
occGrid = map.getOccupancyGrid();
n = map.mapSize;

minX = robotPos(1) - robotRadius;
maxX = robotPos(1) + robotRadius;
minY = robotPos(2) - robotRadius;
maxY = robotPos(2) + robotRadius;

rMin = max(1, floor(minY) + 1);
rMax = min(n, ceil(maxY));
cMin = max(1, floor(minX) + 1);
cMax = min(n, ceil(maxX));

for r = rMin:rMax
    for c = cMin:cMax
        if occGrid(r, c)
            gridCX = c - 0.5;
            gridCY = r - 0.5;
            dist = sqrt((robotPos(1) - gridCX)^2 + (robotPos(2) - gridCY)^2);
            if dist < robotRadius + 0.5
                collision = true;
                return;
            end
        end
    end
end

% --- 2. 动态障碍物：直接检查距离（不受检测范围限制）---
for i = 1:length(map.dynamicObstacles)
    obs = map.dynamicObstacles(i);
    dist = norm(robotPos - obs.currentPos);
    if dist < robotRadius + 0.3  % 0.3 为动态障碍物等效半径
        collision = true;
        return;
    end
end
end
