function collision = checkCollision(robotPos, robotRadius, map)
%CHECKCOLLISION 碰撞检测
%   检查机器人圆域是否与障碍物栅格重叠
%   robotPos: [x, y] 连续坐标
%   robotRadius: 机器人半径
%   map: Map 对象

collision = false;
occGrid = map.getOccupancyGrid();
n = map.mapSize;

% 检查机器人边界框覆盖的栅格范围
% 连续坐标 (x, y) → 栅格 (r, c): r=ceil(y), c=ceil(x)
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
            % 栅格中心连续坐标
            gridCX = c - 0.5;
            gridCY = r - 0.5;
            % 检查圆与正方形栅格的重叠（简化：检查栅格角点或中心）
            dist = sqrt((robotPos(1) - gridCX)^2 + (robotPos(2) - gridCY)^2);
            if dist < robotRadius + 0.5  % 栅格半对角线 ≈ 0.707，取 0.5 保守
                collision = true;
                return;
            end
        end
    end
end
end
