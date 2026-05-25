function path = RRT(map, startGrid, goalGrid, delay)
%RRT 快速随机扩展树路径规划
%   path = RRT(map, startGrid, goalGrid, delay)

if nargin < 4
    delay = 0;
end

n = map.mapSize;
occGrid = map.getOccupancyGrid();

if occGrid(startGrid(1), startGrid(2)) || occGrid(goalGrid(1), goalGrid(2))
    path = [];
    return;
end

% 可调参数
stepSize = 2.0;          % 扩展步长（栅格单位）
maxIter = 5000;           % 最大迭代次数
goalBias = 0.1;          % 目标偏置概率
goalThreshold = 1.5;     % 到达目标的距离阈值

% 树节点: [row, col, parentIdx]
nodes = [startGrid(1), startGrid(2), 0];

if delay > 0
    hold on;
end

for iter = 1:maxIter
    % 采样
    if rand < goalBias
        sample = [goalGrid(1), goalGrid(2)];
    else
        sample = [randi(n), randi(n)];
        % 跳过障碍物采样
        retryCount = 0;
        while occGrid(sample(1), sample(2)) && retryCount < 50
            sample = [randi(n), randi(n)];
            retryCount = retryCount + 1;
        end
    end

    % 找最近节点
    nearestIdx = 1;
    nearestDist = inf;
    for i = 1:size(nodes, 1)
        d = norm([nodes(i, 1) - sample(1), nodes(i, 2) - sample(2)]);
        if d < nearestDist
            nearestDist = d;
            nearestIdx = i;
        end
    end

    % 向采样点扩展
    dirVec = [sample(1) - nodes(nearestIdx, 1), sample(2) - nodes(nearestIdx, 2)];
    dist = norm(dirVec);
    if dist < 1e-6
        continue;
    end
    dirVec = dirVec / dist;
    newPt = [nodes(nearestIdx, 1) + dirVec(1) * min(stepSize, dist), ...
             nodes(nearestIdx, 2) + dirVec(2) * min(stepSize, dist)];

    % 栅格化检查
    nr = round(newPt(1));
    nc = round(newPt(2));
    nr = max(1, min(n, nr));
    nc = max(1, min(n, nc));

    if occGrid(nr, nc)
        continue;
    end

    % 检查连线上是否无障碍（步进检查）
    if ~isLineFree(nodes(nearestIdx, 1:2), [nr, nc], occGrid, n)
        continue;
    end

    newIdx = size(nodes, 1) + 1;
    nodes(newIdx, :) = [nr, nc, nearestIdx];

    % 可视化
    if delay > 0
        plot([nodes(nearestIdx, 2) - 0.5, nc - 0.5], ...
             [nodes(nearestIdx, 1) - 0.5, nr - 0.5], 'c-', 'LineWidth', 0.5);
        drawnow;
        pause(delay);
    end

    % 检查是否到达目标
    if norm([nr - goalGrid(1), nc - goalGrid(2)]) < goalThreshold
        if isLineFree([nr, nc], goalGrid, occGrid, n)
            nodes(end + 1, :) = [goalGrid(1), goalGrid(2), newIdx];
            break;
        end
    end
end

% 重建路径
if nodes(end, 1) ~= goalGrid(1) || nodes(end, 2) ~= goalGrid(2)
    path = [];
    return;
end

path = [goalGrid(1), goalGrid(2)];
parentIdx = nodes(end, 3);
while parentIdx > 0
    path = [nodes(parentIdx, 1:2); path];
    parentIdx = nodes(parentIdx, 3);
end
end

function free = isLineFree(p1, p2, occGrid, n)
% 步进检测两点连线上是否有障碍物
steps = max(abs(p1(1) - p2(1)), abs(p1(2) - p2(2))) * 2;
steps = max(steps, 10);
for t = 0:steps
    alpha = t / steps;
    r = round(p1(1) + alpha * (p2(1) - p1(1)));
    c = round(p1(2) + alpha * (p2(2) - p1(2)));
    r = max(1, min(n, r));
    c = max(1, min(n, c));
    if occGrid(r, c)
        free = false;
        return;
    end
end
free = true;
end
