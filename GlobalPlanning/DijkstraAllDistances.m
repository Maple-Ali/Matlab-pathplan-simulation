function distances = DijkstraAllDistances(map, startGrid, goalGrids)
%DIJKSTRAALLDISTANCES Dijkstra 单源多目标最短路径
%   从 startGrid 出发，计算到 goalGrids 中所有目标点的最短路径距离。
%   搜索到所有目标点后立即停止，无需遍历全图。
%
%   distances = DijkstraAllDistances(map, startGrid, goalGrids)
%
%   Inputs:
%     map       - Map 对象（栅格地图）
%     startGrid - 1×2 起点 [row, col]
%     goalGrids - M×2 各目标点 [row, col]
%
%   Output:
%     distances - M×1 各目标点的最短路径距离
%                 不可达的点为 inf

n = map.mapSize;
occGrid = map.getOccupancyGrid();

numGoals = size(goalGrids, 1);
distances = inf(numGoals, 1);

% 起点在障碍物上则直接返回
if occGrid(startGrid(1), startGrid(2))
    return;
end

dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

dist = inf(n, n);
dist(startGrid(1), startGrid(2)) = 0;
visited = false(n, n);

% 目标点查找表：goalMap(r,c) = 目标索引（0 表示非目标）
goalMap = zeros(n, n);
for g = 1:numGoals
    gr = goalGrids(g, 1);
    gc = goalGrids(g, 2);
    if ~occGrid(gr, gc)
        goalMap(gr, gc) = g;
    end
end

foundCount = 0;
totalReachable = numGoals - sum(occGrid(sub2ind([n,n], goalGrids(:,1), goalGrids(:,2))));
unvisitedCount = n * n - sum(occGrid(:));

for iter = 1:unvisitedCount
    % 找未访问中距离最小的节点
    candidates = find(~visited & ~occGrid);
    if isempty(candidates)
        break;
    end
    [~, idx] = min(dist(candidates));
    currentIdx = candidates(idx);
    [cr, cc] = ind2sub([n, n], currentIdx);

    if dist(cr, cc) == inf
        break;
    end

    visited(cr, cc) = true;

    % 检查是否为目标点
    g = goalMap(cr, cc);
    if g > 0 && isinf(distances(g))
        distances(g) = dist(cr, cc);
        foundCount = foundCount + 1;
        if foundCount >= totalReachable
            break;  % 所有可达目标点已找到
        end
    end

    % 扩展邻居
    for d = 1:8
        nr = cr + dRow(d);
        nc = cc + dCol(d);
        if nr < 1 || nr > n || nc < 1 || nc > n
            continue;
        end
        if visited(nr, nc) || occGrid(nr, nc)
            continue;
        end
        % 对角线穿障检测
        if dRow(d) ~= 0 && dCol(d) ~= 0
            if occGrid(cr + dRow(d), cc) || occGrid(cr, cc + dCol(d))
                continue;
            end
        end

        newDist = dist(cr, cc) + moveCost(d);
        if newDist < dist(nr, nc)
            dist(nr, nc) = newDist;
        end
    end
end
end
