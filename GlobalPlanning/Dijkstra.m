function path = Dijkstra(map, startGrid, goalGrid, delay)
%DIJKSTRA Dijkstra 全局路径规划
%   与 A* 接口一致，使用 8 邻域扩展

if nargin < 4
    delay = 0;
end

n = map.mapSize;
occGrid = map.getOccupancyGrid();

if occGrid(startGrid(1), startGrid(2)) || occGrid(goalGrid(1), goalGrid(2))
    path = [];
    return;
end

dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

dist = inf(n, n);
dist(startGrid(1), startGrid(2)) = 0;

parent = zeros(n, n, 2);
visited = false(n, n);

if delay > 0
    hold on;
end

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

    if cr == goalGrid(1) && cc == goalGrid(2)
        path = reconstructPath(parent, startGrid, goalGrid);
        return;
    end

    if delay > 0 && mod(iter, 10) == 0
        plot(cc - 0.5, cr - 0.5, 's', ...
            'MarkerSize', 3, 'MarkerFaceColor', [0.8, 0.8, 0.8], ...
            'MarkerEdgeColor', 'none');
        drawnow;
        pause(delay);
    end

    for d = 1:8
        nr = cr + dRow(d);
        nc = cc + dCol(d);
        if nr < 1 || nr > n || nc < 1 || nc > n
            continue;
        end
        if visited(nr, nc) || occGrid(nr, nc)
            continue;
        end
        if dRow(d) ~= 0 && dCol(d) ~= 0
            if occGrid(cr + dRow(d), cc) || occGrid(cr, cc + dCol(d))
                continue;
            end
        end

        newDist = dist(cr, cc) + moveCost(d);
        if newDist < dist(nr, nc)
            dist(nr, nc) = newDist;
            parent(nr, nc, :) = [cr, cc];
        end
    end
end

path = [];
end

function path = reconstructPath(parent, startGrid, goalGrid)
    path = goalGrid;
    current = goalGrid;
    while ~(current(1) == startGrid(1) && current(2) == startGrid(2))
        current = squeeze(parent(current(1), current(2), :))';
        if all(current == 0)
            path = [];
            return;
        end
        path = [current; path];
    end
end
