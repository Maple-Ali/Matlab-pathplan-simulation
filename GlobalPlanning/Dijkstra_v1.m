function path = Dijkstra_v1(map, startGrid, goalGrid, delay, callback)
%DIJKSTRA_V1 Dijkstra 全局路径规划（二叉堆优化版）
%   使用最小二叉堆替代线性扫描提取最小距离节点，
%   复杂度从 O(V²) 降至 O((V+E)logV)。
%
%   接口与 Dijkstra 完全一致。

if nargin < 4
    delay = 0;
end
if nargin < 5
    callback = [];
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
parent = zeros(n, n, 2);
visited = false(n, n);

dist(startGrid(1), startGrid(2)) = 0;

% --- 二叉堆（预分配，避免动态增长） ---
maxHeapSize = n * n;
heapDist = inf(1, maxHeapSize);
heapRow = zeros(1, maxHeapSize);
heapCol = zeros(1, maxHeapSize);
heapSize = 0;

% 插入起点
heapSize = heapSize + 1;
heapDist(heapSize) = 0;
heapRow(heapSize) = startGrid(1);
heapCol(heapSize) = startGrid(2);

if delay > 0
    hold on;
end

iter = 0;
while heapSize > 0
    % --- extract-min ---
    currentDist = heapDist(1);
    cr = heapRow(1);
    cc = heapCol(1);

    % 末尾元素移到堆顶
    heapDist(1) = heapDist(heapSize);
    heapRow(1) = heapRow(heapSize);
    heapCol(1) = heapCol(heapSize);
    heapSize = heapSize - 1;

    % 下沉调整
    pos = 1;
    while true
        left = 2 * pos;
        right = 2 * pos + 1;
        smallest = pos;
        if left <= heapSize && heapDist(left) < heapDist(smallest)
            smallest = left;
        end
        if right <= heapSize && heapDist(right) < heapDist(smallest)
            smallest = right;
        end
        if smallest ~= pos
            heapDist([pos, smallest]) = heapDist([smallest, pos]);
            heapRow([pos, smallest]) = heapRow([smallest, pos]);
            heapCol([pos, smallest]) = heapCol([smallest, pos]);
            pos = smallest;
        else
            break;
        end
    end

    % 跳过已访问节点
    if visited(cr, cc)
        continue;
    end
    visited(cr, cc) = true;
    iter = iter + 1;

    % --- callback ---
    if ~isempty(callback)
        stateInfo = struct('type', 'step', ...
            'current', [cr, cc], ...
            'openSet', ~visited & ~occGrid & dist < inf, ...
            'closedSet', visited, ...
            'dist', dist, 'iteration', iter);
        action = callback(stateInfo);
        if strcmp(action, 'stop')
            path = []; return;
        end
    end

    % --- 早期终止：到达目标 ---
    if cr == goalGrid(1) && cc == goalGrid(2)
        path = reconstructPath(parent, startGrid, goalGrid);
        if ~isempty(callback)
            callback(struct('type', 'finish', 'path', path, ...
                'iteration', iter, 'success', true));
        end
        return;
    end

    if delay > 0 && mod(iter, 10) == 0
        plot(cc - 0.5, cr - 0.5, 's', ...
            'MarkerSize', 3, 'MarkerFaceColor', [0.8, 0.8, 0.8], ...
            'MarkerEdgeColor', 'none');
        drawnow;
        pause(delay);
    end

    % --- 扩展邻居 ---
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

        newDist = currentDist + moveCost(d);
        if newDist < dist(nr, nc)
            dist(nr, nc) = newDist;
            parent(nr, nc, :) = [cr, cc];

            % 插入堆
            heapSize = heapSize + 1;
            heapDist(heapSize) = newDist;
            heapRow(heapSize) = nr;
            heapCol(heapSize) = nc;

            % 上浮
            pos = heapSize;
            while pos > 1
                parentPos = floor(pos / 2);
                if heapDist(parentPos) > heapDist(pos)
                    heapDist([pos, parentPos]) = heapDist([parentPos, pos]);
                    heapRow([pos, parentPos]) = heapRow([parentPos, pos]);
                    heapCol([pos, parentPos]) = heapCol([parentPos, pos]);
                    pos = parentPos;
                else
                    break;
                end
            end
        end
    end
end

% 无路径
if ~isempty(callback)
    callback(struct('type', 'finish', 'path', [], ...
        'iteration', iter, 'success', false));
end
path = [];
end

function path = reconstructPath(parent, startGrid, goalGrid)
    pathBuf = zeros(1, size(parent,1) * size(parent,2));
    pathLen = 1;
    pathBuf(1) = sub2ind(size(parent(:,:,1)), goalGrid(1), goalGrid(2));
    current = goalGrid;
    while ~(current(1) == startGrid(1) && current(2) == startGrid(2))
        current = squeeze(parent(current(1), current(2), :))';
        if all(current == 0)
            path = [];
            return;
        end
        pathLen = pathLen + 1;
        pathBuf(pathLen) = sub2ind(size(parent(:,:,1)), current(1), current(2));
    end
    pathLin = pathBuf(pathLen:-1:1);
    [rows, cols] = ind2sub([size(parent,1), size(parent,2)], pathLin);
    path = [rows(:), cols(:)];
end
