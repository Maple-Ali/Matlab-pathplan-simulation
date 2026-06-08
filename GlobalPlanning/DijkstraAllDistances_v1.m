function distances = DijkstraAllDistances_v1(map, startGrid, goalGrids)
%DIJKSTRAALLDISTANCES_V1 Dijkstra 单源多目标最短路径（二叉堆+早期终止版）
%   使用最小二叉堆替代线性扫描提取最小距离节点，
%   堆顶距离超过已找到最远目标时提前停止。
%
%   distances = DijkstraAllDistances_v1(map, startGrid, goalGrids)
%
%   Inputs / Outputs: 同 DijkstraAllDistances

n = map.mapSize;
occGrid = map.getOccupancyGrid();

numGoals = size(goalGrids, 1);
distances = inf(numGoals, 1);

if occGrid(startGrid(1), startGrid(2))
    return;
end

dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

dist = inf(n, n);
visited = false(n, n);
dist(startGrid(1), startGrid(2)) = 0;

% 目标点查找表：goalMap(r,c) = 目标索引（0 表示非目标）
goalMap = zeros(n, n);
validGoals = 0;
for g = 1:numGoals
    gr = goalGrids(g, 1);
    gc = goalGrids(g, 2);
    if ~occGrid(gr, gc)
        goalMap(gr, gc) = g;
        validGoals = validGoals + 1;
    end
end

% --- 二叉堆（存储 [dist, row, col] 三元组） ---
% 使用单个 n*n 大小的数组，避免动态增长
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

foundCount = 0;
maxFoundDist = 0;

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

    % 跳过已访问节点（堆中可能有同一节点的多个副本）
    if visited(cr, cc)
        continue;
    end
    visited(cr, cc) = true;

    % 检查是否为目标点
    g = goalMap(cr, cc);
    if g > 0 && isinf(distances(g))
        distances(g) = currentDist;
        foundCount = foundCount + 1;
        if currentDist > maxFoundDist
            maxFoundDist = currentDist;
        end
        if foundCount >= validGoals
            break;
        end
    end

    % 早期终止：堆顶距离已超过最远目标
    if foundCount > 0 && currentDist > maxFoundDist
        break;
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
        if dRow(d) ~= 0 && dCol(d) ~= 0
            if occGrid(cr + dRow(d), cc) || occGrid(cr, cc + dCol(d))
                continue;
            end
        end

        newDist = currentDist + moveCost(d);
        if newDist < dist(nr, nc)
            dist(nr, nc) = newDist;

            % 插入堆（允许重复，visited 会过滤）
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
end
