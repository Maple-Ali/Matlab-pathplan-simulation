function [path, info] = AStar_v1_2(map, startGrid, goalGrid, delay, callback, alpha, d_ref)
%ASTAR_V1_2 A* 全局路径规划 — 改进版 v1_2（按需局部障碍物距离计算）
%   [path, info] = AStar_v1_2(map, startGrid, goalGrid, delay, callback, alpha, d_ref)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组
%   info: 结构体 — .expandedNodes, .pathLength, .pathCost, .openMaxSize
%
%   改进内容（相比 v1_1）：
%     1. 按需局部障碍物距离计算 — 不再预计算全局距离场，仅对实际访问的节点计算
%     2. 缓存机制 — 已计算距离的节点缓存结果，避免重复计算
%     3. 局部搜索半径限制 — 搜索范围可控，平衡精度与效率
%
%   ============================================
%   主要可调节参数（直接修改下方默认值即可调试）
%   ============================================
%   alpha  = 0.3;   % 最大额外权重，范围建议 [0.1, 0.5]
%   d_ref  = 0.08;  % 障碍物距离阈值（占地图对角距离百分比）
%   maxSearchR = 5;  % 局部障碍物搜索半径（栅格数）
%   ============================================

if nargin < 4, delay = 0; end
if nargin < 5, callback = []; end
if nargin < 6 || isempty(alpha), alpha = 0.5; end
if nargin < 7 || isempty(d_ref), d_ref = 0.03; end

maxSearchR = 5;     % 局部搜索半径（越小越快，但远处障碍物距离会被截断）

n = map.mapSize;
occGrid = map.getOccupancyGrid();

if occGrid(startGrid(1), startGrid(2)) || occGrid(goalGrid(1), goalGrid(2))
    path = [];
    info = struct('expandedNodes',0,'pathLength',0,'pathCost',inf,'openMaxSize',0,'distComputeCount',0);
    return;
end

% 8邻域
dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

% 障碍物距离缓存（NaN = 未计算）
distCache = nan(n, n);
distComputeCount = 0;

% 距离阈值
diagonalDist = sqrt(2 * n^2);
d_actual = d_ref * diagonalDist;

% 启发式
h = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);

% g/f/h 值矩阵
gScore = inf(n, n);
gScore(startGrid(1), startGrid(2)) = 0;

hScore = inf(n, n);
hScore(startGrid(1), startGrid(2)) = h(startGrid(1), startGrid(2));

% 计算起点的加权 f 值
d0 = computeLocalDist(startGrid(1), startGrid(2));
w0 = 1 + alpha * (1 - exp(-d0 / d_actual));
fScore = inf(n, n);
fScore(startGrid(1), startGrid(2)) = w0 * hScore(startGrid(1), startGrid(2));

parent = zeros(n, n, 2);

% 二叉堆
heapSize = 0;
heap = zeros(n * n, 4);
heapPos = zeros(n, n);

heapSize = heapSize + 1;
heap(heapSize, :) = [fScore(startGrid(1), startGrid(2)), ...
                     hScore(startGrid(1), startGrid(2)), ...
                     startGrid(1), startGrid(2)];
heapPos(startGrid(1), startGrid(2)) = heapSize;
bubbleUp(heapSize);
openMaxSize = 1;

closedSet = false(n, n);
iter = 0;
bestGoalG = inf;
bestPath = [];
expandedCount = 0;

if delay > 0
    figure(gcf);
    hold on;
    exploredNodes = [];
end

while heapSize > 0
    topRow = heap(1, 3);
    topCol = heap(1, 4);
    current = [topRow, topCol];

    heapPos(topRow, topCol) = 0;
    heap(1, :) = heap(heapSize, :);
    heapPos(heap(heapSize, 3), heap(heapSize, 4)) = 1;
    heapSize = heapSize - 1;
    if heapSize > 0, bubbleDown(1); end

    if ~isempty(callback)
        iter = iter + 1;
        openSet = false(n, n);
        for i = 1:heapSize
            openSet(heap(i, 3), heap(i, 4)) = true;
        end
        stateInfo = struct('type', 'step', ...
            'current', current, ...
            'openSet', openSet, 'closedSet', closedSet, ...
            'gScore', gScore, 'fScore', fScore, 'iteration', iter);
        action = callback(stateInfo);
        if strcmp(action, 'stop')
            path = [];
            info = struct('expandedNodes',expandedCount,'pathLength',0,'pathCost',inf,'openMaxSize',openMaxSize,'distComputeCount',distComputeCount);
            return;
        end
    end

    if current(1) == goalGrid(1) && current(2) == goalGrid(2)
        goalG = gScore(goalGrid(1), goalGrid(2));
        if alpha == 0 || heapSize == 0 || heap(1, 1) >= goalG
            path = reconstructPath(parent, startGrid, goalGrid);
            expandedCount = expandedCount + 1;
            info = struct('expandedNodes', expandedCount, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', goalG, ...
                          'openMaxSize', openMaxSize, ...
                          'distComputeCount', distComputeCount);
            if ~isempty(callback)
                callback(struct('type', 'finish', 'path', path, 'iteration', iter, 'success', true));
            end
            return;
        else
            if goalG < bestGoalG
                bestGoalG = goalG;
                bestPath = reconstructPath(parent, startGrid, goalGrid);
            end
        end
    end

    closedSet(current(1), current(2)) = true;
    expandedCount = expandedCount + 1;

    if delay > 0
        exploredNodes(end + 1, :) = current;
        if mod(size(exploredNodes, 1), 5) == 0
            plot(current(2) - 0.5, current(1) - 0.5, 's', ...
                'MarkerSize', 3, 'MarkerFaceColor', [0.8, 0.8, 0.8], ...
                'MarkerEdgeColor', 'none');
            drawnow;
            pause(delay);
        end
    end

    % 扩展邻域
    for d = 1:8
        nr = current(1) + dRow(d);
        nc = current(2) + dCol(d);

        if nr < 1 || nr > n || nc < 1 || nc > n, continue; end
        if closedSet(nr, nc), continue; end
        if occGrid(nr, nc), continue; end

        if dRow(d) ~= 0 && dCol(d) ~= 0
            if occGrid(current(1) + dRow(d), current(2)) || ...
               occGrid(current(1), current(2) + dCol(d))
                continue;
            end
        end

        tentG = gScore(current(1), current(2)) + moveCost(d);
        if tentG < gScore(nr, nc)
            gScore(nr, nc) = tentG;

            % 按需计算障碍物距离并加权
            dn = computeLocalDist(nr, nc);
            wn = 1 + alpha * (1 - exp(-dn / d_actual));
            newH = h(nr, nc);
            newF = tentG + wn * newH;

            hScore(nr, nc) = newH;
            fScore(nr, nc) = newF;
            parent(nr, nc, :) = current;

            pos = heapPos(nr, nc);
            if pos == 0
                heapSize = heapSize + 1;
                heap(heapSize, :) = [newF, newH, nr, nc];
                heapPos(nr, nc) = heapSize;
                bubbleUp(heapSize);
                if heapSize > openMaxSize, openMaxSize = heapSize; end
            else
                heap(pos, :) = [newF, newH, nr, nc];
                bubbleUp(pos);
            end
        end
    end
end

if ~isempty(bestPath)
    path = bestPath;
    info = struct('expandedNodes', expandedCount, ...
                  'pathLength', size(path, 1), ...
                  'pathCost', bestGoalG, ...
                  'openMaxSize', openMaxSize, ...
                  'distComputeCount', distComputeCount);
    if ~isempty(callback)
        callback(struct('type', 'finish', 'path', path, 'iteration', iter, 'success', true));
    end
else
    info = struct('expandedNodes',expandedCount,'pathLength',0,'pathCost',inf,'openMaxSize',openMaxSize, ...
                  'distComputeCount', distComputeCount);
    if ~isempty(callback)
        callback(struct('type', 'finish', 'path', [], 'iteration', iter, 'success', false));
    end
    path = [];
end

% ======================== 内联距离计算函数 ========================

    function dist = computeLocalDist(r, c)
        % 从 (r,c) 向外逐层搜索最近障碍物（带缓存）
        if ~isnan(distCache(r, c))
            dist = distCache(r, c);
            return;
        end

        distComputeCount = distComputeCount + 1;
        searchDist = maxSearchR;  % 默认最大搜索距离

        for layer = 1:maxSearchR
            found = false;
            for dr = -layer:layer
                for dc = -layer:layer
                    if max(abs(dr), abs(dc)) ~= layer, continue; end
                    nr2 = r + dr;
                    nc2 = c + dc;
                    if nr2 < 1 || nr2 > n || nc2 < 1 || nc2 > n, continue; end
                    if occGrid(nr2, nc2)
                        searchDist = sqrt(dr^2 + dc^2);
                        found = true;
                        break;
                    end
                end
                if found, break; end
            end
            if found, break; end
        end

        distCache(r, c) = searchDist;
        dist = searchDist;
    end

% ======================== 堆操作 ========================

    function bubbleUp(idx)
        while idx > 1
            parentIdx = floor(idx / 2);
            if compareHeap(idx, parentIdx) < 0
                swapHeap(idx, parentIdx);
                idx = parentIdx;
            else
                break;
            end
        end
    end

    function bubbleDown(idx)
        while true
            smallest = idx;
            left = 2 * idx;
            right = 2 * idx + 1;
            if left <= heapSize && compareHeap(left, smallest) < 0
                smallest = left;
            end
            if right <= heapSize && compareHeap(right, smallest) < 0
                smallest = right;
            end
            if smallest ~= idx
                swapHeap(idx, smallest);
                idx = smallest;
            else
                break;
            end
        end
    end

    function cmp = compareHeap(i, j)
        if heap(i, 1) < heap(j, 1)
            cmp = -1;
        elseif heap(i, 1) > heap(j, 1)
            cmp = 1;
        elseif heap(i, 2) < heap(j, 2)
            cmp = -1;
        elseif heap(i, 2) > heap(j, 2)
            cmp = 1;
        else
            cmp = 0;
        end
    end

    function swapHeap(i, j)
        ri = heap(i, 3); ci = heap(i, 4);
        rj = heap(j, 3); cj = heap(j, 4);
        tmp = heap(i, :);
        heap(i, :) = heap(j, :);
        heap(j, :) = tmp;
        heapPos(ri, ci) = j;
        heapPos(rj, cj) = i;
    end

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
