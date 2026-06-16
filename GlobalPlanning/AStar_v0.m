function path = AStar_v0(map, startGrid, goalGrid, delay, callback)
%ASTAR_V0 A* 全局路径规划 — 改进版 v0（仅二叉堆优化）
%   path = AStar_v0(map, startGrid, goalGrid, delay, callback)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组
%
%   改进内容（相对原始 AStar）：
%     1. 二叉堆优先队列替代线性扫描 — O(log n) 取最小 f
%        （仅此一项改动，方便与原始 AStar 对比性能）

if nargin < 4
    delay = 0;
end
if nargin < 5
    callback = [];
end

n = map.mapSize;
occGrid = map.getOccupancyGrid();

% 检查起终点合法性
if occGrid(startGrid(1), startGrid(2)) || occGrid(goalGrid(1), goalGrid(2))
    path = [];
    return;
end

% 8邻域方向: 上下左右 + 对角线
dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

% 启发式：欧氏距离（与原始 AStar 完全一致）
h = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);

% g 值矩阵
gScore = inf(n, n);
gScore(startGrid(1), startGrid(2)) = 0;

% f = g + h
fScore = inf(n, n);
fScore(startGrid(1), startGrid(2)) = h(startGrid(1), startGrid(2));

% 父节点记录
parent = zeros(n, n, 2);

% ---- 二叉堆优先队列（替代原始 AStar 的逻辑矩阵 + 线性扫描）----
%   堆元素: [f, row, col]
%   按 f 排序（无 tie-breaking，与原始 AStar 行为一致）
%   heapPos 矩阵记录每个节点在堆中的位置（0 = 不在堆中）
heapSize = 0;
heap = zeros(n * n, 3);  % 预分配最大可能大小
heapPos = zeros(n, n);   % 节点在堆中的索引

% 插入起点
heapSize = heapSize + 1;
heap(heapSize, :) = [fScore(startGrid(1), startGrid(2)), ...
                     startGrid(1), startGrid(2)];
heapPos(startGrid(1), startGrid(2)) = heapSize;
bubbleUp(heapSize);

% 已展开节点
closedSet = false(n, n);

% 迭代计数（供 callback 使用）
iter = 0;

% 可视化准备
if delay > 0
    figure(gcf);
    hold on;
    exploredNodes = [];
end

while heapSize > 0
    % 取堆顶（f 最小）
    topRow = heap(1, 2);
    topCol = heap(1, 3);
    current = [topRow, topCol];

    % 从堆中移除
    heapPos(topRow, topCol) = 0;
    heap(1, :) = heap(heapSize, :);
    heapPos(heap(heapSize, 2), heap(heapSize, 3)) = 1;
    heapSize = heapSize - 1;
    if heapSize > 0
        bubbleDown(1);
    end

    % ---- callback 模式：向 GUI 报告当前状态 ----
    if ~isempty(callback)
        iter = iter + 1;
        % 从堆中提取 openSet（兼容 callback 接口）
        openSet = false(n, n);
        for i = 1:heapSize
            openSet(heap(i, 2), heap(i, 3)) = true;
        end
        stateInfo = struct('type', 'step', ...
            'current', current, ...
            'openSet', openSet, 'closedSet', closedSet, ...
            'gScore', gScore, 'fScore', fScore, 'iteration', iter);
        action = callback(stateInfo);
        if strcmp(action, 'stop')
            path = []; return;
        end
    end

    % 到达目标
    if current(1) == goalGrid(1) && current(2) == goalGrid(2)
        path = reconstructPath(parent, startGrid, goalGrid);
        if ~isempty(callback)
            callback(struct('type', 'finish', 'path', path, ...
                'iteration', iter, 'success', true));
        end
        return;
    end

    closedSet(current(1), current(2)) = true;

    % 可视化探索
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

        if nr < 1 || nr > n || nc < 1 || nc > n
            continue;
        end
        if closedSet(nr, nc)
            continue;
        end
        if occGrid(nr, nc)
            continue;
        end

        % 对角线移动需检查相邻两格
        if dRow(d) ~= 0 && dCol(d) ~= 0
            if occGrid(current(1) + dRow(d), current(2)) || ...
               occGrid(current(1), current(2) + dCol(d))
                continue;
            end
        end

        tentG = gScore(current(1), current(2)) + moveCost(d);
        if tentG < gScore(nr, nc)
            gScore(nr, nc) = tentG;
            newF = tentG + h(nr, nc);
            fScore(nr, nc) = newF;
            parent(nr, nc, :) = current;

            pos = heapPos(nr, nc);
            if pos == 0
                % 新节点：插入堆
                heapSize = heapSize + 1;
                heap(heapSize, :) = [newF, nr, nc];
                heapPos(nr, nc) = heapSize;
                bubbleUp(heapSize);
            else
                % 已在堆中：更新 f 并上浮
                heap(pos, :) = [newF, nr, nc];
                bubbleUp(pos);
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

% ======================== 堆操作（嵌套函数，共享工作区） ========================

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
        % 仅按 f 值比较（与原始 AStar 行为一致，无 tie-breaking）
        if heap(i, 1) < heap(j, 1)
            cmp = -1;
        elseif heap(i, 1) > heap(j, 1)
            cmp = 1;
        else
            cmp = 0;
        end
    end

    function swapHeap(i, j)
        ri = heap(i, 2); ci = heap(i, 3);
        rj = heap(j, 2); cj = heap(j, 3);

        tmp = heap(i, :);
        heap(i, :) = heap(j, :);
        heap(j, :) = tmp;

        heapPos(ri, ci) = j;
        heapPos(rj, cj) = i;
    end

end  % AStar_v0 主函数结束

% ======================== 路径重建 ========================

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
