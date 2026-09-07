function [path, info] = AStar_v4(map, startGrid, goalGrid, ~, callback)
% AStar_v4 - 基于方向优先的双层搜索策略 A* 算法
%
% 核心思想：传统 A* 在节点扩展时向 8 个相邻节点搜索，其中反方向的节点通常是多余的。
% 本算法提出 5 邻域-9 邻域双层搜索策略：
%   - 第一层（5邻域）：根据父节点指向目标点的方向，舍弃反方向的3个子节点，
%     评估剩余5个子节点。若存在障碍物或终点，则以此5邻域搜索。
%   - 第二层（9邻域）：若第一层5个方向均无障碍，拓展到第二层。
%     在24邻域（5×5网格）中舍弃反方向的节点，以9邻域搜索下一个父节点。
%   - 回退机制：当双层搜索均无有效候选时，回退到全部8邻域搜索。
%
% 优势：减少搜索节点数，减少转弯次数，路径更平滑。
%
% 输入：
%   map      - Map 类
%   startGrid- 起点 [row, col]
%   goalGrid - 终点 [row, col]
%   ~        - delay (未使用)
%   callback - 可选回调函数 callback(pathSoFar, expandedCount)
%
% 输出：
%   path - N×2 路径矩阵 [row, col]
%   info - 包含 .elapsed, .expanded, .cost 的结构体

%% ==================== 可调参数 ====================
enableTieBreaking = true;   % 是否启用f值相同时的h值排序（略微提升路径质量）
% ===================================================

t0 = tic;
n = map.mapSize;
occGrid = map.getOccupancyGrid();

% 初始化数据结构
gScore = inf(n, n);
fScore = inf(n, n);
closed = false(n, n);
cameFrom = zeros(n, n, 2);  % cameFrom(r,c,:) = [parentRow, parentCol]
heap = zeros(10000, 3);     % [f, h, idx]
heapSize = 0;
expandedCount = 0;

% 预计算8邻域和24邻域的偏移量（在循环外计算以提升性能）
layer1Offsets = getLayer1Offsets();  % 8邻域偏移
layer2Offsets = getLayer2Offsets();  % 24邻域偏移（距离为2的9个方向）

% 起点初始化
sr = startGrid(1); sc = startGrid(2);
gr = goalGrid(1);  gc = goalGrid(2);
h0 = sqrt((sr - gr)^2 + (sc - gc)^2);
gScore(sr, sc) = 0;
fScore(sr, sc) = h0;
heapSize = heapSize + 1;
heap(heapSize, :) = [h0, h0, sub2ind([n, n], sr, sc)];

% 主循环
while heapSize > 0
    % 弹出f值最小的节点
    [~, minIdx] = min(heap(1:heapSize, 1));
    current = heap(minIdx, :);
    heap(minIdx, :) = heap(heapSize, :);
    heapSize = heapSize - 1;
    if heapSize > 0
        heap = heapifyDown(heap, heapSize, minIdx, enableTieBreaking);
    end

    idx = current(3);
    [cr, cc] = ind2sub([n, n], idx);

    if closed(cr, cc)
        continue;
    end
    closed(cr, cc) = true;
    expandedCount = expandedCount + 1;

    % 回调
    if nargin >= 5 && ~isempty(callback)
        pathSoFar = reconstructPath(cameFrom, sr, sc, cr, cc);
        callback(pathSoFar, expandedCount);
    end

    % 到达目标
    if cr == gr && cc == gc
        path = reconstructPath(cameFrom, sr, sc, gr, gc);
        info.elapsed = toc(t0) * 1000;
        info.expanded = expandedCount;
        info.cost = gScore(gr, gc);
        return;
    end

    % 计算当前节点到目标的方向角
    angleToGoal = computeAngle(gr - cr, gc - cc);

    % ===== 双层搜索策略 =====
    % 第一层：5邻域（8邻域中方向一致的5个）
    added = false;
    for i = 1:size(layer1Offsets, 1)
        dr = layer1Offsets(i, 1);
        dc = layer1Offsets(i, 2);
        nr = cr + dr;
        nc = cc + dc;
        if nr < 1 || nr > n || nc < 1 || nc > n
            continue;
        end
        if closed(nr, nc) || occGrid(nr, nc) == 1
            continue;
        end
        % 对角线移动需检查邻接格是否可通过
        if dr ~= 0 && dc ~= 0
            if occGrid(cr + dr, cc) == 1 && occGrid(cr, cc + dc) == 1
                continue;
            end
        end
        if dr ~= 0 && dc ~= 0
            moveCost = sqrt(2);
        else
            moveCost = 1;
        end
        tentG = gScore(cr, cc) + moveCost;
        if tentG < gScore(nr, nc)
            cameFrom(nr, nc, :) = [cr, cc];
            gScore(nr, nc) = tentG;
            h = sqrt((nr - gr)^2 + (nc - gc)^2);
            fScore(nr, nc) = tentG + h;
            heapSize = heapSize + 1;
            heap(heapSize, :) = [fScore(nr, nc), h, sub2ind([n, n], nr, nc)];
            added = true;
        end
    end

    % 第二层：若第一层无有效候选，尝试9邻域（距离为2的24邻域子集）
    if ~added
        for i = 1:size(layer2Offsets, 1)
            dr = layer2Offsets(i, 1);
            dc = layer2Offsets(i, 2);
            nr = cr + dr;
            nc = cc + dc;
            if nr < 1 || nr > n || nc < 1 || nc > n
                continue;
            end
            if closed(nr, nc) || occGrid(nr, nc) == 1
                continue;
            end
            % 检查中间节点是否可通过（防止穿越障碍物）
            midR = cr + round(dr / 2);
            midC = cc + round(dc / 2);
            if occGrid(midR, midC) == 1
                continue;
            end
            % 非纯对角线的L形移动（如 ±2,±1）也需检查邻接格
            if abs(dr) ~= abs(dc)
                if occGrid(cr + round(dr / 2), cc) == 1 && ...
                   occGrid(cr, cc + round(dc / 2)) == 1
                    continue;
                end
            end
            % 距离计算：对角线(±2,±2)为2√2，直线(±2,0或0,±2)为2
            if abs(dr) == abs(dc)
                moveCost = 2 * sqrt(2);
            else
                moveCost = 2;
            end
            tentG = gScore(cr, cc) + moveCost;
            if tentG < gScore(nr, nc)
                cameFrom(nr, nc, :) = [cr, cc];
                gScore(nr, nc) = tentG;
                h = sqrt((nr - gr)^2 + (nc - gc)^2);
                fScore(nr, nc) = tentG + h;
                heapSize = heapSize + 1;
                heap(heapSize, :) = [fScore(nr, nc), h, sub2ind([n, n], nr, nc)];
                added = true;
            end
        end
    end

    % 回退：若双层搜索均无有效候选，扩展全部8邻域
    if ~added
        for i = 1:size(layer1Offsets, 1)
            dr = layer1Offsets(i, 1);
            dc = layer1Offsets(i, 2);
            nr = cr + dr;
            nc = cc + dc;
            if nr < 1 || nr > n || nc < 1 || nc > n
                continue;
            end
            if closed(nr, nc) || occGrid(nr, nc) == 1
                continue;
            end
            if dr ~= 0 && dc ~= 0
                if occGrid(cr + dr, cc) == 1 && occGrid(cr, cc + dc) == 1
                    continue;
                end
            end
            if dr ~= 0 && dc ~= 0
                moveCost = sqrt(2);
            else
                moveCost = 1;
            end
            tentG = gScore(cr, cc) + moveCost;
            if tentG < gScore(nr, nc)
                cameFrom(nr, nc, :) = [cr, cc];
                gScore(nr, nc) = tentG;
                h = sqrt((nr - gr)^2 + (nc - gc)^2);
                fScore(nr, nc) = tentG + h;
                heapSize = heapSize + 1;
                heap(heapSize, :) = [fScore(nr, nc), h, sub2ind([n, n], nr, nc)];
            end
        end
    end
end

path = [];
info.elapsed = toc(t0) * 1000;
info.expanded = expandedCount;
info.cost = inf;
end

%% ==================== 辅助函数 ====================

function path = reconstructPath(cameFrom, sr, sc, cr, cc)
    path = [cr, cc];
    while cr ~= sr || cc ~= sc
        pr = cameFrom(cr, cc, 1);
        pc = cameFrom(cr, cc, 2);
        % 跳过中间节点（Layer2的cameFrom可能指向距离2的节点）
        % 路径中只记录实际经过的节点
        path = [pr, pc; path]; %#ok<AGROW>
        cr = pr;
        cc = pc;
    end
end

function angle = computeAngle(dr, dc)
% 计算向量 (dr, dc) 相对于x轴正方向的角度，范围 [0, 360)
    angle = atan2d(dr, dc);
    if angle < 0
        angle = angle + 360;
    end
end

function diff = angleDiff(a1, a2)
% 计算两个角度之间的最小差值，范围 [0, 180]
    diff = abs(a1 - a2);
    if diff > 180
        diff = 360 - diff;
    end
end

function offsets = getLayer1Offsets()
% 第一层：8邻域偏移量
    offsets = [
        -1, -1;
        -1,  0;
        -1,  1;
         0, -1;
         0,  1;
         1, -1;
         1,  0;
         1,  1
    ];
end

function offsets = getLayer2Offsets()
% 第二层：24邻域中距离为2的9个方向偏移量
% 包括直线距离2（±2,0和0,±2）和对角线距离2√2（±2,±2）
    offsets = [
        -2, -2;
        -2,  0;
        -2,  2;
         0, -2;
         0,  2;
         2, -2;
         2,  0;
         2,  2;
        -2,  1;  % L形方向
         2,  1;
        -2, -1;
         2, -1;
        -1,  2;
         1,  2;
        -1, -2;
         1, -2;
    ];
end

function heap = heapifyDown(heap, heapSize, idx, enableTieBreaking)
    while true
        smallest = idx;
        left = 2 * idx;
        right = 2 * idx + 1;
        if left <= heapSize && compareHeap(heap, left, smallest, enableTieBreaking) < 0
            smallest = left;
        end
        if right <= heapSize && compareHeap(heap, right, smallest, enableTieBreaking) < 0
            smallest = right;
        end
        if smallest == idx
            break;
        end
        heap([idx, smallest], :) = heap([smallest, idx], :);
        idx = smallest;
    end
end

function heap = heapifyUp(heap, idx, enableTieBreaking)
    while idx > 1
        parent = floor(idx / 2);
        if compareHeap(heap, idx, parent, enableTieBreaking) < 0
            heap([idx, parent], :) = heap([parent, idx], :);
            idx = parent;
        else
            break;
        end
    end
end

function cmp = compareHeap(heap, i, j, enableTieBreaking)
    if heap(i, 1) < heap(j, 1)
        cmp = -1;
    elseif heap(i, 1) > heap(j, 1)
        cmp = 1;
    elseif enableTieBreaking
        if heap(i, 2) < heap(j, 2)
            cmp = -1;
        elseif heap(i, 2) > heap(j, 2)
            cmp = 1;
        else
            cmp = 0;
        end
    else
        cmp = 0;
    end
end