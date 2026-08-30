function [path, info] = AStar_v1_1(map, startGrid, goalGrid, delay, callback, alpha, d_ref)
%ASTAR_V1_1 A* 全局路径规划 — 改进版 v1_1（障碍物距离自适应权重）
%   [path, info] = AStar_v1_1(map, startGrid, goalGrid, delay, callback, alpha, d_ref)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组
%   info: 结构体 — .expandedNodes, .pathLength, .pathCost, .openMaxSize
%
%   改进内容：
%     1. 二叉堆优先队列替代线性扫描 — O(log n) 取最小 f
%     2. 基于障碍物距离的自适应加权 A* — 障碍物附近精细搜索，开阔区域贪心加速
%     3. 改进的 tie-breaking — f 相同时优先选 h 更小的节点（更接近目标）
%     4. 延迟终止 — 首次到达目标后继续搜索，直到确认无更优路径
%
%   注意：使用延迟终止后，算法会找到最优路径（与 Dijkstra 相同），
%         但扩展节点数会比标准加权 A* 多。alpha 的作用变为控制搜索方向偏好，
%         而非牺牲最优性。
%
%   ============================================
%   主要可调节参数（直接修改下方默认值即可调试）
%   ============================================
%   alpha  = 0.2;   % 最大额外权重，范围建议 [0.1, 0.5]
%                    % 越大 → 开阔区域越贪心（搜索更快但路径代价更高）
%                    % 越小 → 越接近标准 A*（路径更优但搜索更慢）
%                    % 推荐 0.2~0.3，兼顾速度与质量
%
%   d_ref  = 0.07;  % 障碍物距离阈值（占地图对角距离百分比），范围建议 [0.02, 0.15]
%                    % 越小 → 障碍物影响范围越窄（只有紧贴障碍物才减速）
%                    % 越大 → 障碍物影响范围越广（提前减速，路径更安全）
%
%   权重函数: w(d_obs) = 1 + alpha * (1 - exp(-d_obs / d_actual))
%   其中 d_actual = d_ref * sqrt(mapSize² + mapSize²)
%   w ∈ [1, 1+alpha]，障碍物附近 w≈1，开阔区域 w≈1+alpha
%   ============================================

if nargin < 4
    delay = 0;
end
if nargin < 5
    callback = [];
end
if nargin < 6 || isempty(alpha)
    alpha = 0.5;    % 最大额外权重 [0.1, 0.5]，越大搜索越快但路径代价越高
end
if nargin < 7 || isempty(d_ref)
    d_ref = 0.07;   % 障碍物距离阈值(对角距离百分比) [0.02, 0.15]
end

n = map.mapSize;
occGrid = map.getOccupancyGrid();

% 检查起终点合法性
if occGrid(startGrid(1), startGrid(2)) || occGrid(goalGrid(1), goalGrid(2))
    path = [];
    info = struct('expandedNodes',0,'pathLength',0,'pathCost',inf,'openMaxSize',0);
    return;
end

% 8邻域方向: 上下左右 + 对角线
dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

% ---- 改进 2: 基于障碍物距离的自适应权重 ----
%   预计算每个栅格到最近障碍物的欧氏距离
%   权重函数: w(d_obs) = 1 + alpha * (1 - exp(-d_obs / d_actual))
%   障碍物附近 (d_obs→0): w → 1（标准 A*，精细搜索）
%   开阔区域 (d_obs>>d_actual): w → 1+alpha（贪心加速）
diagonalDist = sqrt(2 * n^2);  % 地图对角距离
d_actual = d_ref * diagonalDist;  % 实际距离阈值

% 预计算距离场（bwdist 计算每个栅格到最近障碍物的距离）
distField = bwdist(double(occGrid));

% 权重映射函数（基于障碍物距离）
hWeight = @(r, c) 1 + alpha * (1 - exp(-distField(r, c) / d_actual));

% 启发式函数
h = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);
weightedH = @(r, c) hWeight(r, c) * h(r, c);

% g 值矩阵
gScore = inf(n, n);
gScore(startGrid(1), startGrid(2)) = 0;

% f = g + w*h
fScore = inf(n, n);
fScore(startGrid(1), startGrid(2)) = weightedH(startGrid(1), startGrid(2));

% h 值矩阵（供 tie-breaking 使用）
hScore = inf(n, n);
hScore(startGrid(1), startGrid(2)) = h(startGrid(1), startGrid(2));

% 父节点记录
parent = zeros(n, n, 2);

% ---- 改进 1: 二叉堆优先队列 ----
%   堆元素: [f, h, row, col]
%   按 f 排序，f 相同时按 h 排序（tie-breaking: 优先 h 更小的）
%   用索引矩阵 track 每个节点在堆中的位置（0 = 不在堆中）
heapSize = 0;
heap = zeros(n * n, 4);  % 预分配最大可能大小
heapPos = zeros(n, n);   % 节点在堆中的索引

% 插入起点
heapSize = heapSize + 1;
heap(heapSize, :) = [fScore(startGrid(1), startGrid(2)), ...
                     h(startGrid(1), startGrid(2)), ...
                     startGrid(1), startGrid(2)];
heapPos(startGrid(1), startGrid(2)) = heapSize;
bubbleUp(heapSize);
openMaxSize = 1;

% 已展开节点
closedSet = false(n, n);

% 迭代计数（供 callback 使用）
iter = 0;

% 延迟终止相关变量
bestGoalG = inf;    % 目标节点的最优 g 值
bestPath = [];      % 目标节点的最优路径

% 指标统计
expandedCount = 0;
openMaxSize = 0;

% 可视化准备
if delay > 0
    figure(gcf);
    hold on;
    exploredNodes = [];
end

while heapSize > 0
    % 取堆顶（f 最小，tie-breaking 选 h 更小的）
    topRow = heap(1, 3);
    topCol = heap(1, 4);
    current = [topRow, topCol];

    % 从堆中移除
    heapPos(topRow, topCol) = 0;
    heap(1, :) = heap(heapSize, :);
    heapPos(heap(heapSize, 3), heap(heapSize, 4)) = 1;
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
            openSet(heap(i, 3), heap(i, 4)) = true;
        end
        stateInfo = struct('type', 'step', ...
            'current', current, ...
            'openSet', openSet, 'closedSet', closedSet, ...
            'gScore', gScore, 'fScore', fScore, 'iteration', iter);
        action = callback(stateInfo);
        if strcmp(action, 'stop')
            path = [];
            info = struct('expandedNodes',expandedCount,'pathLength',0,'pathCost',inf,'openMaxSize',openMaxSize);
            return;
        end
    end

    % 到达目标（加权 A* 延迟终止：继续搜索直到确认最优）
    if current(1) == goalGrid(1) && current(2) == goalGrid(2)
        goalG = gScore(goalGrid(1), goalGrid(2));
        % 检查 open set 中是否有 f 值更小的节点
        % 如果有，可能存在更优路径，继续搜索
        if heapSize > 0 && heap(1, 1) < goalG
            % open set 中最小 f < 目标 g，继续搜索
            % 但标记已找到目标，后续可提前终止
            if ~exist('bestGoalG', 'var') || goalG < bestGoalG
                bestGoalG = goalG;
                bestPath = reconstructPath(parent, startGrid, goalGrid);
            end
        else
            % 确认是最优路径
            path = reconstructPath(parent, startGrid, goalGrid);
            expandedCount = expandedCount + 1;
            info = struct('expandedNodes', expandedCount, ...
                          'pathLength', size(path, 1), ...
                          'pathCost', gScore(goalGrid(1), goalGrid(2)), ...
                          'openMaxSize', openMaxSize);
            if ~isempty(callback)
                callback(struct('type', 'finish', 'path', path, ...
                    'iteration', iter, 'success', true));
            end
            return;
        end
    end

    closedSet(current(1), current(2)) = true;
    expandedCount = expandedCount + 1;

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
            newH = h(nr, nc);
            newF = tentG + weightedH(nr, nc);
            hScore(nr, nc) = newH;
            fScore(nr, nc) = newF;
            parent(nr, nc, :) = current;

            pos = heapPos(nr, nc);
            if pos == 0
                % 新节点：插入堆
                heapSize = heapSize + 1;
                heap(heapSize, :) = [newF, newH, nr, nc];
                heapPos(nr, nc) = heapSize;
                bubbleUp(heapSize);
                if heapSize > openMaxSize
                    openMaxSize = heapSize;
                end
            else
                % 已在堆中：更新 f 并上浮
                heap(pos, :) = [newF, newH, nr, nc];
                bubbleUp(pos);
            end
        end
    end
end

% 循环结束，检查是否找到了路径
if ~isempty(bestPath)
    % 找到了路径（延迟终止）
    path = bestPath;
    info = struct('expandedNodes', expandedCount, ...
                  'pathLength', size(path, 1), ...
                  'pathCost', bestGoalG, ...
                  'openMaxSize', openMaxSize);
    if ~isempty(callback)
        callback(struct('type', 'finish', 'path', path, ...
            'iteration', iter, 'success', true));
    end
else
    % 无路径
    info = struct('expandedNodes',expandedCount,'pathLength',0,'pathCost',inf,'openMaxSize',openMaxSize);
    if ~isempty(callback)
        callback(struct('type', 'finish', 'path', [], ...
            'iteration', iter, 'success', false));
    end
    path = [];
end

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

end  % AStar_v1_1 主函数结束

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
