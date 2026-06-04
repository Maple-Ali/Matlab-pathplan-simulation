function path = AStar_v3(map, startGrid, goalGrid, delay, callback)
%ASTAR_V3 A* 全局路径规划 — 改进版 v3（Jump Point Search）
%   path = AStar_v3(map, startGrid, goalGrid, delay, callback)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组
%
%   改进内容（相对原始 AStar）：
%     1. 二叉堆优先队列替代线性扫描 — O(log n) 取最小 f
%     2. 自适应指数加权启发式 — 远距离加大权重加速搜索，近距离趋于标准 A*
%     3. Jump Point Search — 利用栅格对称性跳过冗余节点，仅扩展跳点
%     4. 改进的 tie-breaking — f 相同时优先选 h 更小的节点
%
%   JPS 原理：
%     - 直线方向：一直跳到出现强制邻居或障碍/边界
%     - 对角线方向：跳到出现强制邻居，同时递归检测两个分量方向的跳点
%     - 强制邻居：因障碍物阻挡，必须从当前路径经过才能到达的栅格

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

% 起终点距离（用于归一化权重）
startDist = sqrt((startGrid(1) - goalGrid(1))^2 + (startGrid(2) - goalGrid(2))^2);
if startDist == 0
    path = startGrid; return;
end

% ---- 自适应加权启发式（继承自 v1） ----
alpha = 1.0;
beta  = 3.0;
h = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);
hWeight = @(r, c) 1 + alpha * exp(-beta * (1 - h(r, c) / startDist));
weightedH = @(r, c) hWeight(r, c) * h(r, c);

% ---- 二叉堆优先队列（继承自 v1） ----
gScore = inf(n, n);
gScore(startGrid(1), startGrid(2)) = 0;
fScore = inf(n, n);
fScore(startGrid(1), startGrid(2)) = weightedH(startGrid(1), startGrid(2));
hScore = inf(n, n);
hScore(startGrid(1), startGrid(2)) = h(startGrid(1), startGrid(2));
parent = zeros(n, n, 2);
heapSize = 0;
heap = zeros(n * n, 4);
heapPos = zeros(n, n);

% 插入起点
heapSize = 1;
heap(1, :) = [fScore(startGrid(1), startGrid(2)), ...
              h(startGrid(1), startGrid(2)), ...
              startGrid(1), startGrid(2)];
heapPos(startGrid(1), startGrid(2)) = 1;

% 已展开节点 + 父方向记录
closedSet = false(n, n);
% parentDir: 记录从父节点到当前节点的移动方向 [dr, dc]
% 用于 neighbor pruning 和跳点检测。起点无父方向，用 [0, 0] 表示。
parentDirR = zeros(n, n);
parentDirC = zeros(n, n);

iter = 0;

% 可视化
if delay > 0
    figure(gcf); hold on;
    exploredNodes = [];
end

% ---- 主循环 ----
while heapSize > 0
    % 取堆顶
    cr = heap(1, 3);
    cc = heap(1, 4);
    current = [cr, cc];

    % 弹出堆顶
    heapPos(cr, cc) = 0;
    heap(1, :) = heap(heapSize, :);
    heapPos(heap(heapSize, 3), heap(heapSize, 4)) = 1;
    heapSize = heapSize - 1;
    if heapSize > 0
        bubbleDown(1);
    end

    % callback 报告
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
            path = []; return;
        end
    end

    % 到达目标
    if cr == goalGrid(1) && cc == goalGrid(2)
        path = reconstructJPSPath(parent, startGrid, goalGrid);
        if ~isempty(callback)
            callback(struct('type', 'finish', 'path', path, ...
                'iteration', iter, 'success', true));
        end
        return;
    end

    closedSet(cr, cc) = true;

    % 可视化
    if delay > 0
        exploredNodes(end + 1, :) = current;
        if mod(size(exploredNodes, 1), 3) == 0
            plot(cc - 0.5, cr - 0.5, 's', ...
                'MarkerSize', 4, 'MarkerFaceColor', [0.3, 0.7, 0.3], ...
                'MarkerEdgeColor', 'none');
            drawnow;
            pause(delay);
        end
    end

    % ---- JPS 核心：识别后继搜索方向并跳点 ----
    pdr = parentDirR(cr, cc);
    pdc = parentDirC(cr, cc);

    % 获取修剪后的邻居方向列表
    dirs = pruneNeighbors(cr, cc, pdr, pdc);

    for k = 1:size(dirs, 1)
        dr = dirs(k, 1);
        dc = dirs(k, 2);

        jp = jump(cr, cc, dr, dc);
        if ~isempty(jp)
            jr = jp(1);
            jc = jp(2);

            if closedSet(jr, jc)
                continue;
            end

            % 计算从 current 到跳点的欧氏距离
            dist = sqrt((jr - cr)^2 + (jc - cc)^2);
            % g 代价用距离乘以相应移动成本
            if dr ~= 0 && dc ~= 0
                edgeCost = dist * sqrt(2) / sqrt(2);  % 大致对角线
                edgeCost = dist;  % 简化为纯欧氏
            else
                edgeCost = dist;
            end
            % 重新计算：逐段使用正确的代价
            edgeCost = jumpPathCost(cr, cc, dr, dc, jr, jc);

            tentG = gScore(cr, cc) + edgeCost;

            if tentG < gScore(jr, jc)
                gScore(jr, jc) = tentG;
                newH = h(jr, jc);
                newF = tentG + weightedH(jr, jc);

                fScore(jr, jc) = newF;
                hScore(jr, jc) = newH;
                parent(jr, jc, :) = [cr, cc];
                parentDirR(jr, jc) = dr;
                parentDirC(jr, jc) = dc;

                pos = heapPos(jr, jc);
                if pos == 0
                    heapSize = heapSize + 1;
                    heap(heapSize, :) = [newF, newH, jr, jc];
                    heapPos(jr, jc) = heapSize;
                    bubbleUp(heapSize);
                else
                    heap(pos, :) = [newF, newH, jr, jc];
                    bubbleUp(pos);
                end

                % 可视化跳点
                if delay > 0
                    plot(jc - 0.5, jr - 0.5, 'o', ...
                        'MarkerSize', 6, 'MarkerEdgeColor', [0.2, 0.6, 0.2], ...
                        'LineWidth', 1.5);
                    drawnow;
                    pause(delay * 0.5);
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

% ==================== JPS 核心函数 ====================

    function dirs = pruneNeighbors(r, c, pdr, pdc)
    %PRUNENEIGHBORS 邻居修剪：根据父方向确定需要搜索的方向
    %   基于 JPS 标准修剪规则
        if pdr == 0 && pdc == 0
            % 起点：搜索所有 8 个方向
            dirs = [-1,-1; -1,0; -1,1;  0,-1;  0,1;  1,-1;  1,0;  1,1];
            return;
        end

        dirs = [];

        if pdr ~= 0 && pdc ~= 0
            % 对角线移动 (pdr, pdc)
            % 自然邻居：继续对角线 + 两个分量直线方向
            dirs = [pdr, pdc;    % 继续对角线
                    pdr, 0;      % 垂直分量
                    0,   pdc];   % 水平分量

            % 强制邻居检测
            % 如果 cell(r-pdr, c) 被阻挡 → (pdr, -pdc) 为强制邻居
            if isBlocked(r - pdr, c)
                dirs = [dirs; pdr, -pdc];
            end
            % 如果 cell(r, c-pdc) 被阻挡 → (-pdr, pdc) 为强制邻居
            if isBlocked(r, c - pdc)
                dirs = [dirs; -pdr, pdc];
            end

        elseif pdr ~= 0
            % 垂直直线移动 (pdr, 0)
            dirs = [pdr, 0];  % 自然邻居：继续往前

            % 强制对角线邻居
            for dcs = [-1, 1]
                % 如果侧方被阻挡且前方对角空闲 → 强制对角
                if isBlocked(r, c + dcs) && ~isBlocked(r + pdr, c + dcs)
                    dirs = [dirs; pdr, dcs];
                end
            end

        elseif pdc ~= 0
            % 水平直线移动 (0, pdc)
            dirs = [0, pdc];  % 自然邻居：继续往前

            % 强制对角线邻居
            for drs = [-1, 1]
                % 如果侧方被阻挡且前方对角空闲 → 强制对角
                if isBlocked(r + drs, c) && ~isBlocked(r + drs, c + pdc)
                    dirs = [dirs; drs, pdc];
                end
            end
        end
    end

    function blocked = isBlocked(r, c)
    %ISBLOCKED 判断栅格是否被阻挡（含越界）
        if r < 1 || r > n || c < 1 || c > n
            blocked = true;
        else
            blocked = occGrid(r, c);
        end
    end

    function jp = jump(r, c, dr, dc)
    %JUMP 递归跳点函数
    %   从 (r, c) 沿方向 (dr, dc) 跳跃，返回遇到的第一个跳点或空
        nr = r + dr;
        nc = c + dc;

        % 遇到障碍或边界
        if nr < 1 || nr > n || nc < 1 || nc > n || occGrid(nr, nc)
            jp = [];
            return;
        end

        % 到达目标
        if nr == goalGrid(1) && nc == goalGrid(2)
            jp = [nr, nc];
            return;
        end

        % 检测强制邻居
        if hasForcedNeighbor(nr, nc, dr, dc)
            jp = [nr, nc];
            return;
        end

        % 对角线移动：分量方向先跳
        if dr ~= 0 && dc ~= 0
            % 先检查水平分量 (0, dc)
            if ~isempty(jump(nr, nc, 0, dc))
                jp = [nr, nc];
                return;
            end
            % 再检查垂直分量 (dr, 0)
            if ~isempty(jump(nr, nc, dr, 0))
                jp = [nr, nc];
                return;
            end
        end

        % 继续沿同一方向跳跃
        jp = jump(nr, nc, dr, dc);
    end

    function result = hasForcedNeighbor(r, c, dr, dc)
    %HASFORCEDNEIGHBOR 检测在 (r, c) 沿方向 (dr, dc) 移动时是否存在强制邻居
    %   强制邻居定义：邻居节点从父节点（沿反方向）无法到达，
    %   但可以从当前节点通过对角线到达
        result = false;

        if dr ~= 0 && dc ~= 0
            % 对角线：检测两个强制邻居条件
            % 强制邻居1：(dr, -dc) — cell(r-dr, c) 被挡且 (r-dr, c-dc) 空闲
            if isBlocked(r - dr, c) && ~isBlocked(r - dr, c - dc)
                result = true; return;
            end
            % 强制邻居2：(-dr, dc) — cell(r, c-dc) 被挡且 (r-dr, c-dc) 空闲
            if isBlocked(r, c - dc) && ~isBlocked(r - dr, c - dc)
                result = true; return;
            end

        elseif dr ~= 0
            % 垂直直线：检测水平两侧
            for dcs = [-1, 1]
                if isBlocked(r, c + dcs) && ~isBlocked(r + dr, c + dcs)
                    result = true; return;
                end
            end

        elseif dc ~= 0
            % 水平直线：检测垂直两侧
            for drs = [-1, 1]
                if isBlocked(r + drs, c) && ~isBlocked(r + drs, c + dc)
                    result = true; return;
                end
            end
        end
    end

    function cost = jumpPathCost(r, c, dr, dc, jr, jc)
    %JUMPPATHCOST 计算从 (r,c) 跳到 (jr,jc) 的实际移动代价
    %   沿直线路径逐格累加
        cost = 0;
        cr2 = r; cc2 = c;
        while ~(cr2 == jr && cc2 == jc)
            cr2 = cr2 + dr;
            cc2 = cc2 + dc;
            if dr ~= 0 && dc ~= 0
                cost = cost + sqrt(2);
            else
                cost = cost + 1;
            end
        end
    end

% ==================== 堆操作（嵌套函数） ====================

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

end  % AStar_v3 主函数结束

% ==================== 路径重建（JPS 插值） ====================

function path = reconstructJPSPath(parent, startGrid, goalGrid)
%RECONSTRUCTJPSPATH 重建完整路径（跳点之间线性插值）
%   JPS 的 parent 记录的是跳点之间的跳跃关系，需要沿直线插值补充中间格子

    % 先反向追溯跳点序列
    jps = goalGrid;
    current = goalGrid;
    while ~(current(1) == startGrid(1) && current(2) == startGrid(2))
        current = squeeze(parent(current(1), current(2), :))';
        if all(current == 0)
            path = [];
            return;
        end
        jps = [current; jps];  %#ok<AGROW>
    end

    % 逐对跳点之间进行直线插值
    path = jps(1, :);
    for i = 1:size(jps, 1) - 1
        p1 = jps(i, :);
        p2 = jps(i + 1, :);

        dr = sign(p2(1) - p1(1));
        dc = sign(p2(2) - p1(2));
        cr = p1(1);
        cc = p1(2);

        while ~(cr == p2(1) && cc == p2(2))
            cr = cr + dr;
            cc = cc + dc;
            path = [path; cr, cc];  %#ok<AGROW>
        end
    end
end
