function path = AStar_v2(map, startGrid, goalGrid, delay, callback)
%ASTAR_V2 A* 全局路径规划 — 改进版 v2（双向搜索）
%   path = AStar_v2(map, startGrid, goalGrid, delay, callback)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组
%
%   改进内容（相对原始 AStar）：
%     1. 二叉堆优先队列替代线性扫描 — O(log n) 取最小 f
%     2. 自适应指数加权启发式 — 远距离加大权重加速搜索，近距离趋于标准 A*
%     3. 双向搜索 — 从起点和终点同时扩展，两个 closed set 相遇时合并路径
%     4. 交替扩展策略 — 每轮正向/反向各扩展一次
%     5. 改进的 tie-breaking — f 相同时优先选 h 更小的节点

% 可调参数
EXPANSION_BALANCE = 0.8;  % 正向扩展节点数占比阈值（>此比例则切到反向）

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

% 8邻域方向
dRow = [-1, -1, -1,  0,  0,  1,  1,  1];
dCol = [-1,  0,  1, -1,  1, -1,  0,  1];
moveCost = [sqrt(2), 1, sqrt(2), 1, 1, sqrt(2), 1, sqrt(2)];

% ---- 自适应加权启发式（从 v1 继承） ----
alpha = 1.0;
beta  = 3.0;

% 正向启发式：当前节点 → 目标
h_fwd = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);
w_fwd = @(r, c) 1 + alpha * exp(-beta * (1 - h_fwd(r, c) / startDist));

% 反向启发式：当前节点 → 起点
h_back = @(r, c) sqrt((r - startGrid(1))^2 + (c - startGrid(2))^2);
w_back = @(r, c) 1 + alpha * exp(-beta * (1 - h_back(r, c) / startDist));

% ---- 正向搜索数据结构 ----
fwdGScore = inf(n, n);
fwdGScore(startGrid(1), startGrid(2)) = 0;
fwdFScore = inf(n, n);
fwdFScore(startGrid(1), startGrid(2)) = w_fwd(startGrid(1), startGrid(2)) * h_fwd(startGrid(1), startGrid(2));
fwdParent = zeros(n, n, 2);
fwdClosed = false(n, n);
fwdHeapSize = 0;
fwdHeap = zeros(n * n, 4);
fwdHeapPos = zeros(n, n);

% 插入正向起点
fwdHeapSize = 1;
fwdHeap(1, :) = [fwdFScore(startGrid(1), startGrid(2)), ...
                 h_fwd(startGrid(1), startGrid(2)), ...
                 startGrid(1), startGrid(2)];
fwdHeapPos(startGrid(1), startGrid(2)) = 1;

% ---- 反向搜索数据结构 ----
backGScore = inf(n, n);
backGScore(goalGrid(1), goalGrid(2)) = 0;
backFScore = inf(n, n);
backFScore(goalGrid(1), goalGrid(2)) = w_back(goalGrid(1), goalGrid(2)) * h_back(goalGrid(1), goalGrid(2));
backParent = zeros(n, n, 2);
backClosed = false(n, n);
backHeapSize = 0;
backHeap = zeros(n * n, 4);
backHeapPos = zeros(n, n);

% 插入反向起点（即 goal）
backHeapSize = 1;
backHeap(1, :) = [backFScore(goalGrid(1), goalGrid(2)), ...
                  h_back(goalGrid(1), goalGrid(2)), ...
                  goalGrid(1), goalGrid(2)];
backHeapPos(goalGrid(1), goalGrid(2)) = 1;

% 迭代计数
iter = 0;
fwdExpanded = 0;   % 正向已扩展数
backExpanded = 0;  % 反向已扩展数

% 可视化
if delay > 0
    figure(gcf); hold on;
    exploredNodes = [];
end

% ---- 主循环 ----
while fwdHeapSize > 0 && backHeapSize > 0

    % ---- 智能选择扩展方向 ----
    % 优先扩展已扩展数较少的那一侧，保持平衡
    if fwdExpanded <= backExpanded * EXPANSION_BALANCE
        expandForward = true;
    elseif backExpanded <= fwdExpanded * EXPANSION_BALANCE
        expandForward = false;
    else
        % 基本平衡，交替扩展
        expandForward = mod(iter, 2) == 0;
    end

    if expandForward
        % ============ 正向扩展 ============
        current = popHeap();
        fwdClosed(current(1), current(2)) = true;
        fwdExpanded = fwdExpanded + 1;

        % 相遇检测：当前节点已在反向 closed 中？
        if backClosed(current(1), current(2))
            path = reconstructBidirectional(current);
            finishCallback(true); return;
        end

        for d = 1:8
            [nr, nc] = deal(current(1) + dRow(d), current(2) + dCol(d));
            if ~validCell(nr, nc) || fwdClosed(nr, nc) || occGrid(nr, nc)
                continue;
            end
            % 对角线检查
            if dRow(d) ~= 0 && dCol(d) ~= 0
                if occGrid(current(1) + dRow(d), current(2)) || ...
                   occGrid(current(1), current(2) + dCol(d))
                    continue;
                end
            end
            tentG = fwdGScore(current(1), current(2)) + moveCost(d);
            if tentG < fwdGScore(nr, nc)
                fwdGScore(nr, nc) = tentG;
                newH = h_fwd(nr, nc);
                newF = tentG + w_fwd(nr, nc) * newH;
                fwdFScore(nr, nc) = newF;
                fwdParent(nr, nc, :) = current;
                updateOrInsertHeap(newF, newH, nr, nc);
            end
        end

    else
        % ============ 反向扩展 ============
        current = popBackHeap();
        backClosed(current(1), current(2)) = true;
        backExpanded = backExpanded + 1;

        % 相遇检测：当前节点已在正向 closed 中？
        if fwdClosed(current(1), current(2))
            path = reconstructBidirectional(current);
            finishCallback(true); return;
        end

        for d = 1:8
            [nr, nc] = deal(current(1) + dRow(d), current(2) + dCol(d));
            if ~validCell(nr, nc) || backClosed(nr, nc) || occGrid(nr, nc)
                continue;
            end
            if dRow(d) ~= 0 && dCol(d) ~= 0
                if occGrid(current(1) + dRow(d), current(2)) || ...
                   occGrid(current(1), current(2) + dCol(d))
                    continue;
                end
            end
            tentG = backGScore(current(1), current(2)) + moveCost(d);
            if tentG < backGScore(nr, nc)
                backGScore(nr, nc) = tentG;
                newH = h_back(nr, nc);
                newF = tentG + w_back(nr, nc) * newH;
                backFScore(nr, nc) = newF;
                backParent(nr, nc, :) = current;
                updateOrInsertBackHeap(newF, newH, nr, nc);
            end
        end
    end

    % callback 报告（合并两个方向的 open/closed）
    if ~isempty(callback)
        iter = iter + 1;
        openSet = false(n, n);
        for i = 1:fwdHeapSize
            openSet(fwdHeap(i, 3), fwdHeap(i, 4)) = true;
        end
        for i = 1:backHeapSize
            openSet(backHeap(i, 3), backHeap(i, 4)) = true;
        end
        closedSet = fwdClosed | backClosed;
        gScore = min(fwdGScore, backGScore);  % 近似合并

        stateInfo = struct('type', 'step', ...
            'current', current, ...
            'openSet', openSet, 'closedSet', closedSet, ...
            'gScore', gScore, 'fScore', fwdFScore, 'iteration', iter);
        action = callback(stateInfo);
        if strcmp(action, 'stop')
            path = []; return;
        end
    end

    % 可视化
    if delay > 0
        exploredNodes(end + 1, :) = current;
        if mod(size(exploredNodes, 1), 5) == 0
            if expandForward
                plot(current(2) - 0.5, current(1) - 0.5, 's', ...
                    'MarkerSize', 3, 'MarkerFaceColor', [0.6, 0.8, 0.6], ...
                    'MarkerEdgeColor', 'none');
            else
                plot(current(2) - 0.5, current(1) - 0.5, 's', ...
                    'MarkerSize', 3, 'MarkerFaceColor', [0.8, 0.6, 0.6], ...
                    'MarkerEdgeColor', 'none');
            end
            drawnow;
            pause(delay);
        end
    end
end

% 无路径
finishCallback(false);
path = [];

% ======================== 辅助函数 ========================

    function ok = validCell(r, c)
        ok = r >= 1 && r <= n && c >= 1 && c <= n;
    end

% ==================== 正向堆操作 ====================

    function current = popHeap()
        r = fwdHeap(1, 3);
        c = fwdHeap(1, 4);
        current = [r, c];
        fwdHeapPos(r, c) = 0;
        fwdHeap(1, :) = fwdHeap(fwdHeapSize, :);
        fwdHeapPos(fwdHeap(fwdHeapSize, 3), fwdHeap(fwdHeapSize, 4)) = 1;
        fwdHeapSize = fwdHeapSize - 1;
        if fwdHeapSize > 0
            fwdBubbleDown(1);
        end
    end

    function updateOrInsertHeap(newF, newH, r, c)
        pos = fwdHeapPos(r, c);
        if pos == 0
            fwdHeapSize = fwdHeapSize + 1;
            fwdHeap(fwdHeapSize, :) = [newF, newH, r, c];
            fwdHeapPos(r, c) = fwdHeapSize;
            fwdBubbleUp(fwdHeapSize);
        else
            fwdHeap(pos, :) = [newF, newH, r, c];
            fwdBubbleUp(pos);
        end
    end

    function fwdBubbleUp(idx)
        while idx > 1
            p = floor(idx / 2);
            if fwdCompare(idx, p) < 0
                fwdSwap(idx, p);
                idx = p;
            else
                break;
            end
        end
    end

    function fwdBubbleDown(idx)
        while true
            s = idx;
            l = 2 * idx; r = 2 * idx + 1;
            if l <= fwdHeapSize && fwdCompare(l, s) < 0, s = l; end
            if r <= fwdHeapSize && fwdCompare(r, s) < 0, s = r; end
            if s ~= idx
                fwdSwap(idx, s);
                idx = s;
            else
                break;
            end
        end
    end

    function c = fwdCompare(i, j)
        if fwdHeap(i, 1) < fwdHeap(j, 1)
            c = -1;
        elseif fwdHeap(i, 1) > fwdHeap(j, 1)
            c = 1;
        elseif fwdHeap(i, 2) < fwdHeap(j, 2)
            c = -1;
        elseif fwdHeap(i, 2) > fwdHeap(j, 2)
            c = 1;
        else
            c = 0;
        end
    end

    function fwdSwap(i, j)
        ri = fwdHeap(i, 3); ci = fwdHeap(i, 4);
        rj = fwdHeap(j, 3); cj = fwdHeap(j, 4);
        tmp = fwdHeap(i, :);
        fwdHeap(i, :) = fwdHeap(j, :);
        fwdHeap(j, :) = tmp;
        fwdHeapPos(ri, ci) = j;
        fwdHeapPos(rj, cj) = i;
    end

% ==================== 反向堆操作 ====================

    function current = popBackHeap()
        r = backHeap(1, 3);
        c = backHeap(1, 4);
        current = [r, c];
        backHeapPos(r, c) = 0;
        backHeap(1, :) = backHeap(backHeapSize, :);
        backHeapPos(backHeap(backHeapSize, 3), backHeap(backHeapSize, 4)) = 1;
        backHeapSize = backHeapSize - 1;
        if backHeapSize > 0
            backBubbleDown(1);
        end
    end

    function updateOrInsertBackHeap(newF, newH, r, c)
        pos = backHeapPos(r, c);
        if pos == 0
            backHeapSize = backHeapSize + 1;
            backHeap(backHeapSize, :) = [newF, newH, r, c];
            backHeapPos(r, c) = backHeapSize;
            backBubbleUp(backHeapSize);
        else
            backHeap(pos, :) = [newF, newH, r, c];
            backBubbleUp(pos);
        end
    end

    function backBubbleUp(idx)
        while idx > 1
            p = floor(idx / 2);
            if backCompare(idx, p) < 0
                backSwap(idx, p);
                idx = p;
            else
                break;
            end
        end
    end

    function backBubbleDown(idx)
        while true
            s = idx;
            l = 2 * idx; r = 2 * idx + 1;
            if l <= backHeapSize && backCompare(l, s) < 0, s = l; end
            if r <= backHeapSize && backCompare(r, s) < 0, s = r; end
            if s ~= idx
                backSwap(idx, s);
                idx = s;
            else
                break;
            end
        end
    end

    function c = backCompare(i, j)
        if backHeap(i, 1) < backHeap(j, 1)
            c = -1;
        elseif backHeap(i, 1) > backHeap(j, 1)
            c = 1;
        elseif backHeap(i, 2) < backHeap(j, 2)
            c = -1;
        elseif backHeap(i, 2) > backHeap(j, 2)
            c = 1;
        else
            c = 0;
        end
    end

    function backSwap(i, j)
        ri = backHeap(i, 3); ci = backHeap(i, 4);
        rj = backHeap(j, 3); cj = backHeap(j, 4);
        tmp = backHeap(i, :);
        backHeap(i, :) = backHeap(j, :);
        backHeap(j, :) = tmp;
        backHeapPos(ri, ci) = j;
        backHeapPos(rj, cj) = i;
    end

% ======================== 路径合并 ========================

    function p = reconstructBidirectional(meetingPoint)
        % 从 meetingPoint 沿正向 parent 回溯到 start
        fwdPath = [];
        cur = meetingPoint;
        while true
            fwdPath = [cur; fwdPath];  %#ok<AGROW>
            if cur(1) == startGrid(1) && cur(2) == startGrid(2)
                break;
            end
            cur = squeeze(fwdParent(cur(1), cur(2), :))';
            if all(cur == 0), break; end
        end

        % 从 meetingPoint 沿反向 parent 前推到 goal
        backPath = [];
        cur = meetingPoint;
        while true
            if ~(cur(1) == meetingPoint(1) && cur(2) == meetingPoint(2))
                backPath = [backPath; cur];  %#ok<AGROW>
            end
            if cur(1) == goalGrid(1) && cur(2) == goalGrid(2)
                break;
            end
            cur = squeeze(backParent(cur(1), cur(2), :))';
            if all(cur == 0), break; end
        end

        % 合并：正向路径 + 反向路径（去掉重复的 meetingPoint）
        p = [fwdPath; backPath];
    end

    function finishCallback(success)
        if ~isempty(callback)
            if success
                callback(struct('type', 'finish', 'path', path, ...
                    'iteration', iter, 'success', true));
            else
                callback(struct('type', 'finish', 'path', [], ...
                    'iteration', iter, 'success', false));
            end
        end
    end

end
