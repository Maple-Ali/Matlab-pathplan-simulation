function path = AStar(map, startGrid, goalGrid, delay, callback)
%ASTAR A* 全局路径规划
%   path = AStar(map, startGrid, goalGrid, delay, callback)
%   map: Map 对象
%   startGrid, goalGrid: [row, col] 栅格索引
%   delay: 可视化延迟（0=不绘制，仅 callback 为空时生效）
%   callback: 可选回调函数 @(stateInfo) 返回 'continue'/'pause'/'stop'
%   path: N×2 [row, col] 路径点数组

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

% 启发式：欧氏距离
h = @(r, c) sqrt((r - goalGrid(1))^2 + (c - goalGrid(2))^2);

% g 值矩阵
gScore = inf(n, n);
gScore(startGrid(1), startGrid(2)) = 0;

% f = g + h
fScore = inf(n, n);
fScore(startGrid(1), startGrid(2)) = h(startGrid(1), startGrid(2));

% 父节点记录
parent = zeros(n, n, 2);

% open set: 用逻辑矩阵 + 手动找最小 f
openSet = false(n, n);
openSet(startGrid(1), startGrid(2)) = true;

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

while any(openSet, 'all')
    % 找 open 中 f 最小的节点
    [openRows, openCols] = find(openSet);
    minF = inf;
    minIdx = 1;
    for i = 1:length(openRows)
        f = fScore(openRows(i), openCols(i));
        if f < minF
            minF = f;
            minIdx = i;
        end
    end
    current = [openRows(minIdx), openCols(minIdx)];

    % ---- callback 模式：向 GUI 报告当前状态 ----
    if ~isempty(callback)
        iter = iter + 1;
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

    openSet(current(1), current(2)) = false;
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
            fScore(nr, nc) = tentG + h(nr, nc);
            parent(nr, nc, :) = current;
            openSet(nr, nc) = true;
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
