function simplePath = SimplifyPath(path, occGrid, n, safetyMargin)
%SIMPLIFYPATH 路径简化（拐点提取 + 贪心 + 中间点探索优化）
%   simplePath = SimplifyPath(path, occGrid, n, safetyMargin)
%   path: N×2 [row, col] 原始路径
%   occGrid: 占用栅格矩阵
%   n: 地图尺寸
%   safetyMargin: 可选，路径与障碍物栅格的最小安全距离（栅格单位），默认 0.3
%
%   算法流程：
%     Step 1: 提取拐点（方向改变处），减少路径点数
%     Step 2: 贪心——从当前点找最远可达节点
%     Step 3: 中间点探索——检查被跳过的节点能否连接更远的节点
%     Step 4: 路径对比——保留更短的路径方案

if nargin < 4
    safetyMargin = 0.3;
end

if size(path, 1) <= 2
    simplePath = path;
    return;
end

% Step 1: 提取拐点（方向改变处）+ 起终点
corners = extractCorners(path);
if size(corners, 1) <= 2
    simplePath = corners;
    return;
end

% Step 2-4: 贪心 + 中间点探索优化
simplePath = corners(1, :);
i = 1;

while i < size(corners, 1)
    % Step 2: 从当前点找最远可达节点
    farthest = i;
    for j = (i + 1):size(corners, 1)
        if isLineFree(corners(i, :), corners(j, :), occGrid, n, safetyMargin)
            farthest = j;
        end
    end

    % 到达终点
    if farthest == size(corners, 1)
        simplePath(end + 1, :) = corners(end, :); %#ok<AGROW>
        break;
    end

    % 无法前进（不应出现）
    if farthest == i
        simplePath(end + 1, :) = corners(i + 1, :); %#ok<AGROW>
        i = i + 1;
        continue;
    end

    % Step 3: 中间点探索——检查被跳过的节点能否连接更远
    bestMid = 0;
    bestFarMid = 0;
    for k = (i + 1):(farthest - 1)
        for j = (farthest + 1):size(corners, 1)
            if isLineFree(corners(k, :), corners(j, :), occGrid, n, safetyMargin)
                if j > bestFarMid
                    bestMid = k;
                    bestFarMid = j;
                end
            end
        end
    end

    % Step 4: 对比路径代价，选择更优方案
    if bestMid > 0 && bestFarMid > farthest
        % 路径 A: cur → farthest → farMid（贪心 + 直连）
        dA = norm(corners(i, :) - corners(farthest, :)) + ...
             norm(corners(farthest, :) - corners(bestFarMid, :));
        % 路径 B: cur → mid → farMid（经过中间点）
        dB = norm(corners(i, :) - corners(bestMid, :)) + ...
             norm(corners(bestMid, :) - corners(bestFarMid, :));

        if dB < dA
            % 保留中间点
            simplePath(end + 1, :) = corners(bestMid, :); %#ok<AGROW>
            simplePath(end + 1, :) = corners(bestFarMid, :); %#ok<AGROW>
            i = bestFarMid;
        else
            simplePath(end + 1, :) = corners(farthest, :); %#ok<AGROW>
            i = farthest;
        end
    else
        simplePath(end + 1, :) = corners(farthest, :); %#ok<AGROW>
        i = farthest;
    end
end
end

function corners = extractCorners(path)
%EXTRACTCORNERS 从路径中提取拐点（方向改变处）+ 起终点
%   栅格路径每步移动方向为8邻域之一，当连续两步方向不同时即为拐点
    corners = path(1, :);
    for i = 2:(size(path, 1) - 1)
        dr1 = path(i, 1) - path(i-1, 1);
        dc1 = path(i, 2) - path(i-1, 2);
        dr2 = path(i+1, 1) - path(i, 1);
        dc2 = path(i+1, 2) - path(i, 2);
        % 方向改变（包括直线→对角、对角→直线、对角→不同对角）
        if dr1 ~= dr2 || dc1 ~= dc2
            corners(end + 1, :) = path(i, :); %#ok<AGROW>
        end
    end
    corners(end + 1, :) = path(end, :);
end

function free = isLineFree(p1, p2, occGrid, n, safetyMargin)
%ISLINEFREE 检查线段 p1→p2 是否与障碍物保持安全距离
%   在线段上密集采样，对每个采样点检查其邻域内的障碍物栅格。
%   使用精确的点到栅格边界距离，而非粗糙的栅格中心距离。

dr = abs(p1(1) - p2(1));
dc = abs(p1(2) - p2(2));
segLen = max(dr, dc);

% 密集采样：每个栅格单位至少采 10 个点，最少 30 点
% 高密度确保不会在采样点之间漏过障碍物角点
steps = max(ceil(segLen * 10), 30);

% 搜索半径：安全距离覆盖内的栅格都需要检查
%   采样点到栅格边界距离 = sqrt(max(0,|dr|-0.5)^2 + max(0,|dc|-0.5)^2)
%   需要找到所有 boundaryDist < safetyMargin 的栅格
%   必要条件：|行差| < safetyMargin + 0.5 且 |列差| < safetyMargin + 0.5
dMax = ceil(safetyMargin + 0.5);

for t = 0:steps
    alpha = t / steps;
    rCont = p1(1) + alpha * (p2(1) - p1(1));
    cCont = p1(2) + alpha * (p2(2) - p1(2));

    r0 = round(rCont);
    c0 = round(cCont);

    % 检查邻域内所有栅格
    for dr2 = -dMax:dMax
        rCell = r0 + dr2;
        if rCell < 1 || rCell > n
            continue;
        end
        for dc2 = -dMax:dMax
            cCell = c0 + dc2;
            if cCell < 1 || cCell > n
                continue;
            end
            if ~occGrid(rCell, cCell)
                continue;
            end

            % 精确计算采样点到该障碍物栅格边界的距离
            % 栅格 (rCell, cCell) 占据 [rCell-0.5, rCell+0.5] × [cCell-0.5, cCell+0.5]
            % 点到矩形区域的最小距离：
            %   dx = max(0, |rCont - rCell| - 0.5)
            %   dy = max(0, |cCont - cCell| - 0.5)
            %   minDist = sqrt(dx^2 + dy^2)
            dx = max(0, abs(rCont - rCell) - 0.5);
            dy = max(0, abs(cCont - cCell) - 0.5);
            if sqrt(dx * dx + dy * dy) < safetyMargin
                free = false;
                return;
            end
        end
    end
end
free = true;
end
