function simplePath = SimplifyPath(path, occGrid, n, safetyMargin)
%SIMPLIFYPATH 拐角裁剪路径简化（带安全距离检测）
%   simplePath = SimplifyPath(path, occGrid, n, safetyMargin)
%   path: N×2 [row, col] 原始路径
%   occGrid: 占用栅格矩阵
%   n: 地图尺寸
%   safetyMargin: 可选，路径与障碍物栅格的最小安全距离（栅格单位），默认 0.4
%                 线段上任意一点到障碍物栅格边界的距离不得小于此值

if nargin < 4
    safetyMargin = 0.3;
end

if size(path, 1) <= 2
    simplePath = path;
    return;
end

simplePath = path(1, :);
i = 1;

while i < size(path, 1)
    for j = size(path, 1):-1:(i + 1)
        if isLineFree(path(i, :), path(j, :), occGrid, n, safetyMargin)
            simplePath(end + 1, :) = path(j, :);
            i = j;
            break;
        end
    end
end
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
