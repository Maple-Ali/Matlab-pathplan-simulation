function simplePath = SimplifyPath(path, occGrid, n)
%SIMPLIFYPATH 拐角裁剪路径简化
%   simplePath = SimplifyPath(path, occGrid, n)
%   path: N×2 [row, col] 原始路径
%   occGrid: 占用栅格矩阵
%   n: 地图尺寸

if size(path, 1) <= 2
    simplePath = path;
    return;
end

simplePath = path(1, :);
i = 1;

while i < size(path, 1)
    % 从当前点出发，找能直线连接的最远点
    for j = size(path, 1):-1:(i + 1)
        if isLineFree(path(i, :), path(j, :), occGrid, n)
            simplePath(end + 1, :) = path(j, :);
            i = j;
            break;
        end
    end
end
end

function free = isLineFree(p1, p2, occGrid, n)
    steps = max(abs(p1(1) - p2(1)), abs(p1(2) - p2(2))) * 2;
    steps = max(steps, 10);
    for t = 0:steps
        alpha = t / steps;
        r = round(p1(1) + alpha * (p2(1) - p1(1)));
        c = round(p1(2) + alpha * (p2(2) - p1(2)));
        r = max(1, min(n, r));
        c = max(1, min(n, c));
        if occGrid(r, c)
            free = false;
            return;
        end
    end
    free = true;
end
