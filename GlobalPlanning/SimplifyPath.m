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
    dr = abs(p1(1) - p2(1));
    dc = abs(p1(2) - p2(2));
    steps = max(dr, dc) * 3;
    steps = max(steps, 10);
    for t = 0:steps
        alpha = t / steps;
        rCont = p1(1) + alpha * (p2(1) - p1(1));
        cCont = p1(2) + alpha * (p2(2) - p1(2));
        r = round(rCont);
        c = round(cCont);
        r = max(1, min(n, r));
        c = max(1, min(n, c));
        if occGrid(r, c)
            free = false;
            return;
        end
        % 对角线阻挡检测：防止从两个对角障碍物的角点之间穿过
        if cCont > c && rCont > r
            if r < n && c < n && occGrid(r+1, c) && occGrid(r, c+1)
                free = false; return;
            end
        elseif cCont < c && rCont > r
            if r < n && c > 1 && occGrid(r+1, c) && occGrid(r, c-1)
                free = false; return;
            end
        elseif cCont > c && rCont < r
            if r > 1 && c < n && occGrid(r-1, c) && occGrid(r, c+1)
                free = false; return;
            end
        elseif cCont < c && rCont < r
            if r > 1 && c > 1 && occGrid(r-1, c) && occGrid(r, c-1)
                free = false; return;
            end
        end
    end
    free = true;
end
