function smoothPath = SmoothPath(path, density)
%SMOOTHPATH 三次样条插值路径平滑
%   smoothPath = SmoothPath(path, density)
%   path: N×2 [row, col] 栅格路径
%   density: 每段插入点数（默认10）
%   smoothPath: M×2 连续坐标 [x, y]

if nargin < 2
    density = 10;
end

if size(path, 1) < 2
    smoothPath = [path(:, 2) - 0.5, path(:, 1) - 0.5];
    return;
end

% 转连续坐标: (r,c) → (x=c-0.5, y=r-0.5)
x = path(:, 2) - 0.5;
y = path(:, 1) - 0.5;

% 稀疏段加密：沿线段插入精确连续坐标点（避免栅格舍入造成的阶梯锯齿）
newX = x(1); newY = y(1);
for i = 2:length(x)
    dx = x(i) - x(i - 1);
    dy = y(i) - y(i - 1);
    segLen = sqrt(dx^2 + dy^2);
    if segLen > 2
        nInsert = floor(segLen / 2);
        for k = 1:nInsert
            alpha = k / (nInsert + 1);
            newX(end + 1) = x(i - 1) + alpha * dx;
            newY(end + 1) = y(i - 1) + alpha * dy;
        end
    end
    newX(end + 1) = x(i);
    newY(end + 1) = y(i);
end
x = newX(:); y = newY(:);

if length(x) < 3
    smoothPath = [x, y];
    return;
end

% 弧长参数化
t = [0; cumsum(sqrt(diff(x).^2 + diff(y).^2))];

% 避免重复点
[tu, ia] = unique(t);
if length(tu) < 2
    smoothPath = [x, y];
    return;
end

xu = x(ia);
yu = y(ia);

% 密集插值点
tInterp = linspace(tu(1), tu(end), (length(tu) - 1) * density + 1);

try
    sx = spline(tu, xu, tInterp);
    sy = spline(tu, yu, tInterp);
    smoothPath = [sx(:), sy(:)];
catch
    sx = interp1(tu, xu, tInterp, 'linear');
    sy = interp1(tu, yu, tInterp, 'linear');
    smoothPath = [sx(:), sy(:)];
end
end
