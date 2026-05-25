function d = distanceMetric(p1, p2, method)
%DISTANCEMETRIC 距离计算工具
%   p1, p2: 点坐标，支持 [x, y] 或 [row, col] 格式
%   method: 'euclidean' | 'manhattan' | 'chebyshev' | 'grid'
%   grid 模式下将输入视为栅格索引直接计算曼哈顿距离

if nargin < 3
    method = 'euclidean';
end

switch lower(method)
    case 'euclidean'
        d = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2);
    case 'manhattan'
        d = abs(p1(1) - p2(1)) + abs(p1(2) - p2(2));
    case 'chebyshev'
        d = max(abs(p1(1) - p2(1)), abs(p1(2) - p2(2)));
    case 'grid'
        d = abs(p1(1) - p2(1)) + abs(p1(2) - p2(2));
    otherwise
        error('未知距离方法: %s', method);
end
end
