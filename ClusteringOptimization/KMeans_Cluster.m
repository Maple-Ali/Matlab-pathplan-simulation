function [assignment, centroids, history] = KMeans_Cluster(points, k, varargin)
%KMEANS_CLUSTER K-Means 聚类算法
%   将 N 个数据点聚类为 k 个簇，中心点为簇内均值（centroid）。
%
%   [assignment, centroids, history] = KMeans_Cluster(points, k)
%   [assignment, centroids, history] = KMeans_Cluster(points, k, ~)
%   [assignment, centroids, history] = KMeans_Cluster(points, k, ~, initCentroids)
%
%   Inputs:
%     points        - N×2 数据点坐标 [x, y]
%     k             - 聚类数量
%     ~             - 占位（距离矩阵，K-Means 不使用，保持接口一致）
%     initCentroids - (可选) k×2 初始中心坐标（如机器人起点）
%                     若提供则使用这些点作为初始 centroids
%
%   Outputs:
%     assignment    - N×1 每个点的聚类编号 (1..k)
%     centroids     - k×2 最终中心坐标（均值点，不一定是实际数据点）
%     history       - 迭代历史结构体:
%                     .cost        - 各迭代总距离成本
%                     .converged   - 是否收敛
%                     .nIter       - 实际迭代次数
%                     .elapsedTime - 总耗时(秒)

%% 输入解析
if nargin >= 4 && ~isempty(varargin{2})
    initCentroids = varargin{2};
else
    initCentroids = [];
end

n = size(points, 1);

if k > n
    k = n;
end

%% 初始化 centroids
if ~isempty(initCentroids)
    centroids = initCentroids(1:k, :);
else
    perm = randperm(n);
    centroids = points(perm(1:k), :);
end

%% K-Means 主循环
maxIter = 100;
tol = 1e-6;
assignment = zeros(n, 1);
prevCentroids = inf(k, 2);

costHistory = zeros(maxIter, 1);
tStart = tic;

for iter = 1:maxIter
    % === 分配阶段：每个点分配到最近的 centroid ===
    for i = 1:n
        bestDist = inf;
        for c = 1:k
            d = norm(points(i,:) - centroids(c,:));
            if d < bestDist
                bestDist = d;
                assignment(i) = c;
            end
        end
    end

    % === 更新阶段：重新计算每个簇的均值 ===
    for c = 1:k
        members = points(assignment == c, :);
        if ~isempty(members)
            centroids(c,:) = mean(members, 1);
        end
    end

    % 计算本轮总成本（各点到其 centroid 的距离之和）
    totalCost = 0;
    for i = 1:n
        totalCost = totalCost + norm(points(i,:) - centroids(assignment(i),:));
    end
    costHistory(iter) = totalCost;

    % 检查收敛（centroid 移动量 < tol）
    shift = max(sqrt(sum((centroids - prevCentroids).^2, 2)));
    if shift < tol
        costHistory = costHistory(1:iter);
        history.cost = costHistory;
        history.converged = true;
        history.nIter = iter;
        history.elapsedTime = toc(tStart);
        return;
    end

    prevCentroids = centroids;
end

%% 达到最大迭代次数
costHistory = costHistory(1:maxIter);
history.cost = costHistory;
history.converged = false;
history.nIter = maxIter;
history.elapsedTime = toc(tStart);

end
