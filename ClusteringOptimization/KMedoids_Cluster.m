function [assignment, medoids, history] = KMedoids_Cluster(points, k, varargin)
%KMEDOIDS_CLUSTER K-Medoids 聚类算法
%   将 N 个数据点聚类为 k 个簇，中心点为实际数据点（medoids）。
%
%   [assignment, medoids, history] = KMedoids_Cluster(points, k)
%   [assignment, medoids, history] = KMedoids_Cluster(points, k, distMatrix)
%   [assignment, medoids, history] = KMedoids_Cluster(points, k, distMatrix, initMedoids)
%
%   Inputs:
%     points      - N×2 数据点坐标 [x, y]
%     k           - 聚类数量
%     distMatrix  - (可选) N×N 预计算距离矩阵
%     initMedoids - (可选) k×2 初始 medoid 坐标（不要求是 points 中的点）
%                   若提供则使用这些点作为初始 medoids（用于机器人位置作为中心）
%
%   Outputs:
%     assignment  - N×1 每个点的聚类编号 (1..k)
%     medoids     - k×2 最终 medoid 坐标
%     history     - 迭代历史结构体:
%                   .cost        - 各迭代总距离成本
%                   .converged   - 是否收敛
%                   .nIter       - 实际迭代次数
%                   .elapsedTime - 总耗时(秒)

%% 输入解析
if nargin >= 3 && ~isempty(varargin{1})
    distMatrix = varargin{1};
else
    distMatrix = [];
end

if nargin >= 4 && ~isempty(varargin{2})
    initMedoids = varargin{2};
else
    initMedoids = [];
end

n = size(points, 1);

if k > n
    k = n;
end

%% 预计算距离矩阵
if isempty(distMatrix)
    distMatrix = zeros(n, n);
    for i = 1:n
        for j = i+1:n
            d = norm(points(i,:) - points(j,:));
            distMatrix(i, j) = d;
            distMatrix(j, i) = d;
        end
    end
end

%% 初始化 medoids
if ~isempty(initMedoids)
    % 将初始 medoids 映射到最近的数据点索引
    medIdx = zeros(k, 1);
    for c = 1:k
        dists = sum((points - initMedoids(c,:)).^2, 2);
        [~, medIdx(c)] = min(dists);
    end
    medIdx = unique(medIdx, 'stable');
    if length(medIdx) < k
        % 补充随机选择
        remaining = setdiff(1:n, medIdx);
        if ~isempty(remaining)
            extra = remaining(randperm(length(remaining), min(k - length(medIdx), length(remaining))));
            medIdx = [medIdx; extra(:)];
        end
    end
else
    % 随机初始化
    perm = randperm(n);
    medIdx = perm(1:k)';
end

%% K-Medoids 主循环（PAM 算法）
maxIter = 100;
prevMedIdx = zeros(k, 1);
assignment = zeros(n, 1);

costHistory = zeros(maxIter, 1);
tStart = tic;

for iter = 1:maxIter
    % === 分配阶段：每个点分配到最近的 medoid ===
    for i = 1:n
        bestDist = inf;
        for c = 1:k
            d = distMatrix(i, medIdx(c));
            if d < bestDist
                bestDist = d;
                assignment(i) = c;
            end
        end
    end

    % === 更新阶段：对每个簇寻找最优 medoid ===
    newMedIdx = medIdx;
    for c = 1:k
        clusterMembers = find(assignment == c);
        if isempty(clusterMembers)
            continue;
        end

        currentCost = sum(distMatrix(clusterMembers, medIdx(c)));
        bestCost = currentCost;
        bestIdx = medIdx(c);

        % 尝试用簇内每个非 medoid 点替换
        for m = 1:length(clusterMembers)
            candidateIdx = clusterMembers(m);
            if candidateIdx == medIdx(c)
                continue;
            end
            newCost = sum(distMatrix(clusterMembers, candidateIdx));
            if newCost < bestCost
                bestCost = newCost;
                bestIdx = candidateIdx;
            end
        end
        newMedIdx(c) = bestIdx;
    end

    % 计算本轮总成本
    totalCost = 0;
    for i = 1:n
        totalCost = totalCost + distMatrix(i, newMedIdx(assignment(i)));
    end
    costHistory(iter) = totalCost;

    % 检查收敛
    if isequal(sort(newMedIdx), sort(prevMedIdx))
        costHistory = costHistory(1:iter);
        history.cost = costHistory;
        history.converged = true;
        history.nIter = iter;
        history.elapsedTime = toc(tStart);
        medoids = points(medIdx, :);
        return;
    end

    prevMedIdx = medIdx;
    medIdx = newMedIdx;
end

%% 达到最大迭代次数
costHistory = costHistory(1:maxIter);
history.cost = costHistory;
history.converged = false;
history.nIter = maxIter;
history.elapsedTime = toc(tStart);
medoids = points(medIdx, :);

end
