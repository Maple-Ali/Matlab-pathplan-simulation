function [assignment, medoids, history] = KMedoids_Cluster_v1_1(points, k, distMatrix, initMedoids, map)
%KMEDOIDS_CLUSTER_V1_1 K-Medoids 聚类算法（AStar_v3 实际路径距离版）
%   使用 AStar_v3 全局规划算法计算目标点之间的实际最短路径距离，
%   替代欧氏距离，使聚类结果反映真实导航代价。
%
%   [assignment, medoids, history] = KMedoids_Cluster_v1_1(points, k)
%   [assignment, medoids, history] = KMedoids_Cluster_v1_1(points, k, [], [], map)
%
%   Inputs:
%     points      - N×2 目标点坐标 [row, col]
%     k           - 聚类数量（机器人数）
%     distMatrix  - 忽略（接口兼容），由本函数内部计算
%     initMedoids - (可选) k×2 初始 medoid 坐标
%     map         - Map 对象（栅格地图），用于路径规划
%
%   若不提供 map，则回退到欧氏距离（与 KMedoids_Cluster 行为一致）。
%
%   Outputs: 同 KMedoids_Cluster

if nargin < 5 || isempty(map)
    % 无地图时回退到欧氏距离
    [assignment, medoids, history] = KMedoids_Cluster(points, k, [], initMedoids);
    return;
end

% 使用 AStar_v3 计算实际路径距离矩阵
n = size(points, 1);
pathDistMatrix = zeros(n, n);
for i = 1:n
    for j = i+1:n
        p = AStar_v3(map, points(i,:), points(j,:), 0);
        if ~isempty(p)
            d = pathLength(p);
        else
            d = 1e6;  % 不可达时的惩罚值
        end
        pathDistMatrix(i, j) = d;
        pathDistMatrix(j, i) = d;
    end
end

% 调用基础 K-Medoids 算法
[assignment, medoids, history] = KMedoids_Cluster(points, k, pathDistMatrix, initMedoids);
end

function d = pathLength(path)
    d = 0;
    for i = 1:size(path, 1) - 1
        d = d + norm(path(i+1, :) - path(i, :));
    end
end
