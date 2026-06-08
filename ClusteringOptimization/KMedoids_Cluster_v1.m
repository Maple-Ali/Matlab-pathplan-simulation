function [assignment, medoids, history] = KMedoids_Cluster_v1(points, k, distMatrix, initMedoids, map)
%KMEDOIDS_CLUSTER_V1 K-Medoids 聚类算法（Dijkstra 实际路径距离版）
%   使用 Dijkstra 全局规划算法计算目标点之间的实际最短路径距离，
%   替代欧氏距离，使聚类结果反映真实导航代价。
%
%   [assignment, medoids, history] = KMedoids_Cluster_v1(points, k)
%   [assignment, medoids, history] = KMedoids_Cluster_v1(points, k, [], [], map)
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

% 使用 Dijkstra 单源多目标计算实际路径距离矩阵
% 从每个点出发一次 Dijkstra 得到该点到所有其他点的距离
n = size(points, 1);
pathDistMatrix = zeros(n, n);
for i = 1:n
    otherIdx = [1:i-1, i+1:n];
    dists = DijkstraAllDistances_v1(map, points(i,:), points(otherIdx, :));
    pathDistMatrix(i, otherIdx) = dists;
end

% 调用基础 K-Medoids 算法
[assignment, medoids, history] = KMedoids_Cluster(points, k, pathDistMatrix, initMedoids);
end
