function robotTasks = MultiRobotTaskAllocation(startPoints, targets, goalPoints, map, algoName, tspAlgo, clusterAlgo)
%MULTIROBOTTASKALLOCATION 多机器人任务分配
%   将目标点按聚类算法分配给各机器人，每台机器人独立求解 TSP
%
%   Inputs:
%     startPoints - N×2 [row, col] 各机器人起点
%     targets     - K×2 [row, col] 目标点
%     goalPoints  - N×2 [row, col] 各机器人终点（每行对应一台机器人）
%     map         - Map 对象
%     algoName    - 'AStar' | 'Dijkstra' | 'RRT'
%     tspAlgo     - 'TSP_GA' | 'TSP_Permutation' 等 TSP 求解算法（可选）
%     clusterAlgo - 聚类算法名称（可选，默认 'NearestNeighbor'）
%                   'NearestNeighbor' | 'KMedoids_Cluster' | ...
%
%   Output:
%     robotTasks  - 1×N 结构体数组，字段:
%       .orderedPoints    - 有序访问点（含起点和目标点、终点）
%       .segPaths         - 段间路径 cell 数组
%       .tspCost          - 该机器人路径总成本
%       .assignedTargets  - 分配给该机器人的目标点 [row, col]

if nargin < 6 || isempty(tspAlgo)
    tspAlgo = 'TSP_GA';
end
if nargin < 7 || isempty(clusterAlgo)
    clusterAlgo = 'NearestNeighbor';
end

numRobots = size(startPoints, 1);
numTargets = size(targets, 1);

% --- 1. 聚类分配 ---
if strcmp(clusterAlgo, 'NearestNeighbor')
    % 最近邻贪心分配
    assignment = zeros(numTargets, 1);
    for t = 1:numTargets
        tg = targets(t, :);
        minDist = inf;
        bestR = 1;
        for r = 1:numRobots
            d = norm(tg - startPoints(r, :));
            if d < minDist
                minDist = d;
                bestR = r;
            end
        end
        assignment(t) = bestR;
    end
else
    % 调用指定的聚类算法
    clusterFunc = str2func(clusterAlgo);
    if strcmp(clusterAlgo, 'DV_Cluster')
        % DV 偏离量分配：需要 map + goalPoints + 规划器
        [assignment, ~, ~] = DV_Cluster(targets, numRobots, [], startPoints, map, ...
            goalPoints, algoName);
    elseif strcmp(clusterAlgo, 'DV_Cluster_v1')
        % DV + 相似度迁移优化：需要 map + goalPoints + 规划器 + TSP 算法
        [assignment, ~, ~] = DV_Cluster_v1(targets, numRobots, [], startPoints, map, ...
            goalPoints, algoName, tspAlgo);
    elseif strcmp(clusterAlgo, 'DV_Cluster_Hungarian')
        % DV + 匈牙利全局最优分配
        [assignment, ~, ~] = DV_Cluster_Hungarian(targets, numRobots, [], startPoints, map, ...
            goalPoints, algoName);
    elseif strcmp(clusterAlgo, 'DV_Cluster_v1_Hungarian')
        % DV + 匈牙利 + 相似度迁移优化
        [assignment, ~, ~] = DV_Cluster_v1_Hungarian(targets, numRobots, [], startPoints, map, ...
            goalPoints, algoName, tspAlgo);
    elseif contains(clusterAlgo, '_v2')
        % v2 负载均衡版本：传入终点、规划器、TSP 算法
        [assignment, ~, ~] = clusterFunc(targets, numRobots, [], startPoints, map, ...
            goalPoints, algoName, tspAlgo);
    elseif contains(clusterAlgo, '_v1')
        % 路径距离版本：传入 map 对象
        [assignment, ~, ~] = clusterFunc(targets, numRobots, [], startPoints, map);
    else
        [assignment, ~, ~] = clusterFunc(targets, numRobots, [], startPoints);
    end
end

% --- 2. 各机器人独立 TSP ---
robotTasks = struct('orderedPoints', {}, 'segPaths', {}, 'tspCost', {}, 'assignedTargets', {});

for r = 1:numRobots
    myTargets = targets(assignment == r, :);
    myGoal = goalPoints(r, :);
    robotTasks(r).assignedTargets = myTargets;
    robotTasks(r).goalPoint = myGoal;

    if isempty(myTargets)
        % 无分配目标：直接起点→终点
        robotTasks(r).orderedPoints = [startPoints(r, :); myGoal];
        segPath = callPlanner(algoName, map, startPoints(r, :), myGoal);
        robotTasks(r).segPaths = {segPath};
        robotTasks(r).tspCost = 0;
    else
        [ordered, segs, cost] = TSPsolver(startPoints(r, :), myTargets, myGoal, map, algoName, tspAlgo);
        robotTasks(r).orderedPoints = ordered;
        robotTasks(r).segPaths = segs;
        robotTasks(r).tspCost = cost;
    end
end
end

% -------------------------------------------------------------------------
function path = callPlanner(algoName, map, startGrid, goalGrid)
    switch algoName
        case 'AStar'
            path = AStar(map, startGrid, goalGrid, 0);
        case 'AStar_v0'
            path = AStar_v0(map, startGrid, goalGrid, 0);
        case 'AStar_v1'
            path = AStar_v1(map, startGrid, goalGrid, 0);
        case 'AStar_v2'
            path = AStar_v2(map, startGrid, goalGrid, 0);
        case 'AStar_v3'
            path = AStar_v3(map, startGrid, goalGrid, 0);
        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0);
        case 'Dijkstra_v1'
            path = Dijkstra_v1(map, startGrid, goalGrid, 0);
        case 'RRT'
            path = RRT(map, startGrid, goalGrid, 0);
        otherwise
            path = AStar(map, startGrid, goalGrid, 0);
    end
end
