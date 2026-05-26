function robotTasks = MultiRobotTaskAllocation(startPoints, targets, goalPoint, map, algoName)
%MULTIROBOTTASKALLOCATION 多机器人任务分配
%   将目标点按最近邻聚类分配给各机器人，每台机器人独立求解 TSP
%
%   Inputs:
%     startPoints - N×2 [row, col] 各机器人起点
%     targets     - K×2 [row, col] 目标点
%     goalPoint   - 1×2 [row, col] 公共终点
%     map         - Map 对象
%     algoName    - 'AStar' | 'Dijkstra' | 'RRT'
%
%   Output:
%     robotTasks  - 1×N 结构体数组，字段:
%       .orderedPoints    - 有序访问点（含起点和目标点、终点）
%       .segPaths         - 段间路径 cell 数组
%       .tspCost          - 该机器人路径总成本
%       .assignedTargets  - 分配给该机器人的目标点 [row, col]

numRobots = size(startPoints, 1);
numTargets = size(targets, 1);

% --- 1. 最近邻目标分配 ---
assignment = zeros(numTargets, 1);  % assignment(t) = 分配给机器人 r
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

% --- 2. 各机器人独立 TSP ---
robotTasks = struct('orderedPoints', {}, 'segPaths', {}, 'tspCost', {}, 'assignedTargets', {});

for r = 1:numRobots
    myTargets = targets(assignment == r, :);
    robotTasks(r).assignedTargets = myTargets;

    if isempty(myTargets)
        % 无分配目标：直接起点→终点
        robotTasks(r).orderedPoints = [startPoints(r, :); goalPoint];
        segPath = callPlanner(algoName, map, startPoints(r, :), goalPoint);
        robotTasks(r).segPaths = {segPath};
        robotTasks(r).tspCost = 0;
    else
        [ordered, segs, cost] = TSPsolver(startPoints(r, :), myTargets, goalPoint, map, algoName);
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
        case 'Dijkstra'
            path = Dijkstra(map, startGrid, goalGrid, 0);
        case 'RRT'
            path = RRT(map, startGrid, goalGrid, 0);
        otherwise
            path = AStar(map, startGrid, goalGrid, 0);
    end
end
