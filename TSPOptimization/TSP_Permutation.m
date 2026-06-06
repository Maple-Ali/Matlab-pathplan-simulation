function [bestOrder, bestCost, history] = TSP_Permutation(costMatrix, nPts)
%TSP_PERMUTATION 全排列枚举法求解 TSP
%   适用于目标点数 ≤ 5 的小规模问题，保证全局最优
%
%   Inputs:
%     costMatrix - nPts×nPts 对称成本矩阵
%     nPts       - 总点数（含固定起点和终点）
%
%   Outputs:
%     bestOrder  - 1×nPts 最优访问顺序（索引向量）
%     bestCost   - 最优路径总成本
%     history    - （可选）收敛历史结构体:
%       .bestCostHistory  - 累积最优成本曲线
%       .avgCostHistory   - 每个排列的成本
%       .timeHistory      - 每步累计耗时（秒）
%       .iterCount        - 总排列数
%       .elapsedTime      - 纯计算耗时（秒）

midIdx = 2:(nPts - 1);
nMid = length(midIdx);
trackHistory = (nargout >= 3);

if nMid == 0
    bestOrder = [1, nPts];
    bestCost = costMatrix(1, nPts);
    if trackHistory
        history = struct('bestCostHistory', bestCost, ...
            'avgCostHistory', bestCost, ...
            'timeHistory', 0, ...
            'iterCount', 1, 'elapsedTime', 0);
    end
    return;
end

if trackHistory
    tStart = tic;
    bestCostHistory = [];
    avgCostHistory = [];
    timeHistory = [];
end

perms_all = perms(midIdx);
bestCost = inf;
bestPerm = midIdx;

for p = 1:size(perms_all, 1)
    order = [1, perms_all(p, :), nPts];
    c = 0;
    valid = true;
    for k = 1:(length(order) - 1)
        if isinf(costMatrix(order(k), order(k + 1)))
            valid = false;
            break;
        end
        c = c + costMatrix(order(k), order(k + 1));
    end
    if valid && c < bestCost
        bestCost = c;
        bestPerm = perms_all(p, :);
    end
    if trackHistory && valid
        bestCostHistory(end + 1) = bestCost; %#ok<AGROW>
        avgCostHistory(end + 1) = c; %#ok<AGROW>
        timeHistory(end + 1) = toc(tStart); %#ok<AGROW>
    end
end

bestOrder = [1, bestPerm, nPts];

% 输出收敛历史
if trackHistory
    history = struct();
    history.bestCostHistory = bestCostHistory;
    history.avgCostHistory = avgCostHistory;
    history.timeHistory = timeHistory;
    history.iterCount = size(perms_all, 1);
    history.elapsedTime = toc(tStart);
end
end
