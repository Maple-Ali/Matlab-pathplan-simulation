function results = analyzeConvergenceTime(matFilePath)
% analyzeConvergenceTime  分析 mat 文件中 allHistories 的收敛时间，并输出统计信息
%   results = analyzeConvergenceTime(matFilePath) 读取指定的 .mat 文件，
%   从中提取变量 allHistories（应为 1×N 的 cell 数组，每个 cell 是一个结构体）。
%   对每个实验结构体，找出最优成本（最后一次迭代的成本）以及首次达到该成本的
%   迭代编号，然后从 timeHistory 中获取对应的时间。
%   最后在命令窗口显示 OptimalCost 的最佳值、中位数、极差、平均值、标准差，
%   以及 TimeToOptimal 的平均值，并返回包含每个实验详细信息的表格。
%
%   输入以下内容运行：
%   resultTable = analyzeConvergenceTime('exp_aco_kroA150_results.mat'); disp(resultTable);
%
%   输入参数：
%       matFilePath - .mat 文件的完整路径（字符串）
%
%   输出参数：
%       results     - 一个表格，包含以下列：
%                     Experiment     实验编号（按列顺序）
%                     OptimalCost    最优成本（bestCostHistory 的最后一项）
%                     FirstIteration 首次达到最优成本的迭代编号（1-based）
%                     TimeToOptimal  达到最优成本时的运行时间
%
%   注意：
%   - bestCostHistory 和 timeHistory 应为等长的数值向量。
%   - 浮点数比较使用了容差 1e-12。
%   - 缺失字段或数据的实验将被标记为 NaN，并从统计计算中排除。
%   - 统计信息会在命令窗口打印，同时结果表格返回到工作区。
%   - 标准差使用样本标准差（除以 N-1），若需总体标准差可自行修改 std 调用。

    % 加载 .mat 文件
    if ~exist(matFilePath, 'file')
        error('文件不存在: %s', matFilePath);
    end
    data = load(matFilePath);

    % 检查 allHistories 变量
    if ~isfield(data, 'allHistories')
        error('变量 allHistories 在文件中不存在。');
    end
    allHistories = data.allHistories;

    if ~iscell(allHistories)
        error('allHistories 应为 cell 数组。');
    end

    % 获取实验数量（第一行）
    [rows, cols] = size(allHistories);
    if rows > 1
        warning('allHistories 包含多行，仅分析第一行的 %d 个实验。', cols);
    end
    numExperiments = cols;

    % 初始化存储
    expIndex = (1:numExperiments)';
    optCost = nan(numExperiments, 1);
    firstIter = nan(numExperiments, 1);
    timeToOpt = nan(numExperiments, 1);

    tolerance = 1e-12;

    % 逐实验分析
    for k = 1:numExperiments
        expStruct = allHistories{1, k};

        % 有效性检查
        if ~isstruct(expStruct)
            warning('第 %d 个元素不是结构体，已跳过。', k);
            continue;
        end
        if ~isfield(expStruct, 'bestCostHistory') || ~isfield(expStruct, 'timeHistory')
            warning('第 %d 个实验缺少 bestCostHistory 或 timeHistory 字段，已跳过。', k);
            continue;
        end

        best = expStruct.bestCostHistory(:);
        time = expStruct.timeHistory(:);

        if length(best) ~= length(time)
            warning('第 %d 个实验中 bestCostHistory 与 timeHistory 长度不一致，已跳过。', k);
            continue;
        end
        if isempty(best)
            warning('第 %d 个实验中 bestCostHistory 为空，已跳过。', k);
            continue;
        end

        finalCost = best(end);
        optCost(k) = finalCost;

        % 首次达到最优的迭代
        idx = find(abs(best - finalCost) <= tolerance * max(1, abs(finalCost)), 1, 'first');
        if isempty(idx)
            idx = find(best == finalCost, 1, 'first');
        end
        firstIter(k) = idx;
        timeToOpt(k) = time(idx);
    end

    % 构建结果表格
    results = table(expIndex, optCost, firstIter, timeToOpt, ...
        'VariableNames', {'Experiment', 'OptimalCost', 'FirstIteration', 'TimeToOptimal'});

    % ----- 计算并输出统计信息（忽略 NaN） -----
    validIdx = ~isnan(optCost) & ~isnan(timeToOpt);
    if any(validIdx)
        bestCost   = min(optCost(validIdx));          % 最优成本中的最佳值（最小成本）
        medianCost = median(optCost(validIdx));        % 中位数
        rangeCost  = max(optCost(validIdx)) - min(optCost(validIdx)); % 极差
        avgCost    = mean(optCost(validIdx));
        stdCost    = std(optCost(validIdx));           % 标准差（样本标准差）
        avgTime    = mean(timeToOpt(validIdx));

        fprintf('\n========== 统计信息 ==========\n');
        fprintf('OptimalCost 最佳值   : %.6g\n', bestCost);
        fprintf('OptimalCost 中位数   : %.6g\n', medianCost);
        fprintf('OptimalCost 极差     : %.6g\n', rangeCost);
        fprintf('OptimalCost 平均值   : %.4f\n', avgCost);
        fprintf('OptimalCost 标准差   : %.6g\n', stdCost);
        fprintf('TimeToOptimal 平均值 : %.4f\n', avgTime);
        fprintf('（基于 %d 个有效实验）\n', sum(validIdx));
    else
        fprintf('\n没有有效数据可供计算统计量。\n');
    end
end