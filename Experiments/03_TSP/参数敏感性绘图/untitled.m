%% plot_compare — TSP算法在kroA150上性能对比绘图（仅Figure 4 & 5，支持每组多个子文件）
%  从 results/*.mat 加载数据，绘制对比图
%  usage: 将 results/ 下的 .mat 文件名填到 algoLabels 的映射中即可
%  每行格式：file1, base1, label1, file2, base2, label2, file3, base3, label3, groupName, color
%  最多支持3个子文件（可留空），子文件用不同形状表示，图例显示子标签
%  基线值 base 用于图4的Y轴平移，各子文件可独立设置

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

% ===== Config: 每组算法可包含多个子文件（最多3个），共享组名和颜色 =====
%  列1: 子文件1文件名     列2: 子文件1基线值     列3: 子文件1标签
%  列4: 子文件2文件名     列5: 子文件2基线值     列6: 子文件2标签
%  列7: 子文件3文件名     列8: 子文件3基线值     列9: 子文件3标签
%  列10: 组显示名         列11: 颜色 [R,G,B]
algoLabels = {
%    'data1.mat', 620.1509, '标签名1', 'data2.mat', 620.1509, '标签名2', 'data3.mat', 620.1509, '标签名3', '3/3/3/4/2', [0.47, 0.67, 0.19];
%    'A_Map1_1.mat','350.4406','Map1','A_Map2_1.mat','619.1509','Map2','A_Map3_1.mat','22140.0000','Map3',    '1/3/3/4/2',   [0.47, 0.67, 0.19];   % green
%    'A_Map1_2.mat','350.4406','Map1','A_Map2_2.mat','619.1509','Map2','A_Map3_2.mat','22140.0000','Map3',    '2/3/3/4/2',   [0.85, 0.33, 0.10];   % orange
%    'A_Map1_3.mat','350.4406','Map1','A_Map2_3.mat','619.1509','Map2','A_Map3_3.mat','22140.0000','Map3',    '3/3/3/4/2',   [0.93, 0.69, 0.13];   % yellow
%    'A_Map1_4.mat','350.4406','Map1','A_Map2_4.mat','619.1509','Map2','A_Map3_4.mat','22140.0000','Map3',    '4/3/3/4/2',   [0.00, 0.45, 0.74];   % blue
%    'A_Map1_5.mat','350.4406','Map1','A_Map2_5.mat','619.1509','Map2','A_Map3_5.mat','22140.0000','Map3',    '5/3/3/4/2',   [0.30, 0.75, 0.93];   % cyan

%    'A_Map1_-1.mat','350.4406','Map1','A_Map2_-1.mat','619.1509','Map2','A_Map3_-1.mat','22140.0000','Map3',    '3/1/3/4/2',   [0.47, 0.67, 0.19];   % green
%    'A_Map1_-2.mat','350.4406','Map1','A_Map2_-2.mat','619.1509','Map2','A_Map3_-2.mat','22140.0000','Map3',    '3/2/3/4/2',   [0.85, 0.33, 0.10];   % orange
%    'A_Map1_3.mat' ,'350.4406','Map1','A_Map2_3.mat' ,'619.1509','Map2','A_Map3_3.mat' ,'22140.0000','Map3',    '3/3/3/4/2',   [0.93, 0.69, 0.13];   % yellow
%    'A_Map1_-4.mat','350.4406','Map1','A_Map2_-4.mat','619.1509','Map2','A_Map3_-4.mat','22140.0000','Map3',    '3/4/3/4/2',   [0.00, 0.45, 0.74];   % blue
%    'A_Map1_-5.mat','350.4406','Map1','A_Map2_-5.mat','619.1509','Map2','A_Map3_-5.mat','22140.0000','Map3',    '3/5/3/4/2',   [0.30, 0.75, 0.93];   % cyan

%    'A_Map1_--1.mat','350.4406','Map1','A_Map2_--1.mat','619.1509','Map2','A_Map3_--1.mat','22140.0000','Map3',    '3/3/1/4/2',   [0.47, 0.67, 0.19];   % green
%    'A_Map1_--2.mat','350.4406','Map1','A_Map2_--2.mat','619.1509','Map2','A_Map3_--2.mat','22140.0000','Map3',    '3/3/2/4/2',   [0.85, 0.33, 0.10];   % orange
%    'A_Map1_3.mat' ,'350.4406' ,'Map1','A_Map2_3.mat'  ,'619.1509','Map2','A_Map3_3.mat'  ,'22140.0000','Map3',    '3/3/3/4/2',   [0.93, 0.69, 0.13];   % yellow
%    'A_Map1_--4.mat','350.4406','Map1','A_Map2_--4.mat','619.1509','Map2','A_Map3_--4.mat','22140.0000','Map3',    '3/3/4/4/2',   [0.00, 0.45, 0.74];   % blue
%    'A_Map1_--5.mat','350.4406','Map1','A_Map2_--5.mat','619.1509','Map2','A_Map3_--5.mat','22140.0000','Map3',    '3/3/5/4/2',   [0.30, 0.75, 0.93];   % cyan

%    'A_Map1_---1.mat','350.4406','Map1','A_Map2_---1.mat','619.1509','Map2','A_Map3_---1.mat','22140.0000','Map3',    '3/3/3/1/2',   [0.47, 0.67, 0.19];   % green
%    'A_Map1_---2.mat','350.4406','Map1','A_Map2_---2.mat','619.1509','Map2','A_Map3_---2.mat','22140.0000','Map3',    '3/3/3/2/2',   [0.85, 0.33, 0.10];   % orange
%    'A_Map1_---3.mat','350.4406','Map1','A_Map2_---3.mat','619.1509','Map2','A_Map3_---3.mat','22140.0000','Map3',    '3/3/3/3/2',   [0.93, 0.69, 0.13];   % yellow
%    'A_Map1_3.mat','350.4406'   ,'Map1','A_Map2_3.mat'   ,'619.1509','Map2','A_Map3_3.mat'   ,'22140.0000','Map3',    '3/3/3/4/2',   [0.00, 0.45, 0.74];   % blue
%    'A_Map1_---5.mat','350.4406','Map1','A_Map2_---5.mat','619.1509','Map2','A_Map3_---5.mat','22140.0000','Map3',    '3/3/3/5/2',   [0.30, 0.75, 0.93];   % cyan

   'A_Map1_----1.mat','350.4406','Map1','A_Map2_----1.mat','619.1509','Map2','A_Map3_----1.mat','22140.0000','Map3',    '3/3/3/4/1',   [0.47, 0.67, 0.19];   % green
   'A_Map1_3.mat'    ,'350.4406','Map1','A_Map2_3.mat'    ,'619.1509','Map2','A_Map3_3.mat'    ,'22140.0000','Map3',    '3/3/3/4/2',   [0.85, 0.33, 0.10];   % orange
   'A_Map1_----3.mat','350.4406','Map1','A_Map2_----3.mat','619.1509','Map2','A_Map3_----3.mat','22140.0000','Map3',    '3/3/3/4/3',   [0.93, 0.69, 0.13];   % yellow
   'A_Map1_----4.mat','350.4406','Map1','A_Map2_----4.mat','619.1509','Map2','A_Map3_----4.mat','22140.0000','Map3',    '3/3/3/4/4',   [0.00, 0.45, 0.74];   % blue
   'A_Map1_----5.mat','350.4406','Map1','A_Map2_----5.mat','619.1509','Map2','A_Map3_----5.mat','22140.0000','Map3',    '3/3/3/4/5',   [0.30, 0.75, 0.93];   % cyan

   % 在此添加更多组，每个组一行
};
nAlgo = size(algoLabels, 1);       % 组数
nSub  = 3;                          % 每组最大子文件数

% 提取组名和颜色
groupNames = algoLabels(:,10);
defaultColors = lines(nAlgo);       % 自动分配颜色
colors = zeros(nAlgo, 3);
for ai = 1:nAlgo
    if size(algoLabels,2) >= 11 && ~isempty(algoLabels{ai,11})
        colors(ai,:) = algoLabels{ai,11};
    else
        colors(ai,:) = defaultColors(ai,:);
    end
end

% 提取子标签（每组同一子索引的标签可能不同，取第一个非空）
subLabels = cell(nSub,1);
for si = 1:nSub
    labels = algoLabels(:, 3*si);  % 第3,6,9列
    idx = find(~cellfun(@isempty, labels), 1);
    if ~isempty(idx)
        subLabels{si} = labels{idx};
    else
        subLabels{si} = sprintf('Sub %d', si);
    end
end

% 提取基线值（矩阵 nAlgo x nSub，未使用的子文件为 NaN）
baselines = nan(nAlgo, nSub);
for ai = 1:nAlgo
    for si = 1:nSub
        baseCol = 3*si - 1;   % 第2,5,8列
        if ~isempty(algoLabels{ai, baseCol})
            val = algoLabels{ai, baseCol};
            if ischar(val) || isstring(val)
                baselines(ai, si) = str2double(val);
            else
                baselines(ai, si) = val;
            end
        end
    end
end

tolerance = 1e-12;

%% ===== 加载所有数据 =====
allData = cell(nAlgo, nSub);   % 每个元素为struct或空
for ai = 1:nAlgo
    for si = 1:nSub
        fileCol = 3*si - 2;   % 第1,4,7列
        fname = algoLabels{ai, fileCol};
        if isempty(fname)
            continue;
        end
        fullname = fullfile(resultsDir, fname);
        if ~exist(fullname, 'file')
            warning('Missing: %s', fullname);
            continue;
        end
        data = load(fullname);
        % 统一字段
        data.costs = [];
        if isfield(data, 'allCosts'), data.costs = data.allCosts; end
        data.histories = [];
        if isfield(data, 'allHistories'), data.histories = data.allHistories; end
        data.st = [];
        if isfield(data, 'stats'), data.st = data.stats; end
        % 若缺少统计信息则计算
        if isempty(data.st) || ~isfield(data.st, 'bestCost')
            data.st = struct();
            [data.st.bestCost, ~] = min(data.costs);
            data.st.worstCost   = max(data.costs);
            data.st.avgCost     = mean(data.costs);
            data.st.stdCost     = std(data.costs);
            data.st.medianCost  = median(data.costs);
        end
        allData{ai,si} = data;
        fprintf('Loaded %20s (group %s, sub %s): Best=%.4f  Avg=%.4f±%.4f\n', ...
            fname, groupNames{ai}, subLabels{si}, ...
            data.st.bestCost, data.st.avgCost, data.st.stdCost);
    end
end

%% ===== Figure 4: Average OptimalCost per Algorithm Group (Line, log scale) =====
figure('Position', [50, 50, 900, 500], 'Color', 'w');
avgOptCosts = nan(nAlgo, nSub);
for ai = 1:nAlgo
    for si = 1:nSub
        data = allData{ai,si};
        if isempty(data) || isempty(data.histories)
            continue;
        end
        runCosts = nan(1, length(data.histories));
        for r = 1:length(data.histories)
            h = data.histories{r};
            if isfield(h, 'bestCostHistory') && ~isempty(h.bestCostHistory)
                runCosts(r) = h.bestCostHistory(h.iterCount);
            end
        end
        avgOptCosts(ai,si) = mean(runCosts, 'omitnan');
    end
end

% 计算差值（成本 - 基线）
avgDiff = avgOptCosts - baselines;
% 对数坐标下，将非正值替换为一个很小的正数（以便显示）
minPositive = 1e-6;
avgDiffPlot = avgDiff;
avgDiffPlot(avgDiffPlot <= 0) = minPositive;

hold on;
% 子文件形状定义
markers = {'o', 's', '^', 'd', 'v', '>', '<', 'p', 'h'};

% 绘制每个子文件的折线和点
for si = 1:nSub
    if all(isnan(avgDiffPlot(:,si)))
        continue;   % 该子标签没有任何数据
    end
    x = 1:nAlgo;
    y = avgDiffPlot(:,si);
    
    % 绘制点（使用组颜色和子形状）
    for ai = 1:nAlgo
        if ~isnan(y(ai))
            plot(ai, y(ai), 'Marker', markers{si}, 'Color', colors(ai,:), ...
                'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, ...
                'LineStyle', 'none', 'HandleVisibility', 'off');
            % 标注实际值与基线的差值
            text(ai, y(ai)*1.2, sprintf('%.3f', avgDiff(ai,si)), ...
                'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
        end
    end
    
    % 连接相邻点，折线颜色统一灰色
    validIdx = find(~isnan(y));
    for k = 1:length(validIdx)-1
        i1 = validIdx(k);
        i2 = validIdx(k+1);
        if i2 == i1 + 1   % 仅连接相邻组
            plot([i1, i2], [y(i1), y(i2)], '-', 'Color', [0.5 0.5 0.5], ...
                'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end
    
    % 创建图例项（黑色标记，仅显示形状）
    plot(NaN, NaN, 'Marker', markers{si}, 'Color', 'k', 'MarkerFaceColor', 'k', ...
        'MarkerSize', 8, 'LineStyle', 'none', 'DisplayName', subLabels{si});
end

set(gca, 'XTick', 1:nAlgo, 'XTickLabel', groupNames, 'YScale', 'log');
ylabel('Cost − baseline (log)');
title('Average OptimalCost per Algorithm Group');
grid on;
hold off;
legend('show', 'Location', 'best');

%% ===== Figure 5: Average TimeToOptimal per Algorithm Group (Line) =====
figure('Position', [50, 50, 900, 500], 'Color', 'w');
avgTTO = nan(nAlgo, nSub);
for ai = 1:nAlgo
    for si = 1:nSub
        data = allData{ai,si};
        if isempty(data) || isempty(data.histories)
            continue;
        end
        runTTO = nan(1, length(data.histories));
        for r = 1:length(data.histories)
            h = data.histories{r};
            if ~isfield(h, 'bestCostHistory') || ~isfield(h, 'timeHistory')
                continue;
            end
            best = h.bestCostHistory(1:h.iterCount);
            time = h.timeHistory(1:h.iterCount);
            if isempty(best) || length(best) ~= length(time)
                continue;
            end
            finalCost = best(end);
            idx = find(abs(best - finalCost) <= tolerance * max(1, abs(finalCost)), 1, 'first');
            if isempty(idx)
                idx = find(best == finalCost, 1, 'first');
            end
            if ~isempty(idx)
                runTTO(r) = time(idx);
            end
        end
        avgTTO(ai,si) = mean(runTTO, 'omitnan');
    end
end

hold on;
for si = 1:nSub
    if all(isnan(avgTTO(:,si)))
        continue;
    end
    x = 1:nAlgo;
    y = avgTTO(:,si);
    
    % 绘制点
    for ai = 1:nAlgo
        if ~isnan(y(ai))
            plot(ai, y(ai), 'Marker', markers{si}, 'Color', colors(ai,:), ...
                'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, ...
                'LineStyle', 'none', 'HandleVisibility', 'off');
            text(ai, y(ai) + max(y, [], 'omitnan')*0.03, sprintf('%.2f ms', y(ai)*1000), ...
                'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
        end
    end
    
    % 连接相邻点，折线颜色统一灰色
    validIdx = find(~isnan(y));
    for k = 1:length(validIdx)-1
        i1 = validIdx(k);
        i2 = validIdx(k+1);
        if i2 == i1 + 1
            plot([i1, i2], [y(i1), y(i2)], '-', 'Color', [0.5 0.5 0.5], ...
                'LineWidth', 1.5, 'HandleVisibility', 'off');
        end
    end
    
    % 图例项
    plot(NaN, NaN, 'Marker', markers{si}, 'Color', 'k', 'MarkerFaceColor', 'k', ...
        'MarkerSize', 8, 'LineStyle', 'none', 'DisplayName', subLabels{si});
end

set(gca, 'XTick', 1:nAlgo, 'XTickLabel', groupNames);
ylabel('Average TimeToOptimal (s)');
title('Average TimeToOptimal per Algorithm Group');
grid on;
hold off;
legend('show', 'Location', 'best');

fprintf('\nAll figures ready. (Not auto-saved)\n');