%% plot_compare — TSP算法在kroA150上性能对比绘图（仅 Figure 4，支持每组多个子文件）
%  从 results/*.mat 加载数据，绘制对比图
%  usage: 将 results/ 下的 .mat 文件名填到 algoLabels 的映射中即可
%  每行格式：file1, base1, label1, file2, base2, label2, file3, base3, label3, groupName, color
%  最多支持3个子文件（可留空，对应不同 map）。图例按 map 颜色区分（蓝/橙/绿）。
%  Figure 4 双 Y 轴：左轴 OptimalCost 折线（圆形数据点、黑色线段、黑色描边），
%  右轴 TimeToOptimal 柱状图；同一 X 轴项的柱体紧贴（0 间距）；
%  不同 map 用不同颜色区分（折线填充色与柱体共用），图例提示为方块；
%  图例含 map 配色方块，并注明折线=OptimalCost、柱状=TimeToOptimal；全局字号 14。
%  基线值 base 用于图4左轴 Y 轴平移，各子文件可独立设置。
%  列11颜色已不再使用（颜色改为按 map 固定分配）。

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

% ===== Config: 每组算法可包含多个子文件（最多3个），共享组名 =====
%  列1: 子文件1文件名     列2: 子文件1基线值     列3: 子文件1标签
%  列4: 子文件2文件名     列5: 子文件2基线值     列6: 子文件2标签
%  列7: 子文件3文件名     列8: 子文件3基线值     列9: 子文件3标签
%  列10: 组显示名         列11: 颜色（已废弃，保留兼容；实际配色按 map 固定）
algoLabels = {
%    'data1.mat', 620.1509, '标签名1', 'data2.mat', 620.1509, '标签名2', 'data3.mat', 620.1509, '标签名3', '3/3/3/4/2', [0.47, 0.67, 0.19];
%n_ants/q_0/y_max/r_elite/k

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

% 提取组名（X 轴刻度）
groupNames = algoLabels(:,10);

% 按 map（子文件）固定配色：折线圆点与柱体共用；不再按 X 轴项着色
mapColors = [
    0.00, 0.45, 0.74;   % Map1 blue
    0.85, 0.33, 0.10;   % Map2 orange
    0.47, 0.67, 0.19;   % Map3 green
    0.49, 0.18, 0.56;   % extra purple
    0.30, 0.75, 0.93;   % extra cyan
];
if nSub > size(mapColors, 1)
    mapColors = [mapColors; lines(nSub - size(mapColors, 1))];
end
mapColors = mapColors(1:nSub, :);

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

%% ===== 计算 Average OptimalCost =====
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

% 计算差值（成本 - 基线）；对数坐标下非正值替换为很小的正数
avgDiff = avgOptCosts - baselines;
minPositive = 1e-6;
avgDiffPlot = avgDiff;
avgDiffPlot(avgDiffPlot <= 0) = minPositive;

%% ===== 计算 Average TimeToOptimal =====
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

%% ===== Figure 4: OptimalCost 折线（左轴）+ TimeToOptimal 柱状（右轴） =====
%  叠加两个坐标轴：底层画柱（右轴），顶层画折线（左轴），保证折线不被柱体遮挡
figure('Position', [50, 50, 1000, 560], 'Color', 'w');
plotPos = [0.10 0.12 0.80 0.80];
fontSize = 14;   % 全局字号

% --- 底层坐标轴：TimeToOptimal 柱状图（右轴，线性） ---
axBar = axes('Position', plotPos, 'FontSize', fontSize);
hold(axBar, 'on');
barW = 0.8 * 0.8 / nSub;                   % 柱宽（原 0.8/nSub 的 0.8 倍）；组内仍紧贴
xCenterOff = (1:nSub) - (nSub + 1) / 2;     % 各 map 在组内的偏移系数
for si = 1:nSub
    for ai = 1:nAlgo
        yv = avgTTO(ai, si);
        if isnan(yv)
            continue;
        end
        xl = ai + xCenterOff(si) * barW - barW / 2;
        patch([xl, xl + barW, xl + barW, xl], [0, 0, yv, yv], mapColors(si,:), ...
            'Parent', axBar, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    end
end
yMaxTTO = max(avgTTO(:), [], 'omitnan');
if ~isempty(yMaxTTO) && isfinite(yMaxTTO) && yMaxTTO > 0
    ylim(axBar, [0, yMaxTTO * 1.10]);
else
    ylim(axBar, [0, 1]);
end
xlim(axBar, [0.5, nAlgo + 0.5]);
set(axBar, ...
    'YAxisLocation', 'right', ...
    'YColor', [0.25 0.25 0.25], ...
    'XTick', 1:nAlgo, 'XTickLabel', groupNames, ...
    'Color', 'none', 'Box', 'on', ...
    'FontSize', fontSize);
ylabel(axBar, 'Average TimeToOptimal (s)', 'FontSize', fontSize);
grid(axBar, 'on');

% --- 顶层坐标轴：OptimalCost 折线（左轴，对数；黑色线段 + 圆点填色/黑描边） ---
axLine = axes('Position', plotPos, 'Color', 'none', 'FontSize', fontSize);
hold(axLine, 'on');
for si = 1:nSub
    if all(isnan(avgDiffPlot(:,si)))
        continue;
    end
    y = avgDiffPlot(:,si);
    % 折线：黑色；数据点：圆形、map 填充色、黑色描边
    plot(axLine, 1:nAlgo, y, '-o', ...
        'Color', [0 0 0], ...
        'MarkerFaceColor', mapColors(si,:), ...
        'MarkerEdgeColor', [0 0 0], ...
        'MarkerSize', 8, 'LineWidth', 1.5, ...
        'HandleVisibility', 'off');
    % 图例提示：方块（map 颜色 + 黑描边）
    plot(axLine, NaN, NaN, 's', ...
        'Color', mapColors(si,:), ...
        'MarkerFaceColor', mapColors(si,:), ...
        'MarkerEdgeColor', [0 0 0], ...
        'MarkerSize', 10, 'LineStyle', 'none', ...
        'DisplayName', subLabels{si});
end
% 图例：折线 = OptimalCost
plot(axLine, NaN, NaN, '-o', ...
    'Color', [0 0 0], ...
    'MarkerFaceColor', [0.65 0.65 0.65], ...
    'MarkerEdgeColor', [0 0 0], ...
    'MarkerSize', 8, 'LineWidth', 1.5, ...
    'DisplayName', 'OptimalCost');
% 图例：柱状 = TimeToOptimal
hBarLegend = patch(NaN, NaN, [0.65 0.65 0.65], ...
    'Parent', axLine, 'EdgeColor', 'none', ...
    'DisplayName', 'TimeToOptimal');
xlim(axLine, [0.5, nAlgo + 0.5]);
set(axLine, 'YScale', 'log', 'YColor', [0 0 0], ...
    'YAxisLocation', 'left', 'Box', 'off', 'XTick', [], ...
    'FontSize', fontSize);
% 对数轴上下留白，避免数据点贴边
yl = ylim(axLine);
if all(isfinite(yl)) && yl(2) > yl(1)
    ylim(axLine, [yl(1) * 0.7, yl(2) * 1.35]);
end
ylabel(axLine, 'Cost − baseline (log)', 'FontSize', fontSize);

linkaxes([axLine, axBar], 'x');

legend(axLine, 'show', 'Location', 'best', 'FontSize', fontSize);
hold(axBar, 'off');
hold(axLine, 'off');

fprintf('\nAll figures ready. (Not auto-saved)\n');