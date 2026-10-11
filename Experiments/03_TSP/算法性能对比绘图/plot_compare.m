%% plot_compare — TSP算法在kroA150上性能对比绘图
%  从 results/*.mat 加载数据，绘制对比图
%  usage: 将 results/ 下的 .mat 文件名填到 algoLabels 的映射中即可

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

% 全局字体大小统一为 16（坐标轴刻度、标题、标签、图例、文本）
set(0, 'DefaultAxesFontSize', 16);
set(0, 'DefaultTextFontSize', 16);
set(0, 'DefaultLegendFontSize', 16);

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

% ===== Config: map result files → display labels → color (optional) =====
%  Column 1: result .mat filename
%  Column 2: display name in legend
%  Column 3: color [R,G,B] (optional — auto-assigned if omitted or empty)
algoLabels = {

%   Map 3 最优结果：3 52.56
   'C_Map1_1.mat',            'MMAS-VND-CL',   [0.62, 0.75, 0.45];   % green
   'C_Map1_2.mat',            'ACO',           [0.90, 0.53, 0.47];   % orange
   'C_Map1_3.mat',            'GA',            [0.96, 0.82, 0.47];   % yellow
   'C_Map1_4.mat',            'SA',            [0.27, 0.60, 0.82];   % blue


%   Map 2 最优结果：6 23.78
%    'C_Map2_1.mat',            'MMAS-VND-CL',   [0.62, 0.75, 0.45];   % green
%    'C_Map2_2.mat',            'ACO',           [0.90, 0.53, 0.47];   % orange
%    'C_Map2_3.mat',            'GA',            [0.96, 0.82, 0.47];   % yellow
%    'C_Map2_4.mat',            'SA',            [0.27, 0.60, 0.82];   % blue


%    Map 3 最优结果：2 2141
%    'C_Map3_1.mat',            'MMAS-VND-CL',   [0.62, 0.75, 0.45];   % green
%    'C_Map3_2.mat',            'ACO',           [0.90, 0.53, 0.47];   % orange
%    'C_Map3_3.mat',            'GA',            [0.96, 0.82, 0.47];   % yellow
%    'C_Map3_4.mat',            'SA',            [0.27, 0.60, 0.82];   % blue

};
algoFiles    = algoLabels(:,1);
displayNames = algoLabels(:,2);
nAlgo = length(algoFiles);

% Build colors: use user-specified color if provided in column 3, else auto-assign
defaultColors = lines(nAlgo);
colors = zeros(nAlgo, 3);
for ai = 1:nAlgo
    if size(algoLabels, 2) >= 3 && ~isempty(algoLabels{ai, 3})
        colors(ai,:) = algoLabels{ai, 3};
    else
        colors(ai,:) = defaultColors(ai,:);
    end
end
knownOpt = 352.56;

%% ===== Load all data =====
allData = cell(1, nAlgo);
for ai = 1:nAlgo
    fname = fullfile(resultsDir, algoFiles{ai});
    if ~exist(fname, 'file')
        warning('Missing: %s', fname); allData{ai} = []; continue;
    end
    data = load(fname);
    % All result files use the same field names
    data.costs = [];
    if isfield(data, 'allCosts'), data.costs = data.allCosts; end
    data.histories = [];
    if isfield(data, 'allHistories'), data.histories = data.allHistories; end
    data.st = [];
    if isfield(data, 'stats'), data.st = data.stats; end
    % Compute stats if missing
    if isempty(data.st) || ~isfield(data.st, 'bestCost')
        data.st = struct();
        [data.st.bestCost, data.st.bestRunIdx] = min(data.costs);
        data.st.worstCost   = max(data.costs);
        data.st.avgCost     = mean(data.costs);
        data.st.stdCost     = std(data.costs);
        data.st.medianCost  = median(data.costs);
    end
    allData{ai} = data;
    fprintf('Loaded %20s: Best=%.1f  Avg=%.1f±%.1f  Gap=%.1f%%\n', algoFiles{ai}, ...
        data.st.bestCost, data.st.avgCost, data.st.stdCost, ...
        (data.st.bestCost - knownOpt) / knownOpt * 100);
end

%% ===== Figure 1: Convergence (Iteration) — all algorithms overlaid =====
baseline = 351.56;
figure('Position', [50, 50, 900, 600], 'Color', 'w');
hold on;
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    nR = length(data.histories);
    maxIter = max(cellfun(@(h) h.iterCount, data.histories));
    cm = nan(nR, maxIter);
    for r = 1:nR
        h = data.histories{r}; n = h.iterCount;
        cm(r, 1:n) = h.bestCostHistory(1:n);
        cm(r, n+1:end) = h.bestCostHistory(n);
    end
    mu  = mean(cm, 1, 'omitnan') - baseline;
    lo  = prctile(cm, 2.5, 1) - baseline; hi = prctile(cm, 97.5, 1) - baseline;
    x   = 1:maxIter;
    fill([x, fliplr(x)], [lo, fliplr(hi)], colors(ai,:), ...
        'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(x, mu, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, ...
        'DisplayName', displayNames{ai});
end
yline(knownOpt - baseline, '--k', '(352.56)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xlabel('Iteration'); ylabel('Best Cost − 351.56 (log)');
title(sprintf('Convergence — Iteration (Mean + 95%% CI, %d runs each)', nR));
set(gca, 'YScale', 'log');
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 2: Convergence (Time) — all algorithms overlaid =====
figure('Position', [50, 50, 900, 600], 'Color', 'w');
hold on;
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    nR = length(data.histories);
    runEndTimes = cellfun(@(h) h.elapsedTime, data.histories);
    medEndTime  = median(runEndTimes);
    nPts = 500; tC = linspace(0, medEndTime, nPts); cm = nan(nR, nPts);
    for r = 1:nR
        h = data.histories{r};
        tR = h.timeHistory(1:h.iterCount);
        cR = h.bestCostHistory(1:h.iterCount);
        tR = [0; tR(:)]; cR = [cR(1); cR(:)];
        [tU, ia] = unique(tR); cU = cR(ia);
        if length(tU) >= 2
            cm(r,:) = interp1(tU, cU, tC, 'linear', cU(end));
        else
            cm(r,:) = cU(end);
        end
    end
    % Only keep time points where >= half of runs are still active
    nActive = zeros(1, nPts);
    for p = 1:nPts, nActive(p) = sum(runEndTimes >= tC(p)); end
    validMask = nActive >= max(1, nR/2);
    cm(:, ~validMask) = NaN;
    mu  = mean(cm, 1, 'omitnan') - baseline;
    lo  = prctile(cm, 2.5, 1) - baseline; hi = prctile(cm, 97.5, 1) - baseline;
    fill([tC, fliplr(tC)], [lo, fliplr(hi)], colors(ai,:), ...
        'FaceAlpha', 0.08, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(tC, mu, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, ...
        'DisplayName', displayNames{ai});
end
yline(knownOpt - baseline, '--k', '(352.56)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Best Cost − 351.56 (log)');
title(sprintf('Convergence — Time (Mean + 95%% CI, %d runs each)', nR));
set(gca, 'YScale', 'log');
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 3: Cost Boxplot Comparison =====
figure('Position', [50, 50, 900, 600], 'Color', 'w');
boxData = []; boxGroup = [];
for ai = 1:nAlgo
    if ~isempty(allData{ai})
        c = allData{ai}.costs(:);
        boxData  = [boxData; c];
        boxGroup = [boxGroup; repmat(ai, length(c), 1)];
    end
end
% Apply baseline shift: subtract 351.56 so optimal=352.56 → y=1
boxData = boxData - 351.56;
h = boxplot(boxData, boxGroup, 'Labels', displayNames, 'Symbol', 'o');
for ai = 1:nAlgo
    set(h(:,ai), 'Color', colors(ai,:));
    % Median line → black
    if size(h,1) >= 6, set(h(6,ai), 'Color', 'k', 'LineWidth', 1.2); end
    % Outlier markers → filled circles (kept as-is)
    if size(h,1) >= 7
        set(h(7,ai), 'MarkerFaceColor', colors(ai,:), 'MarkerSize', 5);
    end
end
hold on;
% Overlay all data points as jittered scatter
jitterWidth = 0.25;
for ai = 1:nAlgo
    if ~isempty(allData{ai})
        cShifted = allData{ai}.costs(:) - 351.56;
        nPts = length(cShifted);
        xJitter = ai + jitterWidth * (rand(nPts, 1) - 0.5);
        scatter(xJitter, cShifted, 12, colors(ai,:), 'filled', ...
            'MarkerFaceAlpha', 0.5, 'MarkerEdgeColor', 'none');
    end
end
yline(1, '--k', '(352.56)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom');
set(gca, 'YScale', 'log');
ylabel('Best Cost − 351.56 (log)');
title(sprintf('Cost Distribution Comparison (%d runs each)', nR));
grid on;

%% ===== Figure 4: Average OptimalCost + TimeToOptimal per Algorithm (Bar + Line) =====
figure('Position', [50, 50, 900, 600], 'Color', 'w');
avgOptCosts = nan(1, nAlgo);
avgTTO = nan(1, nAlgo);
tolerance = 1e-12;
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    runCosts = nan(1, length(data.histories));
    runTTO = nan(1, length(data.histories));
    for r = 1:length(data.histories)
        h = data.histories{r};
        if isfield(h, 'bestCostHistory') && ~isempty(h.bestCostHistory)
            runCosts(r) = h.bestCostHistory(h.iterCount);
        end
        if isfield(h, 'bestCostHistory') && isfield(h, 'timeHistory') && ...
           ~isempty(h.bestCostHistory) && ~isempty(h.timeHistory)
            best = h.bestCostHistory(1:h.iterCount);
            time = h.timeHistory(1:h.iterCount);
            if length(best) == length(time)
                finalCost = best(end);
                idx = find(abs(best - finalCost) <= tolerance * max(1, abs(finalCost)), 1, 'first');
                if isempty(idx), idx = find(best == finalCost, 1, 'first'); end
                if ~isempty(idx), runTTO(r) = time(idx); end
            end
        end
    end
    avgOptCosts(ai) = mean(runCosts, 'omitnan');
    avgTTO(ai) = mean(runTTO, 'omitnan');
end

avgTTO_ms = avgTTO * 1000;                 % 平均耗时（毫秒）

% 双标尺用「两个重叠 Axes」实现：yyaxis 是单 Axes 双标尺，其右标尺上的对象恒在左标尺
% 对象之上（uistack 亦被禁止），无法把折线画到柱状图之上；重叠 Axes 则按创建顺序叠放。
% 先建柱状图轴（下层，右标尺），再建折线轴（上层，左标尺，背景透明以露出柱子）。
axPos  = [0.13, 0.11, 0.775, 0.815];       % 常规坐标轴位置，四周留出标签空间
axBar  = axes('Parent', gcf, 'Position', axPos, 'Color', 'w', ...
    'YAxisLocation', 'right', 'Box', 'on', 'FontSize', 16);
hold(axBar, 'on');
b = bar(axBar, 1:nAlgo, avgTTO_ms, 0.6, 'FaceColor', 'flat', ...
    'EdgeColor', 'k', 'LineWidth', 0.5, 'DisplayName', 'Average TimeToOptimal');
for ai = 1:nAlgo
    b.CData(ai,:) = colors(ai,:);
end
set(axBar, 'YScale', 'log');               % 各算法耗时差异大，用 log 标尺
ylabel(axBar, 'Average TimeToOptimal (ms, log)', 'FontSize', 16);
xlim(axBar, [0.5, nAlgo + 0.5]);
ylim(axBar, [min(avgTTO_ms) * 0.5, max(avgTTO_ms) * 1.05]);   % log 轴下界不能为 0
% 刻度位置保留用于网格线，标签留空（X 轴标签由上层折线轴提供，避免重复）
set(axBar, 'XTick', 1:nAlgo, 'XTickLabel', repmat({''}, 1, nAlgo), 'FontSize', 16);
grid(axBar, 'on');

% 上层：折线 — Average OptimalCost
axLine = axes('Parent', gcf, 'Position', axPos, 'Color', 'none', ...
    'YAxisLocation', 'left', 'Box', 'on', 'FontSize', 16);
hold(axLine, 'on');
hl = plot(axLine, 1:nAlgo, avgOptCosts, '-o', 'Color', [0.64, 0.08, 0.18], ...
    'LineWidth', 1.8, 'MarkerSize', 7, 'MarkerFaceColor', [0.64, 0.08, 0.18], ...
    'DisplayName', 'Average OptimalCost');
set(axLine, 'GridLineStyle', 'none');
yline(axLine, knownOpt, '--k', '(352.56)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom');
ylabel(axLine, 'Average OptimalCost', 'FontSize', 16);
xlim(axLine, [0.5, nAlgo + 0.5]);
% 折线轴范围收紧到数据附近（含最优参考线），避免宽范围压平各算法差异
optLims = [min([avgOptCosts, knownOpt]), max([avgOptCosts, knownOpt])];
padOpt  = max(0.5, 0.05 * diff(optLims));
ylim(axLine, optLims + [-padOpt, padOpt]);

% 图例（灰色色块代表柱状图，折线沿用自身颜色；两句柄同属上层轴，图例正常绘制）
hb = bar(axLine, nan(nAlgo,1), 0.6, 'FaceColor', [0.6 0.6 0.6], 'EdgeColor', 'k', ...
    'LineWidth', 0.5, 'DisplayName', 'Average TimeToOptimal');
legend(axLine, [hb, hl], 'Location', 'northwest');

set(axLine, 'XTick', 1:nAlgo, 'XTickLabel', displayNames, 'FontSize', 16);
title(axLine, sprintf('Average TimeToOptimal & OptimalCost per Algorithm (%d runs each)', nR));

fprintf('\nAll figures ready. (Not auto-saved)\n');
