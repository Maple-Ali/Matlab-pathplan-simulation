%% plot_compare — TSP算法在kroA150上性能对比绘图
%  从 results/*.mat 加载数据，绘制对比图
%  usage: 将 results/ 下的 .mat 文件名填到 algoLabels 的映射中即可

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

% ===== Config: map result files → display labels → color (optional) =====
%  Column 1: result .mat filename
%  Column 2: display name in legend
%  Column 3: color [R,G,B] (optional — auto-assigned if omitted or empty)
algoLabels = {

%   Map 3 最优结果：3 51.441
%    'B_Map1_1.mat',            'Complete Algorithm',   [0.47, 0.67, 0.19];   % green
%    'B_Map1_2.mat',            'No module A',          [0.85, 0.33, 0.10];   % orange
%    'B_Map1_3.mat',            'No module B',          [0.93, 0.69, 0.13];   % yellow
%    'B_Map1_4.mat',            'No module C',          [0.00, 0.45, 0.74];   % blue
%    'B_Map1_5.mat',            'No module D',          [0.30, 0.75, 0.93];   % cyan
%    'B_Map1_6.mat',            'No module E',          [0.49, 0.18, 0.56];   % purple

%   Map 2 最优结果：6 20.15
%    'B_Map2_1.mat',            'Complete Algorithm',   [0.47, 0.67, 0.19];   % green
%    'B_Map2_2.mat',            'No module A',          [0.85, 0.33, 0.10];   % orange
%    'B_Map2_3.mat',            'No module B',          [0.93, 0.69, 0.13];   % yellow
%    'B_Map2_4.mat',            'No module C',          [0.00, 0.45, 0.74];   % blue
%    'B_Map2_5.mat',            'No module D',          [0.30, 0.75, 0.93];   % cyan
%    'B_Map2_6.mat',            'No module E',          [0.49, 0.18, 0.56];   % purple
%    'name.mat',                  'show_name', [0.64, 0.08, 0.18]; % maroon

%    Map 3 最优结果：2 2141
   'B_Map3_1.mat',            'Complete Algorithm',   [0.47, 0.67, 0.19];   % green
   'B_Map3_2.mat',            'No module A',          [0.85, 0.33, 0.10];   % orange
   'B_Map3_3.mat',            'No module B',          [0.93, 0.69, 0.13];   % yellow
   'B_Map3_4.mat',            'No module C',          [0.00, 0.45, 0.74];   % blue
   'B_Map3_5.mat',            'No module D',          [0.30, 0.75, 0.93];   % cyan
   'B_Map3_6.mat',            'No module E',          [0.49, 0.18, 0.56];   % purple

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
knownOpt = 22141;

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
baseline = 22140;
figure('Position', [50, 50, 1000, 650], 'Color', 'w');
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
yline(knownOpt - baseline, '--k', '(22141)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xlabel('Iteration'); ylabel('Best Cost − 619.15 (log)');
title(sprintf('Convergence — Iteration (Mean + 95%% CI, %d runs each)', nR));
set(gca, 'YScale', 'log');
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 2: Convergence (Time) — all algorithms overlaid =====
figure('Position', [50, 50, 1000, 650], 'Color', 'w');
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
yline(knownOpt - baseline, '--k', '(22141)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Best Cost − 619.15 (log)');
title(sprintf('Convergence — Time (Mean + 95%% CI, %d runs each)', nR));
set(gca, 'YScale', 'log');
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 3: Cost Boxplot Comparison =====
figure('Position', [50, 50, 1000, 550], 'Color', 'w');
boxData = []; boxGroup = [];
for ai = 1:nAlgo
    if ~isempty(allData{ai})
        c = allData{ai}.costs(:);
        boxData  = [boxData; c];
        boxGroup = [boxGroup; repmat(ai, length(c), 1)];
    end
end
% Apply baseline shift: subtract 22140 so optimal=22141 → y=1
boxData = boxData - 22140;
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
        cShifted = allData{ai}.costs(:) - 22140;
        nPts = length(cShifted);
        xJitter = ai + jitterWidth * (rand(nPts, 1) - 0.5);
        scatter(xJitter, cShifted, 12, colors(ai,:), 'filled', ...
            'MarkerFaceAlpha', 0.5, 'MarkerEdgeColor', 'none');
    end
end
yline(1, '--k', '(22141)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom');
set(gca, 'YScale', 'log');
ylabel('Best Cost − 22140 (log)');
title(sprintf('Cost Distribution Comparison (%d runs each)', nR));
grid on;

%% ===== Figure 4: Average OptimalCost per Algorithm (Bar) =====
figure('Position', [50, 50, 900, 500], 'Color', 'w');
avgOptCosts = nan(1, nAlgo);
tolerance = 1e-12;
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    runCosts = nan(1, length(data.histories));
    for r = 1:length(data.histories)
        h = data.histories{r};
        if isfield(h, 'bestCostHistory') && ~isempty(h.bestCostHistory)
            runCosts(r) = h.bestCostHistory(h.iterCount);
        end
    end
    avgOptCosts(ai) = mean(runCosts, 'omitnan');
end
hold on;
b = bar(1:nAlgo, avgOptCosts, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'k', 'LineWidth', 0.5);
for ai = 1:nAlgo
    b.CData(ai,:) = colors(ai,:);
end
for ai = 1:nAlgo
    text(ai, avgOptCosts(ai) + max(avgOptCosts)*0.01, sprintf('%.3f', avgOptCosts(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end
yline(knownOpt, '--k', '(22141)', 'LineWidth', 0.5, ...
    'LabelVerticalAlignment', 'bottom');
set(gca, 'XTick', 1:nAlgo, 'XTickLabel', displayNames, 'XTickLabelRotation', 15);
ylabel('Average OptimalCost');
title(sprintf('Average OptimalCost per Algorithm (%d runs each)', nR));
grid on; hold off;

%% ===== Figure 5: Average TimeToOptimal per Algorithm (Bar) =====
figure('Position', [50, 50, 900, 500], 'Color', 'w');
avgTTO = nan(1, nAlgo);
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    runTTO = nan(1, length(data.histories));
    for r = 1:length(data.histories)
        h = data.histories{r};
        if ~isfield(h, 'bestCostHistory') || ~isfield(h, 'timeHistory'), continue; end
        best = h.bestCostHistory(1:h.iterCount);
        time = h.timeHistory(1:h.iterCount);
        if isempty(best) || length(best) ~= length(time), continue; end
        finalCost = best(end);
        idx = find(abs(best - finalCost) <= tolerance * max(1, abs(finalCost)), 1, 'first');
        if isempty(idx), idx = find(best == finalCost, 1, 'first'); end
        if ~isempty(idx), runTTO(r) = time(idx); end
    end
    avgTTO(ai) = mean(runTTO, 'omitnan');
end
avgTTO_ms = avgTTO * 1000;  % 转换为毫秒
hold on;
b = bar(1:nAlgo, avgTTO_ms, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'k', 'LineWidth', 0.5);
for ai = 1:nAlgo
    b.CData(ai,:) = colors(ai,:);
end
for ai = 1:nAlgo
    text(ai, avgTTO_ms(ai) + max(avgTTO_ms)*0.01, sprintf('%.1f ms', avgTTO_ms(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end
set(gca, 'XTick', 1:nAlgo, 'XTickLabel', displayNames, 'XTickLabelRotation', 15);
ylabel('Average TimeToOptimal (ms)');
title(sprintf('Average TimeToOptimal per Algorithm (%d runs each, ms)', nR));
grid on; hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');
