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
   'C33442.mat',                'MMAS-VND-CL',   [0.47, 0.67, 0.19];   % green
   'A2.mat',                    'No module A',   [0.85, 0.33, 0.10];   % orange
   'A3.mat',                    'No module B',   [0.93, 0.69, 0.13];   % yellow
   'A4.mat',                    'No module C',   [0.00, 0.45, 0.74];   % blue
   'A5.mat',                    'No module D',   [0.30, 0.75, 0.93];   % cyan
   'A6.mat',                    'No module E',   [0.49, 0.18, 0.56];   % purple
%    'name.mat',                  'show_name', [0.64, 0.08, 0.18]; % maroon

    % 'C33442.mat',                'MMAS-VND-CL',   [0.47, 0.67, 0.19];   % green
    % 'B2.mat',                    'Standard ACO',   [0.85, 0.33, 0.10];   % orange
    % 'B3.mat',                    'GA',             [0.93, 0.69, 0.13];   % yellow
    % 'B4.mat',                    'SA',             [0.00, 0.45, 0.74];   % blue
    % 'B5.mat',                    'Improved GA',    [0.30, 0.75, 0.93];   % cyan
    % 'B6.mat',                    'Improved SA',    [0.49, 0.18, 0.56];   % purple
 
    % 'C13442.mat',                    '1/3/4/4/2',   [0.85, 0.33, 0.10];   % orange
    % 'C23442.mat',                    '2/3/4/4/2',   [0.93, 0.69, 0.13];   % yellow
    % 'C33442.mat',                    '3/3/4/4/2',   [0.00, 0.45, 0.74];   % blue
    % 'C43442.mat',                    '4/3/4/4/2',   [0.30, 0.75, 0.93];   % cyan
    % 'C53442.mat',                    '5/3/4/4/2',   [0.49, 0.18, 0.56];   % purple

    % 'C31442.mat',                    '3/1/4/4/2',   [0.85, 0.33, 0.10];   % orange
    % 'C32442.mat',                    '3/2/4/4/2',   [0.93, 0.69, 0.13];   % yellow
    % 'C33442.mat',                    '3/3/4/4/2',   [0.00, 0.45, 0.74];   % blue
    % 'C34442.mat',                    '3/4/4/4/2',   [0.30, 0.75, 0.93];   % cyan
    % 'C35442.mat',                    '3/5/4/4/2',   [0.49, 0.18, 0.56];   % purple

    % 'C33142.mat',                    '3/3/1/4/2',   [0.85, 0.33, 0.10];   % orange
    % 'C33242.mat',                    '3/3/2/4/2',   [0.93, 0.69, 0.13];   % yellow
    % 'C33342.mat',                    '3/3/3/4/2',   [0.00, 0.45, 0.74];   % blue
    % 'C33442.mat',                    '3/3/4/4/2',   [0.30, 0.75, 0.93];   % cyan
    % 'C33542.mat',                    '3/3/5/4/2',   [0.49, 0.18, 0.56];   % purple

    % 'C33412.mat',                    '3/3/4/1/2',   [0.85, 0.33, 0.10];   % orange
    % 'C33422.mat',                    '3/3/4/2/2',   [0.93, 0.69, 0.13];   % yellow
    % 'C33432.mat',                    '3/3/4/3/2',   [0.00, 0.45, 0.74];   % blue
    % 'C33442.mat',                    '3/3/4/4/2',   [0.30, 0.75, 0.93];   % cyan
    % 'C33452.mat',                    '3/3/4/5/2',   [0.49, 0.18, 0.56];   % purple

    % 'C33441.mat',                    '3/3/4/4/1',   [0.85, 0.33, 0.10];   % orange
    % 'C33442.mat',                    '3/3/4/4/2',   [0.93, 0.69, 0.13];   % yellow
    % 'C33443.mat',                    '3/3/4/4/3',   [0.00, 0.45, 0.74];   % blue
    % 'C33444.mat',                    '3/3/4/4/4',   [0.30, 0.75, 0.93];   % cyan
    % 'C33445.mat',                    '3/3/4/4/5',   [0.49, 0.18, 0.56];   % purple
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
knownOpt = 26524;

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
    med = median(cm, 1, 'omitnan');
    lo  = prctile(cm, 2.5, 1); hi = prctile(cm, 97.5, 1);
    x   = 1:maxIter;
    fill([x, fliplr(x)], [lo, fliplr(hi)], colors(ai,:), ...
        'FaceAlpha', 0.08, 'EdgeColor', 'none');
    plot(x, med, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, ...
        'DisplayName', displayNames{ai});
end
yline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0, ...
    'LabelVerticalAlignment', 'bottom');
xlabel('Iteration'); ylabel('Best Cost (log)');
title(sprintf('Convergence — Iteration (Median + 95%% CI, %d runs each)', nR));
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
    med = median(cm, 1, 'omitnan');
    lo  = prctile(cm, 2.5, 1); hi = prctile(cm, 97.5, 1);
    fill([tC, fliplr(tC)], [lo, fliplr(hi)], colors(ai,:), ...
        'FaceAlpha', 0.08, 'EdgeColor', 'none');
    plot(tC, med, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, ...
        'DisplayName', displayNames{ai});
end
yline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0, ...
    'LabelVerticalAlignment', 'bottom');
xlabel('Time (s)'); ylabel('Best Cost (log)');
title(sprintf('Convergence — Time (Median + 95%% CI, %d runs each)', nR));
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
% Apply baseline shift: subtract 26523 so optimal=26524 → y=1
boxData = boxData - 26523;
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
        cShifted = allData{ai}.costs(:) - 26523;
        nPts = length(cShifted);
        xJitter = ai + jitterWidth * (rand(nPts, 1) - 0.5);
        scatter(xJitter, cShifted, 12, colors(ai,:), 'filled', ...
            'MarkerFaceAlpha', 0.5, 'MarkerEdgeColor', 'none');
    end
end
yline(1, '--k', 'Optimal (26524)', 'LineWidth', 1.0, ...
    'LabelVerticalAlignment', 'bottom');
set(gca, 'YScale', 'log');
ylabel('Best Cost − 26523 (log)');
title(sprintf('Cost Distribution Comparison (%d runs each)', nR));
grid on;

%% ===== Figure 4: Average OptimalCost per Algorithm (Line, log scale) =====
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
% Baseline shift + log scale (same as boxplot Figure 3)
avgShifted = avgOptCosts - 26523;
hold on;
for ai = 1:nAlgo
    if ai == 1 || isnan(avgShifted(ai-1))
        plot(ai, avgShifted(ai), '-o', 'Color', [0.4 0.4 0.4], ...
            'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, 'LineWidth', 1.5);
    else
        plot([ai-1, ai], [avgShifted(ai-1), avgShifted(ai)], '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
        plot(ai, avgShifted(ai), '-o', 'Color', [0.4 0.4 0.4], ...
            'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, 'LineWidth', 1.5);
    end
    text(ai, avgShifted(ai) * 1.15, sprintf('%.2f', avgOptCosts(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end
set(gca, 'XTick', 1:nAlgo, 'XTickLabel', displayNames, 'XTickLabelRotation', 15, 'YScale', 'log');
ylabel('Average OptimalCost − 26523 (log)');
title(sprintf('Average OptimalCost per Algorithm (%d runs each)', nR));
yline(1, '--k', 'Optimal (26524)', 'LineWidth', 1.0, ...
    'LabelVerticalAlignment', 'bottom');
grid on; hold off;

%% ===== Figure 5: Average TimeToOptimal per Algorithm (Line) =====
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
hold on;
for ai = 1:nAlgo
    if ai == 1 || isnan(avgTTO(ai-1))
        plot(ai, avgTTO(ai), '-o', 'Color', [0.4 0.4 0.4], ...
            'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, 'LineWidth', 1.5);
    else
        plot([ai-1, ai], [avgTTO(ai-1), avgTTO(ai)], '-', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
        plot(ai, avgTTO(ai), '-o', 'Color', [0.4 0.4 0.4], ...
            'MarkerFaceColor', colors(ai,:), 'MarkerSize', 8, 'LineWidth', 1.5);
    end
    text(ai, avgTTO(ai) + max(avgTTO)*0.03, sprintf('%.2f s', avgTTO(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end
set(gca, 'XTick', 1:nAlgo, 'XTickLabel', displayNames, 'XTickLabelRotation', 15);
ylabel('Average TimeToOptimal (s)');
title(sprintf('Average TimeToOptimal per Algorithm (%d runs each)', nR));
grid on; hold off;

fprintf('\nAll figures ready. (Not auto-saved)\n');
