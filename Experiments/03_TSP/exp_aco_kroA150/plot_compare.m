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
    '1.mat',                    'MMAS-VND-CL',   [0.47, 0.67, 0.19];   % green
%    '2.mat',                    'Algo #2',   [0.85, 0.33, 0.10];   % orange
%    '3.mat',                    'Algo #3',   [0.93, 0.69, 0.13];   % yellow
%    '4.mat',                    'Algo #4',   [0.00, 0.45, 0.74];   % blue
%    '5.mat',                    'Algo #5',   [0.30, 0.75, 0.93];   % cyan
%    '6.mat',                    'Algo #6',   [0.49, 0.18, 0.56];   % purple
%    'B1.mat',                   'ACO\_v2\_3', [0.64, 0.08, 0.18]; % maroon

    'B1.mat',                    'Standard ACO',   [0.85, 0.33, 0.10];   % orange
    'B2.mat',                    'GA',             [0.93, 0.69, 0.13];   % yellow
    'B3.mat',                    'SA',             [0.00, 0.45, 0.74];   % blue
    'B5.mat',                    'Improved GA',    [0.30, 0.75, 0.93];   % cyan
    'B6.mat',                    'Improved SA',    [0.49, 0.18, 0.56];   % purple
 
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
yline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0);
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
yline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0);
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
yline(1, '--k', 'Optimal (26524)', 'LineWidth', 1.0);
set(gca, 'YScale', 'log');
ylabel('Best Cost − 26523 (log)');
title(sprintf('Cost Distribution Comparison (%d runs each)', nR));
grid on;

%% ===== Figure 4: Best Cost Bar Chart =====
figure('Position', [50, 50, 850, 500], 'Color', 'w');
bestVals  = zeros(1, nAlgo);
avgVals   = zeros(1, nAlgo);
for ai = 1:nAlgo
    if isempty(allData{ai}), continue; end
    bestVals(ai) = allData{ai}.st.bestCost;
    avgVals(ai)  = allData{ai}.st.avgCost;
end
bar(bestVals, 'FaceColor', 'flat', 'CData', colors); hold on;
for ai = 1:nAlgo
    text(ai, bestVals(ai) + 15, sprintf('%.0f', bestVals(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end
set(gca, 'XTickLabel', displayNames, 'XTickLabelRotation', 15);
ylabel('Best Cost');
title('Best Cost by Algorithm');
yline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0);
grid on; hold off;

%% ===== Figure 5+: Individual Cost Distributions =====
for ai = 1:nAlgo
    data = allData{ai};
    if isempty(data), continue; end
    figure('Position', [50 + mod(ai-1,3)*350, 50 + floor((ai-1)/3)*400, 600, 450], 'Color', 'w');
    histogram(data.costs, 10, 'FaceColor', colors(ai,:), 'EdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    xline(data.st.bestCost, '--r', sprintf('Best=%.1f', data.st.bestCost), 'LineWidth', 1.5);
    xline(data.st.medianCost, '--', sprintf('Median=%.1f', data.st.medianCost), ...
        'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
    xline(knownOpt, '--k', 'Optimal (26524)', 'LineWidth', 1.0);
    xlabel('Best Cost'); ylabel('Frequency');
    title(sprintf('%s — Cost Distribution (Gap: %.1f%%)', displayNames{ai}, ...
        (data.st.bestCost - knownOpt) / knownOpt * 100));
    grid on; hold off;
end

fprintf('\nAll figures ready. (Not auto-saved)\n');
