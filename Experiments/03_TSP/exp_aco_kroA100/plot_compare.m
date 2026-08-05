%% plot_compare — 7种TSP算法在kroA100上性能对比绘图
%  从 results/*.mat 加载数据，绘制对比图

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');
algoNames = {'ACO', 'ACO_v0', 'ACO_v2_2', 'GA', 'GA_v1_1', 'SA_v0', 'SA_v0_1'};
displayNames = {'ACO', 'ACO_v0', 'ACO_v2_2', 'GA', 'GA_v1_1', 'SA_v0', 'SA_v0_1'};
colors = lines(length(algoNames));
knownOpt = 21282;

%% ===== Load all data =====
allData = cell(1, length(algoNames));
for ai = 1:length(algoNames)
    fname = fullfile(resultsDir, sprintf('kroA100_40_%s.mat', algoNames{ai}));
    if ~exist(fname, 'file')
        warning('Missing: %s', fname); allData{ai} = []; continue;
    end
    data = load(fname);
    data.costs = [];
    % Handle different field names from batch vs individual runs
    if isfield(data, 'ac'), data.costs = data.ac;
    elseif isfield(data, 'allCosts'), data.costs = data.allCosts; end
    data.histories = [];
    if isfield(data, 'ah'), data.histories = data.ah;
    elseif isfield(data, 'allHistories'), data.histories = data.allHistories; end
    data.st = [];
    if isfield(data, 's'), data.st = data.s;
    elseif isfield(data, 'stats'), data.st = data.stats; end
    % Compute stats if missing
    if isempty(data.st) || ~isfield(data.st,'bestCost')
        data.st = struct();
        [data.st.bestCost, data.st.bestRunIdx] = min(data.costs);
        data.st.worstCost = max(data.costs);
        data.st.avgCost = mean(data.costs);
        data.st.stdCost = std(data.costs);
        data.st.medianCost = median(data.costs);
    end
    allData{ai} = data;
    fprintf('Loaded %20s: Best=%.1f  Avg=%.1f±%.1f  Gap=%.1f%%\n', algoNames{ai}, ...
        data.st.bestCost, data.st.avgCost, data.st.stdCost, (data.st.bestCost-knownOpt)/knownOpt*100);
end

%% ===== Figure 1: Convergence (Iteration) — all algorithms overlaid =====
figure('Position', [50, 50, 1000, 650], 'Color', 'w');
hold on;
for ai = 1:length(algoNames)
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    nR = length(data.histories); maxIter = max(cellfun(@(h) h.iterCount, data.histories));
    cm = nan(nR, maxIter);
    for r = 1:nR, h = data.histories{r}; n = h.iterCount;
        cm(r, 1:n) = h.bestCostHistory(1:n); cm(r, n+1:end) = h.bestCostHistory(n);
    end
    med = median(cm, 1, 'omitnan');
    lo = prctile(cm, 2.5, 1); hi = prctile(cm, 97.5, 1); x = 1:maxIter;
    fill([x, fliplr(x)], [lo, fliplr(hi)], colors(ai,:), 'FaceAlpha', 0.08, 'EdgeColor', 'none');
    plot(x, med, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, 'DisplayName', displayNames{ai});
end
yline(knownOpt, '--k', 'Optimal (21282)', 'LineWidth', 1.0);
xlabel('Iteration'); ylabel('Best Cost');
title(sprintf('Convergence — Iteration (Median + 95%% CI, %d runs each)', nR));
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 2: Convergence (Time) — all algorithms overlaid =====
figure('Position', [50, 50, 1000, 650], 'Color', 'w');
hold on;
for ai = 1:length(algoNames)
    data = allData{ai};
    if isempty(data) || isempty(data.histories), continue; end
    nR = length(data.histories);
    % Collect all per-run elapsed times for validity check
    runEndTimes = cellfun(@(h) h.elapsedTime, data.histories);
    medEndTime = median(runEndTimes);  % stop curve at median end time
    nPts = 500; tC = linspace(0, medEndTime, nPts); cm = nan(nR, nPts);
    for r = 1:nR, h = data.histories{r};
        tR = h.timeHistory(1:h.iterCount); cR = h.bestCostHistory(1:h.iterCount);
        tR = [0; tR(:)]; cR = [cR(1); cR(:)]; [tU, ia] = unique(tR); cU = cR(ia);
        if length(tU)>=2, cm(r,:) = interp1(tU, cU, tC, 'linear', cU(end));
        else, cm(r,:) = cU(end); end
    end
    % Only include time points where >= half of runs are still active
    nActive = zeros(1, nPts);
    for p = 1:nPts, nActive(p) = sum(runEndTimes >= tC(p)); end
    validMask = nActive >= max(1, nR/2);
    cm(:, ~validMask) = NaN;
    med = median(cm, 1, 'omitnan'); lo = prctile(cm, 2.5, 1); hi = prctile(cm, 97.5, 1);
    fill([tC, fliplr(tC)], [lo, fliplr(hi)], colors(ai,:), 'FaceAlpha', 0.08, 'EdgeColor', 'none');
    plot(tC, med, '-', 'Color', colors(ai,:), 'LineWidth', 1.8, 'DisplayName', displayNames{ai});
end
yline(knownOpt, '--k', 'Optimal (21282)', 'LineWidth', 1.0);
xlabel('Time (s)'); ylabel('Best Cost');
title(sprintf('Convergence — Time (Median + 95%% CI, %d runs each)', nR));
legend('Location', 'northeast'); grid on; hold off;

%% ===== Figure 3: Cost boxplot comparison (all algorithms) =====
figure('Position', [50, 50, 1000, 550], 'Color', 'w');
% Build grouped data for boxplot
boxData = []; boxGroup = [];
for ai = 1:length(algoNames)
    if ~isempty(allData{ai})
        c = allData{ai}.costs(:);
        boxData = [boxData; c];
        boxGroup = [boxGroup; repmat(ai, length(c), 1)];
    end
end
h = boxplot(boxData, boxGroup, 'Labels', displayNames);
% Color the boxes
for ai = 1:length(algoNames)
    set(h(:,ai), 'Color', colors(ai,:));
end
yline(knownOpt, '--k', 'Optimal (21282)', 'LineWidth', 1.0);
ylabel('Best Cost'); title(sprintf('Cost Distribution Comparison (%d runs each)', nR));
grid on;

%% ===== Figure 4: Best Cost bar chart =====
figure('Position', [50, 50, 850, 500], 'Color', 'w');
bestVals = zeros(1, length(algoNames));
worstVals = zeros(1, length(algoNames));
avgVals = zeros(1, length(algoNames));
for ai = 1:length(algoNames)
    if isempty(allData{ai}), continue; end
    bestVals(ai) = allData{ai}.st.bestCost;
    worstVals(ai) = allData{ai}.st.worstCost;
    avgVals(ai) = allData{ai}.st.avgCost;
end
bar(bestVals, 'FaceColor', 'flat', 'CData', colors); hold on;
for ai = 1:length(algoNames), text(ai, bestVals(ai)+5, sprintf('%.0f', bestVals(ai)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold'); end
set(gca, 'XTickLabel', displayNames, 'XTickLabelRotation', 15);
ylabel('Best Cost'); title('Best Cost by Algorithm');
yline(knownOpt, '--k', 'Optimal (21282)', 'LineWidth', 1.0); grid on; hold off;

%% ===== Figures 5+: Individual Best Cost distributions =====
for ai = 1:length(algoNames)
    data = allData{ai};
    if isempty(data), continue; end
    figure('Position', [50, 50, 600, 450], 'Color', 'w');
    histogram(data.costs, 10, 'FaceColor', colors(ai,:), 'EdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    xline(data.st.bestCost, '--r', sprintf('Best=%.1f', data.st.bestCost), 'LineWidth', 1.5);
    xline(data.st.medianCost, '--', sprintf('Median=%.1f', data.st.medianCost), ...
        'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
    xline(knownOpt, '--k', 'Optimal (21282)', 'LineWidth', 1.0);
    xlabel('Best Cost'); ylabel('Frequency');
    title(sprintf('%s — Cost Distribution (Gap: %.1f%%)', displayNames{ai}, ...
        (data.st.bestCost-knownOpt)/knownOpt*100));
    grid on; hold off;
end

fprintf('\nAll figures ready. (Not auto-saved)\n');
