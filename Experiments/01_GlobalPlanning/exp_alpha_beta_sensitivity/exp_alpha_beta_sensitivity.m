%% exp_alpha_beta_sensitivity — AStar_v1 自适应启发式 alpha/beta 参数敏感性分析
%  地图: 迷宫.mat
%  alpha: [0.0, 0.2, 0.4, 0.6, 0.8]
%  beta:  [1, 2, 3, 4, 5]
%  alpha=0.0 时等价于标准 A*，只需 1 组实验
%  共 21 组实验

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% 实验配置
mapName = '杂乱不规则';
alphaVals = [0.0, 0.1, 0.2, 0.3, 0.4];
betaVals  = [1, 2, 3, 4];

%% 加载地图（使用预设地图自带的起点终点）
[map, mapData] = loadPresetMap(mapName);
if ~isempty(mapData.startPoint)
    startGrid = mapData.startPoint;
else
    startGrid = [2, 2];
end
if ~isempty(mapData.goalPoint)
    goalGrid = mapData.goalPoint;
else
    goalGrid = [29, 29];
end
fprintf('使用预设地图参数: 起点=[%d,%d], 终点=[%d,%d]\n', startGrid, goalGrid);
% alpha=0.0 只做一组（beta 无影响），alpha>0 做所有 beta 组合
combos = [];
for ai = 1:length(alphaVals)
    a = alphaVals(ai);
    if a == 0
        combos(end+1, :) = [a, 0]; %#ok<SAGROW>  % beta 标记为 0 表示不适用
    else
        for bi = 1:length(betaVals)
            combos(end+1, :) = [a, betaVals(bi)]; %#ok<SAGROW>
        end
    end
end
nCombos = size(combos, 1);

%% 运行实验
fprintf('=== AStar_v1 alpha/beta 参数敏感性分析 ===\n');
fprintf('地图: %s | 起点: [%d,%d] | 终点: [%d,%d]\n', mapName, startGrid, goalGrid);
fprintf('共 %d 组实验\n\n', nCombos);

results = struct('alpha',{}, 'beta',{}, 'expandedNodes',{}, 'pathLength',{}, ...
                 'pathCost',{}, 'openMaxSize',{}, 'elapsed',{});

for ci = 1:nCombos
    a = combos(ci, 1);
    b = combos(ci, 2);

    if a == 0
        label = sprintf('alpha=0.0 (标准A*)');
    else
        label = sprintf('alpha=%.1f, beta=%d', a, b);
    end

    fprintf('[%2d/%d] %-30s ... ', ci, nCombos, label);

    tStart = tic;
    if a == 0
        [path, info] = AStar_v1(map, startGrid, goalGrid, 0, [], 0, 1);
    else
        [path, info] = AStar_v1(map, startGrid, goalGrid, 0, [], a, b);
    end
    elapsed = toc(tStart);

    results(end+1).alpha = a;
    results(end).beta = b;
    results(end).expandedNodes = info.expandedNodes;
    results(end).pathLength = info.pathLength;
    results(end).pathCost = info.pathCost;
    results(end).openMaxSize = info.openMaxSize;
    results(end).elapsed = elapsed;

    fprintf('扩展=%d, 路径长=%d, 代价=%.2f, 耗时=%.4fs\n', ...
        info.expandedNodes, info.pathLength, info.pathCost, elapsed);
end

%% 汇总表格
fprintf('\n=== 结果汇总 ===\n');
fprintf('%-25s %10s %10s %10s %10s %10s\n', '参数组合', '扩展节点', '路径长度', '路径代价', '最大Open', '耗时(s)');
fprintf('%s\n', repmat('-', 1, 80));
for i = 1:length(results)
    r = results(i);
    if r.alpha == 0
        label = 'alpha=0.0 (标准A*)';
    else
        label = sprintf('alpha=%.1f, beta=%d', r.alpha, r.beta);
    end
    fprintf('%-25s %10d %10d %10.2f %10d %10.4f\n', ...
        label, r.expandedNodes, r.pathLength, r.pathCost, r.openMaxSize, r.elapsed);
end

%% 保存结果
ts = datestr(now, 'yyyymmdd_HHMMSS');
saveDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

matFile = fullfile(saveDir, ['alpha_beta_sensitivity_' mapName '_' ts '.mat']);
save(matFile, 'results', 'alphaVals', 'betaVals', 'startGrid', 'goalGrid', 'mapName');

% 写日志
logFile = fullfile(saveDir, ['alpha_beta_sensitivity_' mapName '_' ts '_log.txt']);
fid = fopen(logFile, 'w');
fprintf(fid, '=== AStar_v1 alpha/beta 参数敏感性分析 ===\n');
fprintf(fid, '时间: %s\n\n', ts);

fprintf(fid, '--- 实验简介 ---\n');
fprintf(fid, '实验目的: 分析 AStar_v1 自适应启发式中 alpha（贪心权重）和 beta（衰减速率）\n');
fprintf(fid, '         两个参数对搜索效率和路径质量的影响，确定最优参数区间。\n');
fprintf(fid, '实验方法: 在迷宫地图上固定起终点，遍历 alpha=[0,0.2,0.4,0.6,0.8] x beta=[1,2,3,4,5]\n');
fprintf(fid, '         共 21 组参数组合，alpha=0 等价于标准 A* 作为 baseline。\n');
fprintf(fid, '测试指标: 扩展节点数、搜索耗时、路径长度、路径代价、Open 表最大大小。\n');
fprintf(fid, '预期结论: 自适应启发式应在不牺牲路径质量的前提下显著减少搜索开销。\n\n');

fprintf(fid, '--- 实验配置 ---\n');
fprintf(fid, '地图: %s | 起点: [%d,%d] | 终点: [%d,%d]\n', mapName, startGrid, goalGrid);
fprintf(fid, '%-25s %10s %10s %10s %10s %10s\n', '参数组合', '扩展节点', '路径长度', '路径代价', '最大Open', '耗时(s)');
fprintf(fid, '%s\n', repmat('-', 1, 80));
for i = 1:length(results)
    r = results(i);
    if r.alpha == 0
        label = 'alpha=0.0 (标准A*)';
    else
        label = sprintf('alpha=%.1f, beta=%d', r.alpha, r.beta);
    end
    fprintf(fid, '%-25s %10d %10d %10.2f %10d %10.4f\n', ...
        label, r.expandedNodes, r.pathLength, r.pathCost, r.openMaxSize, r.elapsed);
end
fclose(fid);

fprintf('\n数据已保存: %s\n', matFile);
fprintf('日志已保存: %s\n', logFile);

%% 绘图
figH = figure('Position', [100, 20, 1200, 800]);

% 提取 alpha>0 的数据用于热力图
alphaActive = alphaVals(alphaVals > 0);
nA = length(alphaActive);
nB = length(betaVals);
expMatrix = zeros(nA, nB);
costMatrix = zeros(nA, nB);
timeMatrix = zeros(nA, nB);
for i = 1:length(results)
    r = results(i);
    if r.alpha > 0
        ai = find(alphaActive == r.alpha);
        bi = find(betaVals == r.beta);
        expMatrix(ai, bi) = r.expandedNodes;
        costMatrix(ai, bi) = r.pathCost;
        timeMatrix(ai, bi) = r.elapsed;
    end
end
baselineExp = results(1).expandedNodes;
baselineCost = results(1).pathCost;
baselineTime = results(1).elapsed;

% 自定义热力图配色: 浅蓝(190,220,240) → 浅橙(255,188,168)
cLight = [190, 220, 240] / 255;
cDark  = [255, 188, 168] / 255;
nColors = 256;
customCMap = [linspace(cLight(1), cDark(1), nColors)', ...
              linspace(cLight(2), cDark(2), nColors)', ...
              linspace(cLight(3), cDark(3), nColors)'];

% 子图1: 扩展节点数热力图
subplot(2,2,1);
imagesc(betaVals, alphaActive, expMatrix);
colorbar; colormap(gca, customCMap);
set(gca, 'YDir', 'normal', 'XTick', betaVals, 'YTick', alphaActive);
xlabel('\beta'); ylabel('\alpha');
title(sprintf('扩展节点数 (baseline=%d)', baselineExp));
for ai = 1:nA
    for bi = 1:nB
        text(betaVals(bi), alphaActive(ai), num2str(expMatrix(ai,bi)), ...
            'HorizontalAlignment','center', 'FontSize',8);
    end
end

% 子图2: 路径代价热力图
subplot(2,2,2);
imagesc(betaVals, alphaActive, costMatrix);
colorbar; colormap(gca, customCMap);
set(gca, 'YDir', 'normal', 'XTick', betaVals, 'YTick', alphaActive);
xlabel('\beta'); ylabel('\alpha');
title(sprintf('路径代价 (baseline=%.2f)', baselineCost));
for ai = 1:nA
    for bi = 1:nB
        text(betaVals(bi), alphaActive(ai), sprintf('%.1f', costMatrix(ai,bi)), ...
            'HorizontalAlignment','center', 'FontSize',8);
    end
end

% 子图3: 搜索耗时热力图
subplot(2,2,3);
imagesc(betaVals, alphaActive, timeMatrix * 1000);
colorbar; colormap(gca, customCMap);
set(gca, 'YDir', 'normal', 'XTick', betaVals, 'YTick', alphaActive);
xlabel('\beta'); ylabel('\alpha');
title(sprintf('搜索耗时/ms (baseline=%.1fms)', baselineTime*1000));
for ai = 1:nA
    for bi = 1:nB
        text(betaVals(bi), alphaActive(ai), sprintf('%.1f', timeMatrix(ai,bi)*1000), ...
            'HorizontalAlignment','center', 'FontSize',8);
    end
end

% 子图4: alpha=0.0 baseline 对比折线图
subplot(2,2,4);
lineColors = lines(nA);  % 使用 lines 色板区分不同 alpha
yyaxis left;
for ai = 1:nA
    plot(betaVals, expMatrix(ai,:), '-o', 'LineWidth', 1.5, ...
        'Color', lineColors(ai,:), 'DisplayName', sprintf('\\alpha=%.1f', alphaActive(ai)));
    hold on;
end
yline(baselineExp, '--k', 'LineWidth', 1.5, 'DisplayName', 'baseline (\alpha=0)');
ylabel('扩展节点数');
yyaxis right;
for ai = 1:nA
    plot(betaVals, costMatrix(ai,:), '--s', 'LineWidth', 1.2, ...
        'Color', lineColors(ai,:), 'HandleVisibility', 'off');
end
yline(baselineCost, ':k', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('路径代价');
xlabel('\beta');
title('不同 \alpha 下 \beta 的影响');
legend('Location', 'best');
grid on;

sgtitle(sprintf('AStar_v1 Parameter Sensitivity — %s', mapName), 'FontSize', 14);
