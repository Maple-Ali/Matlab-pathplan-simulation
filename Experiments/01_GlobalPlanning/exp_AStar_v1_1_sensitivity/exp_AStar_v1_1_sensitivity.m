%% exp_AStar_v1_1_sensitivity — AStar_v1_1 障碍物距离自适应权重 alpha/d_ref 参数敏感性分析
%  地图: Map1, Map2_kong, 迷宫
%  alpha: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8] — 最大额外权重
%  d_ref: [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15] — 障碍物距离阈值(对角距离百分比)
%  共 3 地图 x 8 x 7 = 168 组实验
%
%  输出:
%    results/ — .mat 数据文件 + .txt 日志
%    热力图仅显示，不自动保存

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% ======================== 参数配置（在此修改） ========================
mapNames = {'Map1', 'Map2_kong', '迷宫'};   % 地图列表

alphaVals = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8];  % alpha 参数范围
d_refVals = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15];  % d_ref 参数范围

nRepeats = 1;   % 每组参数重复次数（取平均，减少随机波动）
% =======================================================================

nAlpha = length(alphaVals);
nDref  = length(d_refVals);
nMaps  = length(mapNames);
totalRuns = nMaps * nAlpha * nDref * nRepeats;

fprintf('=== AStar_v1_1 alpha/d_ref 参数敏感性分析 ===\n');
fprintf('地图: %s\n', strjoin(mapNames, ', '));
fprintf('alpha: [%.1f, ..., %.1f] (%d 级)\n', alphaVals(1), alphaVals(end), nAlpha);
fprintf('d_ref: [%.2f, ..., %.2f] (%d 级)\n', d_refVals(1), d_refVals(end), nDref);
fprintf('重复: %d 次 | 总实验数: %d\n\n', nRepeats, totalRuns);

%% ======================== 加载地图 ========================
maps = cell(1, nMaps);
mapInfos = cell(1, nMaps);
for mi = 1:nMaps
    [maps{mi}, mapInfos{mi}] = loadPresetMap(mapNames{mi});
    fprintf('已加载地图 [%d/%d]: %s (size=%d, obstacles=%d)\n', ...
        mi, nMaps, mapNames{mi}, mapInfos{mi}.mapSize, ...
        size(mapInfos{mi}.staticObstacles, 1));
end

%% ======================== 运行实验 ========================
% 为每个地图预分配结果矩阵
% expandedNodes(alpha_idx, d_ref_idx)
% pathCost(alpha_idx, d_ref_idx)
% elapsed(alpha_idx, d_ref_idx)
allResults = cell(1, nMaps);
for mi = 1:nMaps
    allResults{mi} = struct(...
        'expandedNodes', zeros(nAlpha, nDref), ...
        'pathCost',      zeros(nAlpha, nDref), ...
        'elapsed',       zeros(nAlpha, nDref), ...
        'pathLength',    zeros(nAlpha, nDref), ...
        'openMaxSize',   zeros(nAlpha, nDref));
end

runCount = 0;
tTotal = tic;

for mi = 1:nMaps
    map = maps{mi};
    mapData = mapInfos{mi};
    startGrid = mapData.startPoint;
    goalGrid  = mapData.goalPoint;

    fprintf('\n--- 地图: %s | 起点: [%d,%d] | 终点: [%d,%d] ---\n', ...
        mapNames{mi}, startGrid, goalGrid);

    for ai = 1:nAlpha
        for di = 1:nDref
            alpha = alphaVals(ai);
            d_ref = d_refVals(di);

            % 多次重复取平均
            expAccum = 0; costAccum = 0; timeAccum = 0;
            lenAccum = 0; openAccum = 0;

            for rep = 1:nRepeats
                runCount = runCount + 1;
                tStart = tic;
                [path, info] = AStar_v1_1(map, startGrid, goalGrid, 0, [], alpha, d_ref);
                elapsed = toc(tStart);

                expAccum  = expAccum  + info.expandedNodes;
                costAccum = costAccum + info.pathCost;
                timeAccum = timeAccum + elapsed;
                lenAccum  = lenAccum  + info.pathLength;
                openAccum = openAccum + info.openMaxSize;
            end

            allResults{mi}.expandedNodes(ai, di) = expAccum / nRepeats;
            allResults{mi}.pathCost(ai, di)      = costAccum / nRepeats;
            allResults{mi}.elapsed(ai, di)       = timeAccum / nRepeats;
            allResults{mi}.pathLength(ai, di)    = lenAccum / nRepeats;
            allResults{mi}.openMaxSize(ai, di)   = openAccum / nRepeats;

            if mod(runCount, 20) == 0 || runCount == totalRuns
                fprintf('  进度: [%d/%d] (%.1f%%)\n', runCount, totalRuns, 100*runCount/totalRuns);
            end
        end
    end
end

tElapsed = toc(tTotal);
fprintf('\n全部完成，总耗时: %.1f 秒\n', tElapsed);

%% ======================== 汇总输出 ========================
fprintf('\n=== 结果汇总 ===\n');
for mi = 1:nMaps
    fprintf('\n--- %s ---\n', mapNames{mi});
    fprintf('%8s %8s | %10s %10s %10s\n', 'alpha', 'd_ref', '扩展节点', '路径代价', '耗时(ms)');
    fprintf('%s\n', repmat('-', 1, 55));
    for ai = 1:nAlpha
        for di = 1:nDref
            fprintf('%8.1f %8.2f | %10.0f %10.2f %10.1f\n', ...
                alphaVals(ai), d_refVals(di), ...
                allResults{mi}.expandedNodes(ai, di), ...
                allResults{mi}.pathCost(ai, di), ...
                allResults{mi}.elapsed(ai, di) * 1000);
        end
    end
end

%% ======================== 保存数据 ========================
ts = datestr(now, 'yyyymmdd_HHMMSS');
saveDir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(saveDir, 'dir'), mkdir(saveDir); end

matFile = fullfile(saveDir, ['AStar_v1_1_sensitivity_' ts '.mat']);
save(matFile, 'allResults', 'mapNames', 'alphaVals', 'd_refVals', 'nRepeats');

% 写日志
logFile = fullfile(saveDir, ['AStar_v1_1_sensitivity_' ts '_log.txt']);
fid = fopen(logFile, 'w');
fprintf(fid, '=== AStar_v1_1 alpha/d_ref 参数敏感性分析 ===\n');
fprintf(fid, '时间: %s\n', ts);
fprintf(fid, '总耗时: %.1f 秒\n\n', tElapsed);

fprintf(fid, '--- 实验配置 ---\n');
fprintf(fid, '地图: %s\n', strjoin(mapNames, ', '));
fprintf(fid, 'alpha: [%.1f, ..., %.1f] (%d 级)\n', alphaVals(1), alphaVals(end), nAlpha);
fprintf(fid, 'd_ref: [%.2f, ..., %.2f] (%d 级)\n', d_refVals(1), d_refVals(end), nDref);
fprintf(fid, '重复: %d 次\n\n', nRepeats);

fprintf(fid, '--- 参数说明 ---\n');
fprintf(fid, 'alpha: 最大额外权重 [0.1, 0.8]\n');
fprintf(fid, '       越大 → 开阔区域越贪心（路径更短但可能绕远）\n');
fprintf(fid, '       越小 → 越接近标准 A*（更稳定但搜索更慢）\n');
fprintf(fid, 'd_ref: 障碍物距离阈值(占地图对角距离百分比) [0.02, 0.15]\n');
fprintf(fid, '       越小 → 障碍物影响范围越窄（只有紧贴障碍物才减速）\n');
fprintf(fid, '       越大 → 障碍物影响范围越广（提前减速，路径更安全）\n\n');

for mi = 1:nMaps
    fprintf(fid, '--- %s ---\n', mapNames{mi});
    fprintf(fid, '地图大小: %d, 起点: [%d,%d], 终点: [%d,%d], 障碍物: %d\n', ...
        mapInfos{mi}.mapSize, mapInfos{mi}.startPoint, mapInfos{mi}.goalPoint, ...
        size(mapInfos{mi}.staticObstacles, 1));
    fprintf(fid, '%8s %8s | %10s %10s %10s %10s %10s\n', ...
        'alpha', 'd_ref', '扩展节点', '路径代价', '路径长度', '最大Open', '耗时(ms)');
    fprintf(fid, '%s\n', repmat('-', 1, 70));
    for ai = 1:nAlpha
        for di = 1:nDref
            fprintf(fid, '%8.1f %8.2f | %10.0f %10.2f %10.0f %10.0f %10.1f\n', ...
                alphaVals(ai), d_refVals(di), ...
                allResults{mi}.expandedNodes(ai, di), ...
                allResults{mi}.pathCost(ai, di), ...
                allResults{mi}.pathLength(ai, di), ...
                allResults{mi}.openMaxSize(ai, di), ...
                allResults{mi}.elapsed(ai, di) * 1000);
        end
    end
    fprintf(fid, '\n');
end
fclose(fid);

fprintf('\n数据已保存: %s\n', matFile);
fprintf('日志已保存: %s\n', logFile);

%% ======================== 绘制热力图（仅显示，不自动保存） ========================
% 自定义热力图配色: 浅蓝(190,220,240) → 浅橙(255,188,168)
cLight = [190, 220, 240] / 255;
cDark  = [255, 188, 168] / 255;
nColors = 256;
customCMap = [linspace(cLight(1), cDark(1), nColors)', ...
              linspace(cLight(2), cDark(2), nColors)', ...
              linspace(cLight(3), cDark(3), nColors)'];

for mi = 1:nMaps
    R = allResults{mi};
    mapName = mapNames{mi};

    % --- 图1: 扩展节点数热力图 ---
    figure('Position', [100, 100, 700, 500]);
    imagesc(d_refVals, alphaVals, R.expandedNodes);
    colorbar; colormap(gca, customCMap);
    set(gca, 'YDir', 'normal', 'XTick', d_refVals, 'YTick', alphaVals);
    xlabel('d_{ref}'); ylabel('\alpha');
    title(sprintf('扩展节点数 — %s', mapName));
    % 标注数值
    for ai = 1:nAlpha
        for di = 1:nDref
            text(d_refVals(di), alphaVals(ai), sprintf('%.0f', R.expandedNodes(ai,di)), ...
                'HorizontalAlignment', 'center', 'FontSize', 7);
        end
    end

    % --- 图2: 路径代价热力图 ---
    figure('Position', [150, 150, 700, 500]);
    imagesc(d_refVals, alphaVals, R.pathCost);
    colorbar; colormap(gca, customCMap);
    set(gca, 'YDir', 'normal', 'XTick', d_refVals, 'YTick', alphaVals);
    xlabel('d_{ref}'); ylabel('\alpha');
    title(sprintf('路径代价 — %s', mapName));
    for ai = 1:nAlpha
        for di = 1:nDref
            text(d_refVals(di), alphaVals(ai), sprintf('%.1f', R.pathCost(ai,di)), ...
                'HorizontalAlignment', 'center', 'FontSize', 7);
        end
    end

    % --- 图3: 搜索耗时热力图 ---
    figure('Position', [200, 200, 700, 500]);
    imagesc(d_refVals, alphaVals, R.elapsed * 1000);  % 转换为 ms
    colorbar; colormap(gca, customCMap);
    set(gca, 'YDir', 'normal', 'XTick', d_refVals, 'YTick', alphaVals);
    xlabel('d_{ref}'); ylabel('\alpha');
    title(sprintf('搜索耗时/ms — %s', mapName));
    for ai = 1:nAlpha
        for di = 1:nDref
            text(d_refVals(di), alphaVals(ai), sprintf('%.1f', R.elapsed(ai,di)*1000), ...
                'HorizontalAlignment', 'center', 'FontSize', 7);
        end
    end

    fprintf('已绘制 %s 的热力图（不自动保存）\n', mapName);
end

fprintf('\n=== 实验完成 ===\n');
