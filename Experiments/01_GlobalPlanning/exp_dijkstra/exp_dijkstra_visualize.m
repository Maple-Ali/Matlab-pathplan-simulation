%% exp_dijkstra_visualize — Dijkstra_v1 算法搜索过程可视化
%  地图: 杂乱不规则

function exp_dijkstra_visualize()
    clear variables; close all;
    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    addpath(genpath(rootDir));

    [map, mapData] = loadPresetMap('杂乱不规则');
    startGrid = mapData.startPoint;
    goalGrid  = mapData.goalPoint;
    n = map.mapSize;
    occGrid = map.getOccupancyGrid();

    fprintf('地图: 杂乱不规则 | 起点=[%d,%d] | 终点=[%d,%d]\n', startGrid, goalGrid);

    %% 状态容器（嵌套函数可修改）
    snapshot = struct('closedSet', false(n,n), 'openSet', false(n,n), ...
                      'iter', 0, 'expandedCount', 0, 'openMaxSize', 0);

    %% 运行 Dijkstra_v1
    fprintf('运行 Dijkstra_v1 ...\n');
    tStart = tic;
    path = Dijkstra_v1(map, startGrid, goalGrid, 0, @captureCallback);
    elapsed = toc(tStart);

    calcCost = @(p) sum(sqrt(sum(diff(p).^2, 2)));
    pathCost = calcCost(path);

    fprintf('  扩展节点: %d | 路径代价: %.2f | 耗时: %.1fms | 最大Open: %d\n', ...
        snapshot.expandedCount, pathCost, elapsed*1000, snapshot.openMaxSize);

    %% 绘图
    figH = figure('Position', [100, 100, 600, 600], 'Color', 'w');
    ax = axes(figH);
    hold(ax, 'on');

    [obsRows, obsCols] = find(occGrid);
    for i = 1:length(obsRows)
        rectangle(ax, 'Position', [obsCols(i)-1, obsRows(i)-1, 1, 1], ...
            'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 0.3);
    end

    [cRows, cCols] = find(snapshot.closedSet);
    for i = 1:length(cRows)
        rectangle(ax, 'Position', [cCols(i)-1, cRows(i)-1, 1, 1], ...
            'FaceColor', [0.88, 0.88, 0.88], 'EdgeColor', 'none');
    end

    [oRows, oCols] = find(snapshot.openSet);
    for i = 1:length(oRows)
        rectangle(ax, 'Position', [oCols(i)-1, oRows(i)-1, 1, 1], ...
            'FaceColor', [1.0, 0.95, 0.7], 'EdgeColor', 'none');
    end

    if ~isempty(path)
        plot(ax, path(:,2)-0.5, path(:,1)-0.5, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
    end

    plot(ax, startGrid(2)-0.5, startGrid(1)-0.5, 'o', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    plot(ax, goalGrid(2)-0.5, goalGrid(1)-0.5, '^', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    text(ax, startGrid(2)-0.5, startGrid(1)-1.8, 'Start', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    text(ax, goalGrid(2)-0.5, goalGrid(1)+1.0, 'End', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

    xlim(ax, [0, n]); ylim(ax, [0, n]);
    set(ax, 'YDir', 'normal', 'XTick', 0:1:n, 'YTick', 0:1:n);
    grid(ax, 'on');
    set(ax, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    xlabel(ax, 'X'); ylabel(ax, 'Y');
    title(ax, sprintf('Dijkstra_v1 Search Process — Chaotic Irregular\nExpanded: %d | Cost: %.2f | Time: %.1fms | OpenMax: %d', ...
        snapshot.expandedCount, pathCost, elapsed*1000, snapshot.openMaxSize), 'FontSize', 12);

    hExp = patch(ax, NaN, NaN, [0.88, 0.88, 0.88], 'EdgeColor', 'none', 'DisplayName', 'Expanded (closed)');
    hOpen = patch(ax, NaN, NaN, [1.0, 0.95, 0.7], 'EdgeColor', 'none', 'DisplayName', 'Open');
    hPath = plot(ax, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5, 'DisplayName', 'Path');
    legend(ax, [hExp, hOpen, hPath], 'Location', 'southoutside', 'Orientation', 'horizontal');
    hold(ax, 'off');

    %% 嵌套回调（可修改 snapshot）
    function action = captureCallback(stateInfo)
        if strcmp(stateInfo.type, 'step')
            snapshot.closedSet = stateInfo.closedSet;
            snapshot.openSet   = stateInfo.openSet;
            snapshot.iter      = stateInfo.iteration;
            snapshot.expandedCount = stateInfo.iteration;
            nOpen = nnz(stateInfo.openSet);
            if nOpen > snapshot.openMaxSize
                snapshot.openMaxSize = nOpen;
            end
        end
        action = 'continue';
    end
end
