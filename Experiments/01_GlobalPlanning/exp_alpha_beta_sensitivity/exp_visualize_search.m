%% exp_visualize_search — 绘制 AStar_v1 搜索过程可视化
%  图1: alpha=0.0 (标准A*)
%  图2: alpha=0.3, beta=0.3

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(rootDir));

%% 加载地图
[map, mapData] = loadPresetMap('杂乱不规则');
startGrid = mapData.startPoint;  % [3,3]
goalGrid  = mapData.goalPoint;   % [28,28]
n = map.mapSize;
occGrid = map.getOccupancyGrid();

%% 运行两种参数的 AStar_v1，记录搜索状态
configs = {
    struct('alpha', 0.0, 'beta', 0.1, 'label', '\alpha=0.0, \beta=0.1 (Standard A*)');
    struct('alpha', 0.3, 'beta', 3.0, 'label', '\alpha=0.3, \beta=3');
};

for ci = 1:length(configs)
    cfg = configs{ci};

    % 运行算法，收集 closedSet 和 openSet 快照
    [path, info, closedSnap, openSnap] = runAStarWithSnapshot(map, startGrid, goalGrid, cfg.alpha, cfg.beta);

    % 绘图
    figH = figure('Position', [100, 100, 600, 600], 'Color', 'w');
    ax = axes(figH);
    hold(ax, 'on');

    % 1. 绘制障碍物（深灰）
    [obsRows, obsCols] = find(occGrid);
    for i = 1:length(obsRows)
        rectangle(ax, 'Position', [obsCols(i)-1, obsRows(i)-1, 1, 1], ...
            'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 0.3);
    end

    % 2. 绘制扩展节点（浅灰）
    [cRows, cCols] = find(closedSnap);
    for i = 1:length(cRows)
        rectangle(ax, 'Position', [cCols(i)-1, cRows(i)-1, 1, 1], ...
            'FaceColor', [0.88, 0.88, 0.88], 'EdgeColor', 'none');
    end

    % 3. 绘制 open 节点（浅黄）
    [oRows, oCols] = find(openSnap);
    for i = 1:length(oRows)
        rectangle(ax, 'Position', [oCols(i)-1, oRows(i)-1, 1, 1], ...
            'FaceColor', [1.0, 0.95, 0.7], 'EdgeColor', 'none');
    end

    % 4. 绘制路径（绿色细实线）— 坐标偏移0.5到栅格中央
    if ~isempty(path)
        pathX = path(:, 2) - 0.5;  % col → x, 栅格中央
        pathY = path(:, 1) - 0.5;  % row → y, 栅格中央
        plot(ax, pathX, pathY, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
    end

    % 5. 标记起点（绿色圆）和终点（红色三角）— 栅格中央
    plot(ax, startGrid(2)-0.5, startGrid(1)-0.5, 'o', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    plot(ax, goalGrid(2)-0.5, goalGrid(1)-0.5, '^', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

    % 标注起点终点文字
    text(ax, startGrid(2)-0.5, startGrid(1)-1.8, 'Start', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
    text(ax, goalGrid(2)-0.5, goalGrid(1)+1.0, 'End', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

    % 坐标轴设置
    xlim(ax, [0, n]);
    ylim(ax, [0, n]);
    set(ax, 'YDir', 'normal');
    grid(ax, 'on');
    set(ax, 'GridLineStyle', '-', 'GridAlpha', 0.15);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    set(ax, 'XTick', 0:1:n, 'YTick', 0:1:n);
    title(ax, sprintf('AStar_v1 Search Process — %s\nExpanded: %d | Cost: %.2f | Time: %.1fms', ...
        cfg.label, info.expandedNodes, info.pathCost, info.elapsed*1000), 'FontSize', 12);

    % 图例（用透明占位点避免 dummy patch 残留）
    hExp = patch(ax, NaN, NaN, [0.88, 0.88, 0.88], 'EdgeColor', 'none', 'DisplayName', 'Expanded (closed)');
    hOpen = patch(ax, NaN, NaN, [1.0, 0.95, 0.7], 'EdgeColor', 'none', 'DisplayName', 'Open');
    hPath = plot(ax, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5, 'DisplayName', 'Path');
    legend(ax, [hExp, hOpen, hPath], 'Location', 'southoutside', 'Orientation', 'horizontal');

    hold(ax, 'off');
end

%% 局部函数：运行 AStar_v1 并记录搜索状态
function [path, info, closedSnap, openSnap] = runAStarWithSnapshot(map, startGrid, goalGrid, alpha, beta)
    n = map.mapSize;
    occGrid = map.getOccupancyGrid();

    if alpha == 0
        alpha = 0.001;  % 避免除零，效果近似标准A*
    end

    % 启发式
    startDist = sqrt((startGrid(1)-goalGrid(1))^2 + (startGrid(2)-goalGrid(2))^2);
    if startDist == 0, startDist = 1; end
    h = @(r,c) sqrt((r-goalGrid(1))^2 + (c-goalGrid(2))^2);
    hWeight = @(r,c) 1 + alpha * exp(-beta*(1 - h(r,c)/startDist));
    weightedH = @(r,c) hWeight(r,c) * h(r,c);

    % 初始化
    gScore = inf(n,n); gScore(startGrid(1),startGrid(2)) = 0;
    fScore = inf(n,n); fScore(startGrid(1),startGrid(2)) = weightedH(startGrid(1),startGrid(2));
    hScore = inf(n,n); hScore(startGrid(1),startGrid(2)) = h(startGrid(1),startGrid(2));
    parent = zeros(n,n,2);
    closedSet = false(n,n);

    % 二叉堆
    heapSize = 0;
    heap = zeros(n*n, 4);
    heapPos = zeros(n,n);
    heapSize = heapSize + 1;
    heap(heapSize,:) = [fScore(startGrid(1),startGrid(2)), h(startGrid(1),startGrid(2)), startGrid(1), startGrid(2)];
    heapPos(startGrid(1),startGrid(2)) = heapSize;

    dRow = [-1,-1,-1,0,0,1,1,1];
    dCol = [-1,0,1,-1,1,-1,0,1];
    moveCost = [sqrt(2),1,sqrt(2),1,1,sqrt(2),1,sqrt(2)];

    tStart = tic;
    expandedCount = 0;

    while heapSize > 0
        topRow = heap(1,3); topCol = heap(1,4);
        heapPos(topRow,topCol) = 0;
        heap(1,:) = heap(heapSize,:);
        if heapSize > 1
            heapPos(heap(heapSize,3),heap(heapSize,4)) = 1;
        end
        heapSize = heapSize - 1;
        if heapSize > 0, bubbleDown(1); end

        if topRow == goalGrid(1) && topCol == goalGrid(2)
            path = reconstructPath(parent, startGrid, goalGrid);
            info = struct('expandedNodes', expandedCount+1, 'pathLength', size(path,1), ...
                'pathCost', gScore(goalGrid(1),goalGrid(2)), 'elapsed', toc(tStart));
            closedSnap = closedSet;
            % openSet = heap 中的节点
            openSnap = false(n,n);
            for i = 1:heapSize
                openSnap(heap(i,3), heap(i,4)) = true;
            end
            return;
        end

        closedSet(topRow,topCol) = true;
        expandedCount = expandedCount + 1;

        for d = 1:8
            nr = topRow + dRow(d); nc = topCol + dCol(d);
            if nr<1||nr>n||nc<1||nc>n, continue; end
            if closedSet(nr,nc)||occGrid(nr,nc), continue; end
            if dRow(d)~=0 && dCol(d)~=0
                if occGrid(topRow+dRow(d),topCol)||occGrid(topRow,topCol+dCol(d)), continue; end
            end
            tentG = gScore(topRow,topCol) + moveCost(d);
            if tentG < gScore(nr,nc)
                gScore(nr,nc) = tentG;
                newH = h(nr,nc);
                newF = tentG + weightedH(nr,nc);
                fScore(nr,nc) = newF;
                hScore(nr,nc) = newH;
                parent(nr,nc,:) = [topRow,topCol];
                pos = heapPos(nr,nc);
                if pos == 0
                    heapSize = heapSize + 1;
                    heap(heapSize,:) = [newF,newH,nr,nc];
                    heapPos(nr,nc) = heapSize;
                    bubbleUp(heapSize);
                else
                    heap(pos,:) = [newF,newH,nr,nc];
                    bubbleUp(pos);
                end
            end
        end
    end

    path = [];
    info = struct('expandedNodes',expandedCount,'pathLength',0,'pathCost',inf,'elapsed',toc(tStart));
    closedSnap = closedSet;
    openSnap = false(n,n);

    function bubbleUp(idx)
        while idx > 1
            pi = floor(idx/2);
            if heap(idx,1) < heap(pi,1) || (heap(idx,1)==heap(pi,1) && heap(idx,2)<heap(pi,2))
                tmp = heap(idx,:); heap(idx,:) = heap(pi,:); heap(pi,:) = tmp;
                ri=heap(idx,3);ci=heap(idx,4);rj=heap(pi,3);cj=heap(pi,4);
                heapPos(ri,ci)=idx; heapPos(rj,cj)=pi;
                idx = pi;
            else, break; end
        end
    end

    function bubbleDown(idx)
        while true
            sm = idx; L = 2*idx; R = 2*idx+1;
            if L<=heapSize && (heap(L,1)<heap(sm,1)||(heap(L,1)==heap(sm,1)&&heap(L,2)<heap(sm,2))), sm=L; end
            if R<=heapSize && (heap(R,1)<heap(sm,1)||(heap(R,1)==heap(sm,1)&&heap(R,2)<heap(sm,2))), sm=R; end
            if sm~=idx
                tmp=heap(idx,:); heap(idx,:)=heap(sm,:); heap(sm,:)=tmp;
                ri=heap(idx,3);ci=heap(idx,4);rj=heap(sm,3);cj=heap(sm,4);
                heapPos(ri,ci)=idx; heapPos(rj,cj)=sm;
                idx = sm;
            else, break; end
        end
    end
end

function path = reconstructPath(parent, startGrid, goalGrid)
    path = goalGrid;
    current = goalGrid;
    while ~(current(1)==startGrid(1) && current(2)==startGrid(2))
        current = squeeze(parent(current(1),current(2),:))';
        if all(current==0), path=[]; return; end
        path = [current; path];
    end
end
