% =====================================================
% 多折线图脚本（支持最多 5 条折线）
% =====================================================
% 使用说明：
%   1. 修改 lineData 元胞数组，每行格式：
%      {'x轴名称', 值1, 值2, 值3, 值4, 值5}
%      每一列对应一条折线，不足 5 条的位置用 [] 留空
%   2. 修改 lineColors 定义每条折线的颜色
%   3. 修改 legendEntries 定义图例名称
%   4. 调整 lineWidth / markerSize 控制线宽和标记大小
% =====================================================

%% ==================== 数据区域（请在此修改） ====================

% 每行格式：{'x轴名称', 值1, 值2, 值3, 值4, 值5}
% 每一列对应一条折线，不需要的折线用 [] 占位
lineData = {
    % '本文A*',    72.93,  108.17,  113.33;
    % '传统A*',    76.53,  111.44,  119.64;
    % 'Dijkstra',  76.53,  111.44,  119.64;
    % 'RRT',      100.80,  135.40,  144.07;

    '本文A*',    2.45,  11.85,  4.31;
    '传统A*',    6.14,  29.19,  17.88;
    'Dijkstra',  24.26,  202.29,  204.46;
    'RRT',      7.42,  19.33,  3.69;
};

% ---- 颜色定义 ----
% 每行对应一条折线的 [R, G, B]，范围 0~1
lineColors = [
    0.47, 0.67, 0.19;   % 折线1 — 绿色
    0.85, 0.33, 0.10;   % 折线2 — 橙色
    0.93, 0.69, 0.13;   % 折线3 — 黄色
    0.00, 0.45, 0.74;   % 折线4 — 蓝色
    0.30, 0.75, 0.93;   % 折线5 — 浅蓝色
];

% ---- 图例名称 ----
legendEntries = {'Map1', 'Map2', 'Map3', 'Map4', 'Map5'};

% ---- 线型与标记 ----
lineWidth  = 2.0;    % 折线宽度
markerSize = 8;      % 数据点标记大小

% ---- Y轴标尺 ----
useLogScale = true;  % true = 对数坐标, false = 线性坐标

% ---- 标记形状（每条折线不同，循环使用） ----
markers = {'o', 's', 'd', '^', 'v'};

%% ==================== 绘图部分（无需修改） ====================

numGroups = size(lineData, 1);
xLabels   = lineData(:, 1)';

% 提取数值矩阵，[] 位置自动变为 NaN
maxLines = 0;
dataMat = [];
for i = 1:numGroups
    row = lineData(i, 2:end);
    vals = [];
    for j = 1:length(row)
        v = row{j};
        if isempty(v)
            vals(end+1) = NaN;
        else
            vals(end+1) = v;
        end
    end
    dataMat = [dataMat; vals]; %#ok<AGROW>
    maxLines = max(maxLines, length(row));
end

% 补齐列数一致
if size(dataMat, 2) < maxLines
    dataMat(:, end+1:maxLines) = NaN;
end

numLines = maxLines;
xPos = 1:numGroups;

% 创建图形
figure('Name', '多折线图', 'NumberTitle', 'off', 'Position', [100 100 900 500]);
hold on;

% 逐条折线绘制
legend_h = gobjects(1, numLines);
for k = 1:numLines
    yData = dataMat(:, k);
    markerIdx = mod(k-1, length(markers)) + 1;
    legend_h(k) = plot(xPos, yData, ...
        'Color', lineColors(k, :), ...
        'LineWidth', lineWidth, ...
        'Marker', markers{markerIdx}, ...
        'MarkerSize', markerSize, ...
        'MarkerFaceColor', lineColors(k, :), ...
        'MarkerEdgeColor', 'w');

    % 数据点数值标注
    for i = 1:numGroups
        if ~isnan(yData(i))
            text(i, yData(i) + max(dataMat(:), [], 'omitnan') * 0.02, ...
                num2str(yData(i), '%.2f'), ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 9, 'Color', 'k');
        end
    end
end

hold off;

% 坐标轴设置
set(gca, 'XTick', xPos, 'XTickLabel', xLabels, 'FontSize', 11);
xlim([0.5, numGroups + 0.5]);
xlabel(' ', 'FontSize', 12);
ylabel('Value', 'FontSize', 12);
title('Comparison', 'FontSize', 14);
grid on;
box on;

% Y轴标尺切换
if useLogScale
    set(gca, 'YScale', 'log');
end

% 图例
legend(legend_h, legendEntries(1:numLines), 'Location', 'best', 'FontSize', 9);
