% =====================================================
% 分组柱状图脚本（支持每项最多 5 个柱子）
% =====================================================
% 使用说明：
%   1. 修改 barData 元胞数组，每行格式：
%      {'名称', 值1, 值2, 值3, 值4, 值5}
%      不足 5 个柱子的位置用 [] 留空即可
%   2. 修改 barColors 定义每个柱位的颜色
%   3. 调整 barWidth / groupGap 控制柱宽和组间距
% =====================================================

%% ==================== 数据区域（请在此修改） ====================

% 每行格式：{'x轴名称', 值1, 值2, 值3, 值4, 值5}
% 不需要的柱位用 [] 占位
barData = {
    '本文A*',  72.93,  108.17,  113.33;
    '传统A*',  76.53,  111.44,  119.64;
    'Dijkstra',  76.53,  111.44,  119.64;
    'RRT',     100.80,  135.40,  144.07;

};



% ---- 颜色定义 ----
% 每行对应一个柱位的 [R, G, B]，范围 0~1
% 同一列（同一柱位）在所有组中使用相同颜色
barColors = [
    0.47, 0.67, 0.19;   % 柱位1 — 绿色
    0.85, 0.33, 0.10;   % 柱位2 — 橙色
    0.93, 0.69, 0.13;   % 柱位3 — 黄色
    0.00, 0.45, 0.74;   % 柱位4 — 蓝色
    0.30, 0.75, 0.93;   % 柱位5 — 浅蓝色
];

legendEntries = {'Map1', 'Map2', 'Map3', 'Map2', 'Map3'};

% ---- 柱宽与间距 ----
barWidth  = 0.15;   % 每根柱子的宽度（0~1，越大柱子越粗）
groupGap  = 0.4;    % 组内柱子之间的间距比例（0 为紧贴，越大间距越大）

%% ==================== 绘图部分（无需修改） ====================

numGroups = size(barData, 1);
xLabels   = barData(:, 1)';

% 提取数值矩阵，[] 位置自动变为 NaN
maxBars = 0;
dataMat = [];
for i = 1:numGroups
    row = barData(i, 2:end);
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
    maxBars = max(maxBars, length(row));
end

% 补齐列数一致
if size(dataMat, 2) < maxBars
    dataMat(:, end+1:maxBars) = NaN;
end

% 计算每组宽度和柱位偏移（保证各组居中于 x = 1,2,3,...）
numBars = maxBars;
groupW  = numBars * barWidth + (numBars - 1) * groupGap * barWidth;
offsets = -groupW/2 + barWidth/2 + (0:numBars-1) * (barWidth + groupGap * barWidth);

% 创建图形
figure('Name', '分组柱状图', 'NumberTitle', 'off', 'Position', [100 100 900 500]);
hold on;

% 逐根柱子绘制
for k = 1:numBars
    for i = 1:numGroups
        val = dataMat(i, k);
        if ~isnan(val)
            x = i + offsets(k);
            bar_h = bar(x, val, barWidth * 0.9);
            bar_h.FaceColor = 'flat';
            bar_h.CData = barColors(k, :);
            bar_h.EdgeColor = 'w';
            bar_h.LineWidth = 0.5;

            % 柱顶数值标注
            text(x, val + max(dataMat(:), [], 'omitnan') * 0.02, ...
                num2str(val, '%.3f'), ...
                'HorizontalAlignment', 'center', ...
                'FontSize', 9, 'Color', 'k');
        end
    end
end

hold off;

% 坐标轴设置
set(gca, 'XTick', 1:numGroups, 'XTickLabel', xLabels, 'FontSize', 11);
xlim([0.4, numGroups + 0.6]);
xlabel(' ', 'FontSize', 12);
ylabel('Time (s)', 'FontSize', 12);
title('Algorithm Comparison', 'FontSize', 14);
grid on;
box on;

% 图例（自动识别实际使用的柱位）

legend_h = gobjects(1, numBars);
hold on;
for k = 1:numBars
    legend_h(k) = bar(nan, nan, 'FaceColor', 'flat');
    legend_h(k).CData = barColors(k, :);
end
hold off;
legend(legend_h, legendEntries(1:numBars), 'Location', 'best', 'FontSize', 9);
