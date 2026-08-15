% =====================================================
% 自定义柱状图脚本（名称、数值、颜色集中定义）
% 使用说明：修改 barData 元胞数组和 barWidth 的值即可
% =====================================================

%% ----- 可自定义的数据区域（请在此修改） -----
% 每一行格式：{'名称', 数值, [R, G, B]}
% R,G,B 范围 0~1，可自由增减行数
barData = {
    % 'Algo 1',   3.195,   [0.47, 0.67, 0.19];   % green
    % 'Algo 2',   4.226,    [0.85, 0.33, 0.10];   % orange
    % 'Algo 3',   6.931,   [0.93, 0.69, 0.13];   % yellow
    % 'Algo 4',   6.101,    [0.00, 0.45, 0.74];   % blue
    % 'Algo 5',   4.156,   [0.30, 0.75, 0.93];   % cyan
    % 'Algo 6',   2.152,   [0.49, 0.18, 0.56];   % purple
%    '名称7',   90,    [0.64, 0.08, 0.18];   % maroon

    'MMAS-VND-CL',   3.195,   [0.47, 0.67, 0.19];   % green
    'Standard ACO',   1.037,    [0.85, 0.33, 0.10];   % orange
    'GA',   8.414,   [0.93, 0.69, 0.13];   % yellow
    'SA',   2.149,    [0.00, 0.45, 0.74];   % blue
    'Improved GA',   8.705,   [0.30, 0.75, 0.93];   % cyan
    'Improved SA',   2.000,   [0.49, 0.18, 0.56];   % purple
%    '名称7',   90,    [0.64, 0.08, 0.18];   % maroon

};

% 柱子宽度（0~1，默认0.8，越小柱子越细间距越大）
barWidth = 0.6;

%% ----- 绘图部分（无需修改） -----
% 提取数据
numBars = size(barData, 1);
xLabels = barData(:, 1)';          % 名称
yValues = cell2mat(barData(:, 2)); % 数值
barColors = cell2mat(barData(:, 3)); % 颜色矩阵（N×3）

% 创建图形
figure('Name', '自定义柱状图', 'NumberTitle', 'off');
b = bar(yValues, barWidth, 'FaceColor', 'flat');
b.CData = barColors;

% 设置X轴标签
set(gca, 'XTickLabel', xLabels, 'FontSize', 11);

% 坐标轴标签和标题
xlabel(' ', 'FontSize', 12);
ylabel('Time (s)', 'FontSize', 12);
title('Algorithm running time', 'FontSize', 14);
grid on;
box on;

% 在柱顶标注数值
for i = 1:numBars
    text(i, yValues(i) + max(yValues)*0.02, num2str(yValues(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', 'k');
end