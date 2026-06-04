function varargout = plotTools(action, varargin)
%PLOTTOOLS 统一绘图辅助函数
%   color = plotTools('getColor', name)  获取颜色
%   plotTools('setupAxes', ax, mapSize)  设置坐标轴

persistent colors

if isempty(colors)
    colors = struct(...
        'globalPath',    [1.0, 0.4, 0.4], ...  % 淡红色 - 原始全局路径
        'simplifiedPath', [0.0, 0.0, 1.0], ...  % 蓝色 - 简化后路径
        'smoothPath',    [0.0, 0.7, 0.0], ...  % 绿色 - 平滑后路径
        'actualTraj',    [0.2, 0.2, 0.2], ...  % 深灰 - 实际轨迹
        'dynamicObs',    [1.0, 0.0, 1.0], ...  % 品红 - 动态障碍物
        'staticObs',     [0.0, 0.0, 0.0], ...  % 黑色 - 静态障碍物
        'start',         [0.0, 0.8, 0.0], ...  % 绿色 - 起点
        'target',        [0.0, 0.0, 1.0], ...  % 蓝色 - 目标点
        'goal',          [1.0, 0.0, 0.0], ...  % 红色 - 终点
        'robot',         [0.0, 0.5, 0.5], ...  % 青色 - 机器人
        'explored',      [0.8, 0.8, 0.8], ...  % 浅灰 - 探索区域
        'predictTraj',   [1.0, 0.0, 0.0], ...  % 红色 - 预测轨迹
        'openSet',       [0.6, 0.95, 0.6], ... % 浅绿 - 搜索中 open set
        'closedSet',     [0.75, 0.75, 0.75], ... % 中灰 - 已扩展 closed set
        'currentNode',   [1.0, 0.5, 0.0], ...  % 橙色 - 当前扩展节点
        'finalPath',     [0.0, 0.3, 1.0]);     % 亮蓝 - 最终规划路径
end

switch action
    case 'getColor'
        name = varargin{1};
        if isfield(colors, name)
            varargout{1} = colors.(name);
        else
            varargout{1} = [0, 0, 0];
        end
    case 'getAllColors'
        varargout{1} = colors;
    case 'multiRobotColor'
        idx = varargin{1};
        palette = [0, 0.6, 0.6;  1, 0.5, 0;  0.6, 0, 0.6;  0, 0.7, 0;  0.7, 0.2, 0];
        varargout{1} = palette(mod(idx - 1, size(palette, 1)) + 1, :);
    case 'setupAxes'
        ax = varargin{1};
        mapSize = varargin{2};
        cla(ax);
        hold(ax, 'on');
        axis(ax, 'equal');
        axis(ax, [0, mapSize, 0, mapSize]);
        grid(ax, 'on');
        set(ax, 'XTick', 0:1:mapSize, 'YTick', 0:1:mapSize);
        set(ax, 'GridAlpha', 0.3);
        xlabel(ax, 'X (列)');
        ylabel(ax, 'Y (行)');
        title(ax, '移动机器人导航仿真');
    otherwise
        error('未知操作: %s', action);
end
end
