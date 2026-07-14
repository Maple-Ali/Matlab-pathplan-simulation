classdef TempObstacle < handle
    %TEMPOBSTACLE 临时静态障碍物类
    %   不影响全局路径规划，仅在机器人靠近时被检测到
    %   机器人依靠局部规划器（DWA等）实时躲避
    %
    %   与 DynamicObstacle 的区别：
    %     - DynamicObstacle：运动障碍物，有速度，会移动
    %     - TempObstacle：静态障碍物，不移动，但对全局规划器不可见
    %
    %   与普通静态障碍物（Map.grid）的区别：
    %     - 普通静态：写入 Map.grid，A* 能看到并绕行
    %     - 临时静态：不写入 Map.grid，A* 看不到，只有 DWA 在检测范围内能看到

    properties
        position    % [x, y] 连续坐标
        radius      % 等效半径（栅格单位，默认 0.5 = 一个栅格）
        detectionRange  % 检测范围（栅格单位，默认 5.0）
    end

    methods
        function obj = TempObstacle(gridPos, radius, detectionRange)
            %TEMPOBSTACLE 构造函数
            %   gridPos: [row, col] 栅格坐标
            %   radius: 等效半径（可选，默认 0.5）
            %   detectionRange: 检测范围（可选，默认 5.0）
            if nargin < 2 || isempty(radius)
                radius = 0.5;
            end
            if nargin < 3 || isempty(detectionRange)
                detectionRange = 5.0;
            end
            obj.position = [gridPos(2) - 0.5, gridPos(1) - 0.5];  % [x, y]
            obj.radius = radius;
            obj.detectionRange = detectionRange;
        end

        function detected = isDetected(obj, robotPos, detectRange)
            %ISDETECTED 判断是否在机器人检测范围内
            if nargin < 3
                detectRange = obj.detectionRange;
            end
            detected = norm(obj.position - robotPos) <= detectRange;
        end

        function grids = getOccupiedGrids(obj)
            %GETOCCUPIEDGRIDS 返回障碍物占用的栅格坐标 [row, col]
            r = ceil(obj.position(2));
            c = ceil(obj.position(1));
            grids = [r, c];
        end
    end
end
