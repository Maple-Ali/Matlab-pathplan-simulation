classdef DynamicObstacle < handle
    %DYNAMICOBSTACLE 动态障碍物类
    %   沿两点之间线段往复匀速移动

    properties
        startPos    % 起始栅格 [row, col]
        endPos      % 终点栅格 [row, col]
        speed       % 移动速度（栅格/秒）
        currentPos  % 当前位置 [x, y]（连续坐标）
        direction   % 1 或 -1
    end

    properties (Access = private)
        startCont   % 起始连续坐标 [x, y]
        endCont     % 终点连续坐标 [x, y]
    end

    methods
        function obj = DynamicObstacle(startPos, endPos, speed)
            obj.startPos = startPos;
            obj.endPos = endPos;
            obj.speed = speed;
            obj.direction = 1;
            % 栅格 (r,c) → 连续 (c-0.5, r-0.5)
            obj.startCont = [startPos(2) - 0.5, startPos(1) - 0.5];
            obj.endCont = [endPos(2) - 0.5, endPos(1) - 0.5];
            obj.currentPos = obj.startCont;
        end

        function update(obj, dt)
            dirVec = obj.endCont - obj.startCont;
            segLen = norm(dirVec);
            if segLen < 1e-6
                return;
            end
            unitDir = dirVec / segLen;
            delta = obj.speed * dt * obj.direction;
            obj.currentPos = obj.currentPos + unitDir * delta;

            % 检查是否到达端点
            t = dot(obj.currentPos - obj.startCont, unitDir);
            if t > segLen
                obj.currentPos = obj.endCont;
                obj.direction = -1;
            elseif t < 0
                obj.currentPos = obj.startCont;
                obj.direction = 1;
            end
        end

        function grids = getOccupiedGrids(obj)
            % 连续坐标 [x, y] → 栅格 (r, c): r=ceil(y), c=ceil(x)
            r = ceil(obj.currentPos(2));
            c = ceil(obj.currentPos(1));
            grids = [r, c];
        end
    end
end
