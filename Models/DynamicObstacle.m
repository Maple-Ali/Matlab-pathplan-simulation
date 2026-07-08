classdef DynamicObstacle < handle
    %DYNAMICOBSTACLE 动态障碍物类
    %   沿两点之间线段往复匀速移动

    properties
        startPos    % 起始栅格 [row, col]
        endPos      % 终点栅格 [row, col]
        speed       % 移动速度（栅格/秒）
        currentPos  % 当前位置 [x, y]（连续坐标）
        direction   % 1 或 -1
        velocity    % [vx, vy] 速度向量（连续坐标，由 update 自动更新）
        posHistory  % N×3 历史记录 [x, y, t]（最近 10 帧）
        detectionRange  % 检测范围（栅格单位，默认 5.0）
    end

    properties (Access = private)
        startCont   % 起始连续坐标 [x, y]
        endCont     % 终点连续坐标 [x, y]
        elapsedTime % 累计时间（秒）
    end

    methods
        function obj = DynamicObstacle(startPos, endPos, speed)
            obj.startPos = startPos;
            obj.endPos = endPos;
            obj.speed = speed;
            obj.direction = 1;
            obj.velocity = [0, 0];
            obj.posHistory = zeros(0, 3);
            obj.detectionRange = 5.0;
            obj.elapsedTime = 0;
            % 栅格 (r,c) → 连续 (c-0.5, r-0.5)
            obj.startCont = [startPos(2) - 0.5, startPos(1) - 0.5];
            obj.endCont = [endPos(2) - 0.5, endPos(1) - 0.5];
            obj.currentPos = obj.startCont;
        end

        function update(obj, dt)
            oldPos = obj.currentPos;

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

            % 更新速度向量（从位置差分计算，处理反弹方向）
            if dt > 0
                obj.velocity = (obj.currentPos - oldPos) / dt;
            end

            % 记录位置历史（保留最近 10 帧）
            obj.elapsedTime = obj.elapsedTime + dt;
            obj.posHistory = [obj.posHistory; obj.currentPos, obj.elapsedTime];
            if size(obj.posHistory, 1) > 10
                obj.posHistory = obj.posHistory(end-9:end, :);
            end
        end

        function grids = getOccupiedGrids(obj)
            % 连续坐标 [x, y] → 栅格 (r, c): r=ceil(y), c=ceil(x)
            r = ceil(obj.currentPos(2));
            c = ceil(obj.currentPos(1));
            grids = [r, c];
        end

        function vel = getVelocity(obj)
            %GETVELOCITY 返回当前速度向量 [vx, vy]
            vel = obj.velocity;
        end

        function detected = isDetected(obj, robotPos, detectRange)
            %ISDETECTED 判断是否在机器人检测范围内
            %   robotPos: [x, y] 机器人连续坐标
            %   detectRange: 检测范围（栅格单位），若省略则用 obj.detectionRange
            if nargin < 3
                detectRange = obj.detectionRange;
            end
            detected = norm(obj.currentPos - robotPos) <= detectRange;
        end

        function futurePos = predictPosition(obj, t)
            %PREDICTPOSITION 根据当前速度预测未来位置（直线预测）
            %   t: 预测时间（秒）
            %   注意：不考虑反弹，仅做短期线性外推
            futurePos = obj.currentPos + obj.velocity * t;
        end
    end
end
