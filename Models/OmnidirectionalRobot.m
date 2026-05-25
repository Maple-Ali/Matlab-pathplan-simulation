classdef OmnidirectionalRobot < handle
    %OMNIDIRECTIONALROBOT 全向移动机器人类

    properties
        pos         % [x, y] 连续坐标
        vel         % [vx, vy] 当前速度
        maxSpeed    % 最大线速度
        maxAccel    % 最大加速度（预留）
        radius      % 机器人半径
        trajectory  % N×2 实际轨迹记录 [x, y]
    end

    methods
        function obj = OmnidirectionalRobot(startPos, maxSpeed, radius)
            obj.pos = startPos;        % [x, y]
            obj.vel = [0, 0];
            obj.maxSpeed = maxSpeed;
            obj.maxAccel = inf;         % 暂不限制
            obj.radius = radius;
            obj.trajectory = startPos;  % 记录起点
        end

        function applyVelocity(obj, vx, vy, dt)
            obj.vel = [vx, vy];
            obj.pos = obj.pos + obj.vel * dt;
            obj.recordTrajectory();
        end

        function state = getState(obj)
            state = struct('pos', obj.pos, 'vel', obj.vel);
        end

        function recordTrajectory(obj)
            obj.trajectory(end + 1, :) = obj.pos;
        end
    end
end
