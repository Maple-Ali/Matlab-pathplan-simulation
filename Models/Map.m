classdef Map < handle
    %MAP 栅格地图类
    %   管理静态障碍物、动态障碍物，提供占用栅格查询

    properties
        mapSize     % 正方形边长（栅格数）
        grid        % 矩阵，uint8: 0=自由, 1=静态障碍, 2=动态障碍标记
        dynamicObstacles  % DynamicObstacle 对象数组
        tempObstacles     % TempObstacle 对象数组（临时静态障碍物）
    end

    methods
        function obj = Map(sz)
            obj.mapSize = sz;
            obj.grid = zeros(sz, sz, 'uint8');
            obj.dynamicObstacles = DynamicObstacle.empty();
            obj.tempObstacles = TempObstacle.empty();
        end

        function setStaticObstacle(obj, rows, cols)
            for i = 1:length(rows)
                r = rows(i);
                c = cols(i);
                if obj.inBounds(r, c)
                    obj.grid(r, c) = 1;
                end
            end
        end

        function setDynamicObstacle(obj, obs)
            obj.dynamicObstacles(end + 1) = obs;
        end

        function setTempObstacle(obj, obs)
            %SETTEMPOBSTACLE 添加临时静态障碍物
            obj.tempObstacles(end + 1) = obs;
        end

        function free = isFree(obj, row, col)
            if ~obj.inBounds(row, col)
                free = false;
                return;
            end
            if obj.grid(row, col) == 1
                free = false;
                return;
            end
            for i = 1:length(obj.dynamicObstacles)
                occGrids = obj.dynamicObstacles(i).getOccupiedGrids();
                for g = 1:size(occGrids, 1)
                    if occGrids(g, 1) == row && occGrids(g, 2) == col
                        free = false;
                        return;
                    end
                end
            end
            % 检查临时障碍物（不受检测范围限制，碰撞检测始终生效）
            for i = 1:length(obj.tempObstacles)
                occGrids = obj.tempObstacles(i).getOccupiedGrids();
                for g = 1:size(occGrids, 1)
                    if occGrids(g, 1) == row && occGrids(g, 2) == col
                        free = false;
                        return;
                    end
                end
            end
            free = true;
        end

        function occGrid = getOccupancyGrid(obj)
            %GETOCCUPANCYGRID 返回仅含静态障碍物的占用栅格
            %   全局规划器（A*等）使用此方法，动态障碍物不影响全局路径
            occGrid = double(obj.grid == 1);
        end

        function occGrid = getLocalOccGrid(obj, robotPos, detectRange)
            %GETLOCALOCCGRID 返回含检测范围内动态+临时障碍物的占用栅格
            %   robotPos: [x, y] 机器人连续坐标
            %   detectRange: 检测范围（栅格单位）
            %   局部规划器（DWA等）使用此方法感知障碍物
            occGrid = double(obj.grid == 1);
            % 动态障碍物
            for i = 1:length(obj.dynamicObstacles)
                obs = obj.dynamicObstacles(i);
                if obs.isDetected(robotPos, detectRange)
                    occ = obs.getOccupiedGrids();
                    for g = 1:size(occ, 1)
                        r = occ(g, 1);
                        c = occ(g, 2);
                        if obj.inBounds(r, c)
                            occGrid(r, c) = 1;
                        end
                    end
                end
            end
            % 临时静态障碍物（仅检测范围内可见）
            for i = 1:length(obj.tempObstacles)
                obs = obj.tempObstacles(i);
                if obs.isDetected(robotPos, detectRange)
                    occ = obs.getOccupiedGrids();
                    for g = 1:size(occ, 1)
                        r = occ(g, 1);
                        c = occ(g, 2);
                        if obj.inBounds(r, c)
                            occGrid(r, c) = 1;
                        end
                    end
                end
            end
        end

        function updateDynamicObstacles(obj, dt)
            for i = 1:length(obj.dynamicObstacles)
                obj.dynamicObstacles(i).update(dt);
            end
        end

        function ok = inBounds(obj, row, col)
            ok = row >= 1 && row <= obj.mapSize && col >= 1 && col <= obj.mapSize;
        end
    end
end
