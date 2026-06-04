classdef Map < handle
    %MAP 栅格地图类
    %   管理静态障碍物、动态障碍物，提供占用栅格查询

    properties
        mapSize     % 正方形边长（栅格数）
        grid        % 矩阵，uint8: 0=自由, 1=静态障碍, 2=动态障碍标记
        dynamicObstacles  % DynamicObstacle 对象数组
    end

    methods
        function obj = Map(sz)
            obj.mapSize = sz;
            obj.grid = zeros(sz, sz, 'uint8');
            obj.dynamicObstacles = DynamicObstacle.empty();
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
            free = true;
        end

        function occGrid = getOccupancyGrid(obj)
            occGrid = double(obj.grid == 1);
            for i = 1:length(obj.dynamicObstacles)
                occ = obj.dynamicObstacles(i).getOccupiedGrids();
                for g = 1:size(occ, 1)
                    r = occ(g, 1);
                    c = occ(g, 2);
                    if obj.inBounds(r, c)
                        occGrid(r, c) = 1;
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
