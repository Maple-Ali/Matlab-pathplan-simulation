function [map, mapData] = loadPresetMap(mapName)
% loadPresetMap — 加载预设地图并返回 Map 对象
%   [map, mapData] = loadPresetMap('杂乱不规则_1')
%
% 输出:
%   map     — Map 对象（已设置静态障碍物）
%   mapData — 解包后的地图数据结构体（含 mapSize, startPoint, goalPoint 等）

    mapFile = fullfile(fileparts(fileparts(mfilename('fullpath'))), '..', 'PresetMaps', [mapName '.mat']);
    % 确保项目根目录在路径上（Map 类等）
    rootDir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    if ~isempty(rootDir) && exist(rootDir, 'dir')
        addpath(genpath(rootDir));
    end
    md = load(mapFile);
    % 兼容不同 mat 文件的字段名
    if isfield(md, 'mapData')
        mapData = md.mapData;
    else
        mapData = md;
    end
    map = Map(mapData.mapSize);
    if isfield(mapData, 'staticObstacles') && ~isempty(mapData.staticObstacles)
        map.setStaticObstacle(mapData.staticObstacles(:,1), mapData.staticObstacles(:,2));
    end
end
