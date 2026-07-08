function results = runExperiment(expName, mapName, algoFunc, algoNames, varargin)
% runExperiment — 通用实验运行框架
%   results = runExperiment('globalCompare', '杂乱不规则_1', @testAlgo, {'AStar','Dijkstra'}, ...)
%
% 必需参数:
%   expName    — 实验名称（用于文件命名）
%   mapName    — 预设地图名称
%   algoFunc   — 算法测试函数句柄，签名: metrics = algoFunc(map, params)
%   algoNames  — 算法名称 cell 数组
%
% 可选参数 (Name-Value):
%   'NRepeat'     — 每组重复次数（默认 1）
%   'SaveDir'     — 保存根目录（默认自动检测 Experiments/ 下对应子目录）
%   'ExtraParams' — 传递给 algoFunc 的额外参数 struct
%
% 输出:
%   results — 结构体数组，每个算法一个元素
%     .algoName, .mapName, .params, .metrics, .raw, .timestamp

    p = inputParser;
    addParameter(p, 'NRepeat', 1);
    addParameter(p, 'SaveDir', '');
    addParameter(p, 'ExtraParams', struct());
    parse(p, varargin{:});
    opts = p.Results;

    % 加载地图
    [map, ~] = loadPresetMap(mapName);

    % 时间戳
    ts = datestr(now, 'yyyymmdd_HHMMSS');

    % 日志文件
    if isempty(opts.SaveDir)
        baseDir = fileparts(fileparts(mfilename('fullpath')));
        logDir = fullfile(baseDir, 'results');
    else
        logDir = fullfile(opts.SaveDir, 'results');
    end
    if ~exist(logDir, 'dir'), mkdir(logDir); end
    logFile = fullfile(logDir, [expName '_' mapName '_' ts '_log.txt']);
    fid = fopen(logFile, 'w');
    fprintf(fid, '=== 实验日志 ===\n');
    fprintf(fid, '实验名称: %s\n', expName);
    fprintf(fid, '地图: %s\n', mapName);
    fprintf(fid, '时间: %s\n', ts);
    fprintf(fid, '重复次数: %d\n', opts.NRepeat);
    fprintf(fid, '算法列表: %s\n', strjoin(algoNames, ', '));
    fprintf(fid, '\n');

    nAlgo = length(algoNames);
    results = struct('algoName',{}, 'mapName',{}, 'params',{}, 'metrics',{}, 'raw',{}, 'timestamp',{});

    for ai = 1:nAlgo
        algoName = algoNames{ai};
        fprintf(fid, '--- [%d/%d] %s ---\n', ai, nAlgo, algoName);
        fprintf('  [%d/%d] 测试 %s ...\n', ai, nAlgo, algoName);

        allMetrics = [];
        for rep = 1:opts.NRepeat
            params = struct('algoName', algoName, 'mapName', mapName);
            % 合并额外参数
            fn = fieldnames(opts.ExtraParams);
            for k = 1:length(fn)
                params.(fn{k}) = opts.ExtraParams.(fn{k});
            end
            params.algoName = algoName;

            tStart = tic;
            try
                m = algoFunc(map, params);
                m.elapsed = toc(tStart);
            catch ME
                fprintf(fid, '  [ERROR] %s\n', ME.message);
                fprintf('  [ERROR] %s: %s\n', algoName, ME.message);
                m = struct('error', ME.message, 'elapsed', toc(tStart));
            end

            if isempty(allMetrics)
                allMetrics = m;
            else
                allMetrics(end+1) = m; %#ok<AGROW>
            end

            if opts.NRepeat > 1
                fprintf(fid, '  重复 %d: %.3fs\n', rep, m.elapsed);
            end
        end

        % 汇总
        results(end+1).algoName = algoName;
        results(end).mapName = mapName;
        results(end).params = opts.ExtraParams;
        results(end).metrics = allMetrics;
        results(end).raw = struct();
        results(end).timestamp = ts;

        avgTime = mean([allMetrics.elapsed]);
        fprintf(fid, '  平均耗时: %.4fs\n', avgTime);
        if isfield(allMetrics, 'totalCost')
            avgCost = mean([allMetrics.totalCost]);
            fprintf(fid, '  平均代价: %.4f\n', avgCost);
        end
        fprintf(fid, '\n');
    end

    fclose(fid);

    % 保存 MAT
    matFile = fullfile(logDir, [expName '_' mapName '_' ts '.mat']);
    save(matFile, 'results');
    fprintf('  日志: %s\n', logFile);
    fprintf('  数据: %s\n', matFile);
end
