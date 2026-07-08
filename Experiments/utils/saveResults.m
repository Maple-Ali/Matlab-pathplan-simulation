function saveResults(expName, mapName, params, metrics, raw, figH)
% saveResults — 标准化实验结果保存
%   saveResults('globalCompare', '杂乱不规则_1', params, metrics, raw, figH)
%
% 保存内容:
%   1. results/ 目录下的 .mat 文件（params + metrics + raw）
%   2. results/ 目录下的 _log.txt 文件（可读摘要）
%   3. figures/ 目录下的 .png 图片（如果 figH 非空）

    % 确定保存目录（从调用栈推断实验子目录）
    stack = dbstack('-completenames');
    if length(stack) >= 2
        callerDir = fileparts(stack(2).file);
    else
        callerDir = pwd;
    end
    resultsDir = fullfile(callerDir, 'results');
    figuresDir = fullfile(callerDir, 'figures');
    if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
    if ~exist(figuresDir, 'dir'), mkdir(figuresDir); end

    ts = datestr(now, 'yyyymmdd_HHMMSS');
    baseName = [expName '_' mapName '_' ts];

    % 1. 保存 MAT
    matFile = fullfile(resultsDir, [baseName '.mat']);
    save(matFile, 'params', 'metrics', 'raw');

    % 2. 写日志
    logFile = fullfile(resultsDir, [baseName '_log.txt']);
    fid = fopen(logFile, 'w');
    fprintf(fid, '=== %s | %s ===\n', expName, mapName);
    fprintf(fid, '时间: %s\n\n', ts);

    % 参数
    fprintf(fid, '[参数]\n');
    fn = fieldnames(params);
    for i = 1:length(fn)
        v = params.(fn{i});
        if isnumeric(v)
            fprintf(fid, '  %s = %s\n', fn{i}, mat2str(v));
        elseif ischar(v)
            fprintf(fid, '  %s = %s\n', fn{i}, v);
        elseif islogical(v)
            fprintf(fid, '  %s = %d\n', fn{i}, v);
        end
    end

    % 指标
    fprintf(fid, '\n[指标]\n');
    if isstruct(metrics)
        fn = fieldnames(metrics);
        for i = 1:length(fn)
            v = metrics.(fn{i});
            if isnumeric(v) && isscalar(v)
                fprintf(fid, '  %s = %.6f\n', fn{i}, v);
            elseif isnumeric(v)
                fprintf(fid, '  %s = [%s]\n', fn{i}, mat2str(v));
            end
        end
    end

    fclose(fid);

    % 3. 保存图片
    if nargin >= 6 && ~isempty(figH) && ishandle(figH)
        figFile = fullfile(figuresDir, [baseName '.png']);
        saveas(figH, figFile);
        fprintf('  图片: %s\n', figFile);
    end

    fprintf('  数据: %s\n', matFile);
    fprintf('  日志: %s\n', logFile);
end
