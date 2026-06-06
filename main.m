function main()
%MAIN 全向移动机器人多目标自主导航仿真系统入口
%   启动主界面

% 添加所有子目录到路径
rootPath = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(rootPath, 'Models')));
addpath(genpath(fullfile(rootPath, 'GlobalPlanning')));
addpath(genpath(fullfile(rootPath, 'TSPOptimization')));
addpath(genpath(fullfile(rootPath, 'LocalPlanning')));
addpath(genpath(fullfile(rootPath, 'GUI')));
addpath(genpath(fullfile(rootPath, 'Utils')));

% 启动主界面
MainUI();
end
