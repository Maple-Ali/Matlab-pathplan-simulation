%% exp_dwa_performance — DWA_v3 局部规划综合性能测试（含动态动画）
%  地图: 杂乱不规则_动临
%  全局路径: AStar_v1 + SimplifyPath + SmoothPath (红色虚线)
%  实际路径: DWA_v3 导航 (绿色实线)
%  动态障碍物: 灰色栅格实时移动 | 临时障碍物: 浅棕色栅格

clear variables; close all;
rootDir = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
addpath(genpath(rootDir));

%% 加载地图
[map, mapData] = loadPresetMap('杂乱不规则_动临');
n = map.mapSize;
occGrid = map.getOccupancyGrid();
startGrid = mapData.startPoint;
goalGrid  = mapData.goalPoint;

fprintf('地图: 杂乱不规则_动临 | 起点=[%d,%d] | 终点=[%d,%d]\n', startGrid, goalGrid);

%% Step 1: 全局路径规划
fprintf('AStar_v1 全局规划 ...\n');
[gridPath, info] = AStar_v1(map, startGrid, goalGrid, 0, [], 0.3, 3);
fprintf('  扩展节点: %d | 路径点数: %d | 代价: %.2f\n', info.expandedNodes, size(gridPath,1), info.pathCost);

fprintf('SimplifyPath + SmoothPath ...\n');
simPath = SimplifyPath(gridPath, occGrid, n, 0.3);
smoothPath = SmoothPath(simPath, 15);
fprintf('  简化后: %d pts | 平滑后: %d pts\n', size(simPath,1), size(smoothPath,1));

%% Step 2: 初始化动态障碍物和临时障碍物
if isfield(mapData, 'dynamicObstacleDefs') && ~isempty(mapData.dynamicObstacleDefs)
    defs = mapData.dynamicObstacleDefs;
    for di = 1:size(defs, 1)
        obs = DynamicObstacle(defs(di, 1:2), defs(di, 3:4), defs(di, 5));
        map.setDynamicObstacle(obs);
        fprintf('  动态障碍物 %d: [%.1f,%.1f]→[%.1f,%.1f], speed=%.2f\n', ...
            di, defs(di,1), defs(di,2), defs(di,3), defs(di,4), defs(di,5));
    end
end
if isfield(mapData, 'tempObstacleDefs') && ~isempty(mapData.tempObstacleDefs)
    for ti = 1:size(mapData.tempObstacleDefs, 1)
        % 字段: [row, col, detectionRange]，半径固定 0.5（1个栅格）
        map.setTempObstacle(TempObstacle(mapData.tempObstacleDefs(ti, 1:2), 0.5, mapData.tempObstacleDefs(ti, 3)));
    end
    fprintf('  临时障碍物: %d 个 (1格宽)\n', size(mapData.tempObstacleDefs,1));
end

% 预计算临时障碍物占据的栅格（每个占1格）
tempGrids = false(n, n);
if isfield(mapData, 'tempObstacleDefs') && ~isempty(mapData.tempObstacleDefs)
    for ti = 1:size(mapData.tempObstacleDefs, 1)
        r0 = mapData.tempObstacleDefs(ti, 1);
        c0 = mapData.tempObstacleDefs(ti, 2);
        if r0>=1 && r0<=n && c0>=1 && c0<=n
            tempGrids(r0, c0) = true;
        end
    end
end

% 轨迹记录
nDynObs = length(map.dynamicObstacles);
dynObsHistory = cell(1, nDynObs);
for di = 1:nDynObs
    dynObsHistory{di} = map.dynamicObstacles(di).currentPos;
end

% 机器人
startPosCont = [startGrid(2)-0.5, startGrid(1)-0.5];
goalPos = [goalGrid(2)-0.5, goalGrid(1)-0.5];
robot = OmnidirectionalRobot(startPosCont, 1.0, 0.2);

%% Step 3: 设置动画图 =====
fig1 = figure('Position', [50, 50, 750, 700], 'Color', 'w');
ax1 = axes(fig1);
hold(ax1, 'on');

% --- 静态层 ---
% 静态障碍物（纯黑）
[obsRows, obsCols] = find(occGrid);
for i = 1:length(obsRows)
    rectangle(ax1, 'Position', [obsCols(i)-1, obsRows(i)-1, 1, 1], ...
        'FaceColor', [0.1, 0.1, 0.1], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 0.3);
end

% 临时障碍物（浅棕色栅格）
[tempRows, tempCols] = find(tempGrids);
for i = 1:length(tempRows)
    rectangle(ax1, 'Position', [tempCols(i)-1, tempRows(i)-1, 1, 1], ...
        'FaceColor', [0.85, 0.75, 0.60], 'EdgeColor', 'none');
end

% 全局路径（红色虚线）
plot(ax1, smoothPath(:,1), smoothPath(:,2), '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0);

% 起点终点
plot(ax1, startPosCont(1), startPosCont(2), 'o', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(ax1, goalPos(1), goalPos(2), '^', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.9, 0.2, 0.2], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
text(ax1, startPosCont(1), startPosCont(2)-1.8, 'Start', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
text(ax1, goalPos(1), goalPos(2)+1.0, 'End', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');

xlim(ax1, [0, n]); ylim(ax1, [0, n]);
set(ax1, 'YDir', 'normal', 'XTick', 0:1:n, 'YTick', 0:1:n);
grid(ax1, 'on'); set(ax1, 'GridLineStyle', '-', 'GridAlpha', 0.15);
xlabel(ax1, 'X'); ylabel(ax1, 'Y');

% --- 动态层（初始） ---
% 机器人轨迹
hTraj = plot(ax1, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
hRobot = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.2, 0.8, 0.2], 'MarkerEdgeColor', 'k');

% 动态障碍物（连续坐标圆）
hDynObs = gobjects(nDynObs, 1);
for di = 1:nDynObs
    hDynObs(di) = rectangle(ax1, 'Position', [0, 0, 1, 1], 'Curvature', [1, 1], ...
        'FaceColor', [0.6, 0.6, 0.6], 'EdgeColor', [0.4, 0.4, 0.4], 'LineWidth', 0.5);
end

% 动态障碍物轨迹线
hDynTraj = gobjects(nDynObs, 1);
for di = 1:nDynObs
    hDynTraj(di) = plot(ax1, NaN, NaN, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.8);
end

% 图例
hGlob = plot(ax1, NaN, NaN, '--', 'Color', [0.9, 0.2, 0.2], 'LineWidth', 1.0, 'DisplayName', 'Ref Path');
hReal = plot(ax1, NaN, NaN, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5, 'DisplayName', 'DWA Actual');
hDynL = plot(ax1, NaN, NaN, '--', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 0.8, 'DisplayName', 'DynObs Traj');
hTempL = patch(ax1, NaN, NaN, [0.85, 0.75, 0.60], 'EdgeColor', 'none', 'DisplayName', 'Temp Obstacle');
legend(ax1, [hGlob, hReal, hDynL, hTempL], 'Location', 'southoutside', 'Orientation', 'horizontal');

%% Step 4: DWA_v3 导航循环（含动画） =====
dt = 0.05; maxTime = 120; lookAheadDist = 0.5;
robotTraj = startPosCont;
velHistory = [0, 0];
timeHistory = [0];
speedHistory = [0];
prevIdx = 1;
simTime = 0; step = 0;
reachedGoal = false;
drawInterval = 5;  % 每5步刷新一次动画

fprintf('DWA_v3 导航 (动画) ...\n');
tic;

while simTime < maxTime && ~reachedGoal
    step = step + 1;

    % 更新动态障碍物
    map.updateDynamicObstacles(dt);

    % 记录障碍物轨迹
    for di = 1:nDynObs
        dynObsHistory{di}(end+1, :) = map.dynamicObstacles(di).currentPos;
    end

    % 获取前瞻点
    [lookAheadPt, prevIdx] = findLookAheadExt(robot.pos, smoothPath, lookAheadDist, prevIdx);
    distToGoal = norm(robot.pos - goalPos);
    if distToGoal < lookAheadDist + 0.5
        lookAheadPt = goalPos;
    end

    % DWA_v3
    [vx, vy, ~] = DWAPlanner_v3(robot, lookAheadPt, map, [], dt, struct());

    % 机器人执行
    robot.applyVelocity(vx, vy, dt);
    simTime = simTime + dt;
    robotTraj(end+1, :) = robot.pos; %#ok<AGROW>
    velHistory(end+1, :) = [vx, vy]; %#ok<AGROW>
    timeHistory(end+1) = simTime; %#ok<AGROW>
    speedHistory(end+1) = norm([vx, vy]); %#ok<AGROW>

    %% 动画更新（每 drawInterval 步）
    if mod(step, drawInterval) == 0
        % 更新机器人轨迹
        set(hTraj, 'XData', robotTraj(:,1), 'YData', robotTraj(:,2));
        set(hRobot, 'XData', robot.pos(1), 'YData', robot.pos(2));

        % 更新动态障碍物位置（连续坐标，半径0.5的圆）
        for di = 1:nDynObs
            pos = map.dynamicObstacles(di).currentPos;
            set(hDynObs(di), 'Position', [pos(1)-0.5, pos(2)-0.5, 1, 1]);
            % 更新轨迹线
            hist = dynObsHistory{di};
            set(hDynTraj(di), 'XData', hist(:,1), 'YData', hist(:,2));
        end

        title(ax1, sprintf('DWA_v3 Navigation — Step: %d | Time: %.1fs | Speed: %.2f m/s', ...
            step, simTime, speedHistory(end)), 'FontSize', 12);
        drawnow;
    end

    if distToGoal < 0.25
        reachedGoal = true;
    end
end
simElapsed = toc;

% 最终状态更新
set(hTraj, 'XData', robotTraj(:,1), 'YData', robotTraj(:,2));
set(hRobot, 'XData', robot.pos(1), 'YData', robot.pos(2));
for di = 1:nDynObs
    pos = map.dynamicObstacles(di).currentPos;
    set(hDynObs(di), 'Position', [pos(1)-0.5, pos(2)-0.5, 1, 1]);
    hist = dynObsHistory{di};
    set(hDynTraj(di), 'XData', hist(:,1), 'YData', hist(:,2));
end
title(ax1, sprintf('DWA_v3 Navigation — Steps: %d, Time: %.1fs, Final Dist: %.3f', ...
    step, simTime, distToGoal), 'FontSize', 12);
drawnow;

hold(ax1, 'off');

fprintf('  仿真步数: %d | 时间: %.1fs | 壁钟耗时: %.2fs\n', step, simTime, simElapsed);
fprintf('  到达终点: %s | 最终距离: %.3f\n', string(reachedGoal), distToGoal);

%% ===== 图2: 速度曲线 =====
fig2 = figure('Position', [820, 50, 600, 700], 'Color', 'w');

subplot(3,1,1);
plot(timeHistory, velHistory(:,1), '-', 'Color', [0.2, 0.5, 0.9], 'LineWidth', 1.2);
hold on;
plot(timeHistory, velHistory(:,2), '-', 'Color', [0.9, 0.4, 0.2], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Components');
legend('v_x', 'v_y', 'Location', 'best'); grid on;

subplot(3,1,2);
plot(timeHistory, speedHistory, '-', 'Color', [0.2, 0.7, 0.2], 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed Magnitude');
yline(1.0, '--r', 'maxSpeed', 'LineWidth', 1.0); grid on;

subplot(3,1,3);
heading = atan2d(velHistory(:,2), velHistory(:,1));
plot(timeHistory, heading, '-', 'Color', [0.6, 0.3, 0.8], 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Velocity Heading'); grid on;

sgtitle('DWA\_v3 Robot Velocity Profile', 'FontSize', 13);

%% ===== 局部函数 =====
function [pt, newIdx] = findLookAheadExt(robotPos, refPath, lookAheadDist, prevIdx, searchEnd)
    if nargin < 5, searchEnd = size(refPath, 1); end
    if size(refPath, 1) < 2, pt = robotPos; newIdx = 1; return; end
    startIdx = min(prevIdx, searchEnd);
    minDist = inf; closestIdx = startIdx;
    for i = startIdx:searchEnd
        d = norm(refPath(i, :) - robotPos);
        if d < minDist, minDist = d; closestIdx = i; end
    end
    for i = closestIdx:searchEnd
        if norm(refPath(i, :) - robotPos) >= lookAheadDist
            pt = refPath(i, :); newIdx = closestIdx; return;
        end
    end
    pt = refPath(searchEnd, :); newIdx = closestIdx;
end