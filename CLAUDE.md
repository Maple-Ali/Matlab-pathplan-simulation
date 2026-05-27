# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

全向移动机器人多目标自主导航仿真系统。MATLAB R2025b，2D 栅格地图，支持单机器人和多机器人协同（最多 5 台），包含全局路径规划（A*/Dijkstra/RRT）、TSP 多目标排序、多机器人任务分配（最近邻聚类）、局部动态避障（DWA/TEB/MPC），以及完整 GUI。

## 启动与测试

```matlab
% 启动 GUI
main

% 单机器人命令行测试
simParams = struct('mapSize', 30, 'staticObstacles', [10,10], ...
    'startPoint', [2,2], 'targetPoints', [15,15; 20,20], 'goalPoint', [29,29], ...
    'globalAlgo', 'AStar', 'localAlgo', 'DWA', ...
    'enableSimplify', false, 'enableSmooth', true, ...
    'stepDelay', 0, 'robotMaxSpeed', 1.0, 'robotRadius', 0.2);
results = SimulationManager(simParams);

% 多机器人命令行测试
simParams = struct('mapSize', 30, 'staticObstacles', [], ...
    'startPoints', [2,2; 28,2], 'targetPoints', [8,8; 15,15; 22,8], ...
    'goalPoints', [29,5; 29,25], ...
    'globalAlgo', 'AStar', 'localAlgo', 'DWA', ...
    'enableSimplify', false, 'enableSmooth', true, ...
    'stepDelay', 0, 'robotMaxSpeed', 1.0, 'robotRadius', 0.2);
results = SimulationManager(simParams);

% 单元测试——每个算法文件底部可直接粘贴到命令行运行
```

MCP 工具（需要 MATLAB 会话已启动）：
- `mcp__matlab__evaluate_matlab_code` — 执行 MATLAB 代码片段
- `mcp__matlab__run_matlab_file` — 运行 `.m` 脚本文件
- `mcp__matlab__check_matlab_code` — 静态代码分析

## 架构

分层模块化：**表现层(GUI)** → **控制层(SimulationManager)** → **算法层** → **模型层**

### 坐标系统与转换约定

**连续坐标全部使用 `[x, y]`**（x = 列方向在前）。栅格 `(row, col)` 映射为连续坐标 `(col - 0.5, row - 0.5)`。

**GUI 层坐标约定**：`MainUI` 的 `state` 中各点以 UI 坐标 `(x, y) = (col, row)` 存储，在 `onStart` 中统一转换为算法层 `[row, col]`。修改 GUI 时注意此边界。

| 数据 | 坐标格式 | 示例 |
|------|---------|------|
| 绘图用的路径/位置 | `[x, y]` 连续 | `[2.5, 3.5]` |
| 栅格算法用的点 | `[row, col]` | `[4, 3]` |
| `DynamicObstacle.currentPos` | `[x, y]` 连续 | — |
| `OmnidirectionalRobot.pos` / `trajectory` | `[x, y]` 连续 | — |
| `MainUI` 内部 `state.*` | `(x, y) = (col, row)` UI 坐标 | — |

### 关键接口

| 模块 | 输入 | 输出 |
|------|------|------|
| 全局规划器 `AStar/Dijkstra/RRT` | `(map, startGrid, goalGrid, delay)` | `N×2 [row, col]` 路径 |
| 局部规划器 `DWA/TEB/MPC` | `(robot, localGoal, map, ~, dt, params)` | `(vx, vy, predictTraj)` |
| `TSPsolver` | `(start, targets, goal, map, algoName)` | `(orderedPoints, segPaths, totalCost)` |
| `MultiRobotTaskAllocation` | `(startPoints, targets, goalPoints, map, algoName)` | `robotTasks(1:N)` 结构体数组 |
| `checkCollision` | `(robotPos, robotRadius, map)` | `bool` |
| `checkRobotRobotCollision` | `(robots cell array)` | `bool`（`SimulationManager` 内局部函数） |
| `SmoothPath` | `(path, density)` 输入栅格路径 | `M×2 [x, y]` 连续平滑路径 |
| `plotTools('multiRobotColor', idx)` | 机器人索引 (1-based) | `[r, g, b]` 区分色 |

### 仿真主循环 (SimulationManager)

1. 初始化 Map → 创建 N 个 Robot → 任务分配（多机器人 `MultiRobotTaskAllocation` / 单机器人 `TSPsolver`）→ 全局规划 → 路径后处理（simplify → smooth）
2. 执行循环：逐机器人局部规划 → `robot.applyVelocity(vx, vy, dt)` → `map.updateDynamicObstacles(dt)` → 环境碰撞 + 机器人间碰撞检测 → 可视化更新
3. 每机器人独立状态机：到达目标点后停留 1 秒，然后切换下一目标
4. 接近目标时（`distToGoal < lookAheadDist + 0.5`），前瞻点直接使用目标点而非路径上的前瞻点
5. `findLookAheadExt` 为非持久变量版本（支持多机器人），通过 `prevIdx` 参数传递状态

### 多机器人任务分配

`MultiRobotTaskAllocation` 实现**最近邻聚类 + 各机器人独立 TSP**：
1. 每个目标点按欧氏距离分配给最近的机器人起点
2. 每台机器人对其分配到的目标点子集独立调用 `TSPsolver`
3. 未分配到目标的机器人直接规划 start→goal 路径
4. 各机器人可使用不同终点（`goalPoints` N×2）

### TSP 求解策略

- 目标点 ≤5：全排列枚举
- 目标点 >5：遗传算法（OX 交叉 + 锦标赛选择）
- 成本矩阵通过逐对调用全局规划器计算

### SimParams 字段一览

| 字段 | 维度 | 说明 |
|------|------|------|
| `startPoint` | 1×2 `[row, col]` | 单机器人起点（向后兼容） |
| `startPoints` | N×2 `[row, col]` | 多机器人起点 |
| `targetPoints` | K×2 `[row, col]` | 目标点 |
| `goalPoint` | 1×2 `[row, col]` | 单机器人终点（向后兼容） |
| `goalPoints` | N×2 `[row, col]` | 多机器人各自终点 |
| `globalAlgo` / `localAlgo` | string | 规划器选择 |
| `robotIdx` | int | MPC 局部规划器用，标识机器人编号 |

## 注意事项

- **Map 属性名为 `mapSize`**，不是 `size`（避免与 MATLAB 内置 `size()` 冲突）
- **避免在代码中使用 `clear all`** 然后立即 `addpath`——可能清掉已加载的 MCP 会话。用 `clear variables` 或重新 `addpath(genpath(pwd))` 即可
- **拐角裁剪(SimplifyPath)可与路径平滑(SmoothPath)组合使用**：SmoothPath 会自动检测稀疏输入路径并插入中间栅格点防止样条曲线切弯穿障
- **DWAPlanner 使用引力+斥力模型**（非传统 DWA 的 v/w 速度采样），接近目标自动减速，边界和障碍物产生斥力
- **TEB/MPC 为简化框架**，内部用势场法实现，标注了可替换为 `fmincon`/CasADi 的位置
- **MPC 使用 `containers.Map` 管理持久状态**：以 `robotIdx` 为键存储 `prevVelMap`，保证多机器人时状态隔离。调用时需传入 `params.robotIdx`
- **GUI 使用 `uifigure` 编程创建**（`.m` 文件），非 App Designer 的 `.mlapp` 二进制格式，便于版本控制
- **`DynamicObstacle` 对象数组初始化**：必须用 `DynamicObstacle.empty()` 而非 `[]`，否则 MATLAB 无法在数组中追加对象
- **`plotTools('getColor', name)`** 获取统一颜色方案，所有可视化使用此接口保证颜色一致
- **`plotTools('setupAxes', ax, mapSize)`** 设置栅格地图坐标轴（原点在左下角，x 向右，y 向上）
- **`plotTools('multiRobotColor', idx)`** 获取多机器人区分颜色（青、橙、紫、绿、棕，超出则循环）
- **多机器人时 simParams 优先用 `startPoints`/`goalPoints`**：SimulationManager 检测到这些字段非空时优先使用，否则 fallback 到 `startPoint`/`goalPoint` 单机器人字段
