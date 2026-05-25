# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

全向移动机器人多目标自主导航仿真系统。MATLAB R2025b，2D 栅格地图，包含全局路径规划（A*/Dijkstra/RRT）、TSP 多目标排序、局部动态避障（DWA/TEB/MPC），以及完整 GUI。

## 启动与测试

```matlab
% 启动 GUI
main

% 命令行直接测试仿真（跳过 GUI）
simParams = struct('mapSize', 30, 'staticObstacles', [10,10], ...
    'startPoint', [2,2], 'targetPoints', [15,15], 'goalPoint', [29,29], ...
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

### 坐标系统

**连续坐标全部使用 `[x, y]`**（x = 列方向在前）。栅格 `(row, col)` 映射为连续坐标 `(col - 0.5, row - 0.5)`。

- 输出给绘图函数的路径/位置：`[x, y]`
- 输出给栅格算法的点：`[row, col]`
- `DynamicObstacle.currentPos`：`[x, y]` 连续坐标
- `OmnidirectionalRobot.pos`：`[x, y]` 连续坐标，`trajectory` 同理

### 关键接口

| 模块 | 输入 | 输出 |
|------|------|------|
| 全局规划器 `AStar/Dijkstra/RRT` | `(map, startGrid, goalGrid, delay)` | `N×2 [row, col]` 路径 |
| 局部规划器 `DWAPlanner` | `(robot, localGoal, map, ~, dt, params)` | `(vx, vy, predictTraj)` |
| TSP 求解器 | `(start, targets, goal, map, algoName)` | `(orderedPoints, segPaths, totalCost)` |
| 碰撞检测 | `(robotPos, robotRadius, map)` | `bool` |
| `SmoothPath` | `(path, density)` 输入栅格路径 | `M×2 [x, y]` 连续平滑路径 |

### 仿真主循环 (SimulationManager)

1. 初始化 Map → Robot → TSP 排序 → 全局规划 → 路径后处理
2. 执行循环：局部规划 → `robot.applyVelocity(vx, vy, dt)` → `map.updateDynamicObstacles(dt)` → 碰撞检测 → 可视化
3. 到达目标点后停留 1 秒，然后切换下一目标
4. 接近目标时（`distToGoal < lookAheadDist + 0.5`），`lookAheadPt` 直接使用目标点而非路径上的前瞻点，防止机器人越过目标

### TSP 求解策略

- 目标点 ≤5：全排列枚举
- 目标点 >5：遗传算法（OX 交叉 + 锦标赛选择）
- 成本矩阵通过逐对调用全局规划器计算

## 注意事项

- **Map 属性名为 `mapSize`**，不是 `size`（避免与 MATLAB 内置 `size()` 冲突）
- **避免在代码中使用 `clear all`** 然后立即 `addpath`——可能清掉已加载的 MCP 会话。用 `clear variables` 或重新 `addpath(genpath(pwd))` 即可
- **拐角裁剪(SimplifyPath)可与路径平滑(SmoothPath)组合使用**：SmoothPath 会自动检测稀疏输入路径并插入中间栅格点防止样条曲线切弯穿障
- **DWAPlanner 使用引力+斥力模型**（非传统 DWA 的 v/w 速度采样），接近目标自动减速，边界和障碍物产生斥力
- **TEB/MPC 为简化框架**，内部用势场法实现，标注了可替换为 `fmincon`/CasADi 的位置
- **GUI 使用 `uifigure` 编程创建**（`.m` 文件），非 App Designer 的 `.mlapp` 二进制格式，便于版本控制
- **`DynamicObstacle` 对象数组初始化**：必须用 `DynamicObstacle.empty()` 而非 `[]`，否则 MATLAB 无法在数组中追加对象
- **`plotTools('getColor', name)`** 获取统一颜色方案，所有可视化使用此接口保证颜色一致
- **`plotTools('setupAxes', ax, mapSize)`** 设置栅格地图坐标轴（原点在左下角，x 向右，y 向上）
