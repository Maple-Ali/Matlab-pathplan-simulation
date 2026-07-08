# 实验目录

## 目录结构

每个大类下为独立的实验子目录，避免不同实验文件混淆：

```
Experiments/
├── 01_GlobalPlanning/
│   ├── exp_alpha_beta_sensitivity/     ← 每个实验一个独立文件夹
│   │   ├── exp_alpha_beta_sensitivity.m
│   │   ├── results/
│   │   └── figures/
│   ├── exp_astar_variants/             ← 未来新增实验
│   │   ├── ...
├── 02_PathSimplification/
├── 03_TSP/
├── 04_TaskAllocation/
├── 05_LocalPlanning/
├── 06_FullSystem/
└── utils/
```

## 实验类别

| 目录 | 实验内容 | 对应模块 |
|------|---------|---------|
| `01_GlobalPlanning/` | A*/Dijkstra/RRT 及 A* 各变体对比 | `GlobalPlanning/` |
| `02_PathSimplification/` | 拐角裁剪 + 路径平滑效果对比 | `SimplifyPath`, `SmoothPath` |
| `03_TSP/` | 全排列/GA/ACO/SA 各 TSP 算法对比 | `TSPOptimization/` |
| `04_TaskAllocation/` | KMeans/KMedoids/DV 聚类算法对比 | `ClusteringOptimization/` |
| `05_LocalPlanning/` | DWA/TEB/MPC 局部规划对比 | `LocalPlanning/` |
| `06_FullSystem/` | 单/多机器人端到端全流程测试 | `SimulationManager` |

## 通用工具 (`utils/`)

| 函数 | 用途 |
|------|------|
| `loadPresetMap(name)` | 加载预设地图，返回 Map 对象 |
| `runExperiment(expName, mapName, func, names, ...)` | 通用实验运行框架（自动计时、日志、保存） |
| `saveResults(expName, mapName, params, metrics, raw, figH)` | 标准化保存（MAT + 日志 + 图片） |

## 数据保存规范

每个实验独立子目录下保存三类文件：
- `results/*.mat` — 完整数据（params, metrics, raw）
- `results/*_log.txt` — 可读日志摘要（含实验目的、方法、配置）
- `figures/*.png` — 可视化图片

文件命名：`{实验名}_{地图名}_{YYYYMMDD_HHMMSS}`

## 预设地图

| 地图名 | 特点 |
|--------|------|
| `杂乱不规则` | 随机散布障碍物 |
| `杂乱不规则_1` | 随机散布障碍物（变体） |
| `旋转对称` | 旋转对称结构 |
| `旋转对称_1` | 旋转对称结构（变体） |
| `模拟房间` | 类室内环境 |
| `模拟房间（多目标分配）` | 类室内环境，多目标点 |
| `迷宫` | 迷宫结构 |
| `25随机` | 25 个随机目标点 |
| `tspcss` | TSP 专用测试地图 |
