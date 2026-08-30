# Terminology Ledger — Obstacle-Aware Multi-Point Traversal Paper

| Canonical term | First-use definition | Variants in source | Decision |
|---|---|---|---|
| Adaptive Exponential Weighted Heuristic (AEWH) | — | "自适应指数加权启发式", "adaptive weighted heuristic" | Define as AEWH on first use |
| Binary Heap Priority Queue | — | "二叉堆优先队列" | Use "binary heap" after first mention |
| Path Simplification | — | "拐角裁剪", "SimplifyPath", "path pruning", "corner-cutting" | Use "path simplification" |
| Safety-Distance-Aware Simplification | — | "安全距离检测", "safety margin" | Describe as "safety-distance-aware" |
| Path Smoothing | — | "路径平滑", "SmoothPath" | Use "path smoothing" |
| Cubic Spline Smoothing | — | "三次样条插值平滑" | Specify "arc-length parameterized cubic spline" |
| Ant Colony Optimization (ACO) | Ant Colony Optimization (ACO) | "蚁群算法", "ACO" | Spell out once |
| Max-Min Ant System (MMAS) | — | "最大最小蚁群系统", "MMAS" | Spell out once |
| Variable Neighborhood Descent (VND) | — | "可变邻域下降搜索" | Spell out once |
| Candidate List (CL) | — | "候选列表", "candidate list" | Use "candidate list" |
| Pseudo-Random Proportional Rule | — | "伪随机比例规则", "ACS-style" | Describe as "ACS-style pseudo-random proportional rule" |
| S-Curve Progressive Local Search | — | "S曲线渐进局部搜索", "progressive local search" | Use "S-curve progressive local search" |
| Open TSP | TSP with fixed start and end points | "开放TSP", "指定起点终点的TSP" | Define contrast with "closed TSP" |
| Virtual Node (Dummy Node) | Method to convert open TSP to closed TSP by adding a virtual point | "虚拟节点", "哑节点" | Use "virtual node" after first mention |
| Cost Matrix | — | "代价矩阵", "distance matrix" | Use "cost matrix" consistently |
| Occupancy Grid | — | "占用栅格", "occGrid" | Use "occupancy grid" |
| Safety Margin | — | "安全裕度", "safetyMargin" | Use "safety margin" |
| Grid Map | — | "栅格地图" | Use "grid map" |
| 8-neighbor connectivity | — | "8邻域", "8-direction" | Use "8-neighbor connectivity" |
| Tie-breaking | — | "tie-breaking", "打破平局" | Use "tie-breaking" |
| Arc-length parameterization | — | "弧长参数化" | Use "arc-length parameterization" |
| Pheromone evaporation rate (ρ) | — | "信息素蒸发率", "rho" | Use "ρ" |
| Heuristic weight exponent (α, β) | — | alpha, beta | Distinguish A* α,β from ACO α,β |
| Adaptive stopping | — | "自适应停止", "early stopping" | Use "adaptive stopping" |
| Ablation study | — | "消融实验" | Use "ablation study" |

## Key distinctions to maintain:
- A* uses α, β for adaptive heuristic; ACO uses α (pheromone weight), β (heuristic weight), ρ (evaporation)
- Use superscript notation to disambiguate: α_A*, β_A* vs α_ACO, β_ACO
- "Path simplification" ≠ "path smoothing": simplification removes redundant waypoints, smoothing generates continuous curve
