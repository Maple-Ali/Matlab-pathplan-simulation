# 4 方法论

本节介绍所提出的障碍物感知多点遍历路径规划框架。该框架由两个协同工作的模块组成：改进型 A* 全局路径规划器（AStar_v3_1）和改进型蚁群优化求解器（TSP_ACO_v2_4）。前者负责在栅格地图上搜索从起点到目标点的无碰撞最短路径，并通过安全距离感知的路径简化与三次样条平滑生成高质量的连续路径；后者则基于全局规划器构建的成本矩阵，求解多目标点的最优访问序列。两个模块通过成本矩阵的构建实现协同，共同完成从路径规划到序列优化的完整流程。4.1 节和 4.2 节介绍全局路径规划及后处理方法，4.3 节详述 TSP 求解算法，4.4 节阐述两层协同框架的整体设计。

## 4.1 改进型 A* 全局路径规划

### 4.1.1 基于跳跃点搜索的 A* 算法

传统 A* 算法在搜索过程中需要扩展大量节点，尤其在大规模开阔区域中，节点扩展数量与地图面积成正比，计算效率受到严重制约。为解决这一问题，AStar_v3_1 引入了跳跃点搜索（Jump Point Search, JPS）策略 [11], [12]。JPS 的核心思想是利用栅格地图的对称性，在搜索过程中跳过大量冗余的中间节点，仅保留具有"跳跃意义"的关键节点作为扩展候选。

**邻居剪枝规则。** 在传统 A* 中，当前节点的所有邻居均需加入 open set 进行评估。JPS 则通过邻居剪枝规则大幅减少需要考虑的邻居数量。剪枝策略取决于当前节点的扩展方向：

1. **直线移动（水平或垂直）**：仅保留自然邻居（移动方向前方的节点）和被迫邻居（因障碍物阻挡而被迫改变方向的节点）。对于直线移动方向两侧的节点，除非被障碍物阻挡产生被迫邻居，否则均被剪枝。

2. **对角线移动**：保留自然邻居（对角方向前方）、直线方向的自然邻居（水平和垂直分量方向），以及因障碍物产生的被迫邻居。

被迫邻居的判定条件为：当前节点沿移动方向的相邻位置被障碍物阻挡，但沿另一方向可达。图 X 给出了剪枝规则的示意图。

> **[图 X]** JPS 邻居剪枝规则示意图。(a) 直线移动的剪枝：自然邻居 N 和被迫邻居 F；(b) 对角线移动的剪枝：自然邻居 N1（对角）、N2（水平）、N3（垂直）和被迫邻居 F。

**跳跃函数。** JPS 的跳跃函数 `jump(r, c, dr, dc)` 从当前节点 $(r, c)$ 出发，沿方向 $(dr, dc)$ 逐步前进，直到满足以下三个终止条件之一：(1) 到达目标节点；(2) 遇到障碍物或地图边界（该方向不可通行）；(3) 发现被迫邻居（当前位置因障碍物阻挡而需要强制转向）。跳跃过程中经过的中间节点不会被加入 open set，从而显著减少了节点扩展数量。当遇到被迫邻居时，当前节点即为跳跃点，需要加入 open set 进行后续评估。

对于对角线方向的跳跃，算法会先沿对角线前进一步，然后分别沿水平和垂直分量方向执行直线跳跃。若任一方向的直线跳跃发现了被迫邻居或到达目标，则返回当前对角线位置作为跳跃点。

**路径成本计算。** 在 `jump` 函数返回可达的下一个节点后，AStar_v3_1 需要计算从当前节点到该跳跃点的实际移动成本。对于直线跳跃，成本等于跳跃的曼哈顿距离；对于对角线跳跃，成本等于对角线距离（每步对角线移动成本为 √2）。算法 1 给出了 JPS 搜索过程的伪代码。

---

**Algorithm 1: JPS-based A\* search**

**Input:** occupancy grid map, start grid $S$, goal grid $G$

**Output:** grid path $P = [S, \ldots, G]$, or failure

1.  initialize $g(S) = 0$, $f(S) = h(S, G)$, and insert $S$ into the open set (a binary min-heap);
2.  **while** the open set is not empty **do**
3.  $\quad n \gets$ the node in the open set with the smallest $f$-value, breaking ties by the smaller $h$;
4.  $\quad$ **if** $n = G$ **then** reconstruct and return the grid path $P$;
5.  $\quad$ move $n$ into the closed set, and record its parent direction $\mathbf{d}_n$;
6.  $\quad D \gets$ the pruned successor directions of $n$ determined by $\mathbf{d}_n$ (all eight directions if $n = S$);
7.  $\quad$ **for each** direction $\mathbf{d} \in D$ **do**
8.  $\quad\quad s \gets \text{Jump}(n, \mathbf{d})$, the first jump point encountered along $\mathbf{d}$, or null;
9.  $\quad\quad$ **if** $s = \text{null}$ **or** $s$ is closed **then** continue;
10. $\quad\quad \text{tent\_g} \gets g(n) + \text{Cost}(n, s)$, where straight steps cost $1$ and diagonal steps cost $\sqrt{2}$;
11. $\quad\quad$ **if** $\text{tent\_g} < g(s)$ **then**
12. $\quad\quad\quad$ set the parent of $s$ to $n$, $g(s) \gets \text{tent\_g}$, $f(s) \gets \text{tent\_g} + h(s, G)$;
13. $\quad\quad\quad$ insert $s$ into the open set, or update its heap position if already present;
14. $\quad\quad$ **end**
15. $\quad$ **end**
16. **end**
17. **if** the open set is exhausted without reaching $G$ **then** fall back to standard A\* (AStar\_v0);

---

**路径重建与对角拐点修正。** JPS 搜索完成后，通过父节点指针从目标节点回溯至起点，并在相邻跳跃点之间沿直线插值补充被跳过的中间栅格，即可得到完整路径。然而，由于 JPS 的跳跃特性，重建路径中的对角线移动可能穿越障碍物拐角。为此，AStar_v3_1 在路径重建阶段引入了对角拐点修正机制：当检测到对角线移动穿越障碍物角时，将其替换为沿栅格边界行走的 L 形路径（先走行再走列，或先走列再走行），从而避免与障碍物发生碰撞。修正后的路径严格沿栅格边界行进，保证了路径的可行性。

**算法降级机制。** JPS 的剪枝规则依赖于均匀的栅格地图结构。在包含复杂障碍物分布的地图中，被迫邻居的产生频率增加，JPS 的优势会被削弱，个别情况下对称性假设失效可能导致 JPS 搜索失败。为此，AStar_v3_1 在 JPS 搜索耗尽开放集仍未找到路径时，自动回退至标准 A* 模式（AStar_v0）重新搜索，以确保在存在可行解时总能找到路径。这一降级机制保证了算法的鲁棒性，使其能够适应不同复杂度的环境。

### 4.1.2 打破平局策略

当开放集中的多个节点具有相同的 $f$ 值时，标准 A* 的选择顺序取决于具体实现（通常是先入先出或随机），本质上是任意的。然而，在 $f$ 值相同的节点中，$h$ 值较小的节点意味着其实际代价 $g = f - h$ 较大，即该节点距起点更远、更接近目标。优先扩展此类节点可将搜索方向导向目标，且不影响最优性保证。为此，AStar_v3_1 采用"$f$ 值相同时优先选择 $h$ 值更小的节点"的打破平局策略，二叉堆中的节点比较规则定义为：

$$
\text{compare}(n_1, n_2) = \begin{cases}
n_1, & f(n_1) < f(n_2) \\
n_2, & f(n_1) > f(n_2) \\
n_1, & f(n_1) = f(n_2) \land h(n_1) < h(n_2) \\
n_2, & f(n_1) = f(n_2) \land h(n_1) > h(n_2)
\end{cases}
\tag{1}
$$

该比较规则直接嵌入二叉堆的上浮（`bubbleUp`）与下沉（`bubbleDown`）操作中。

**物理意义。** 由评估函数 $f(n) = g(n) + h(n)$ 可知，当两个节点的 $f$ 值相同时，$g$ 值较大者 $h$ 值较小。$g$ 值较大表示该节点距起点更远，而 $h$ 值较小表示其更接近目标。优先扩展此类节点使搜索偏向目标方向，减少了远离目标区域的无效探索。由于 $h$ 值在节点扩展时已经计算，此打破平局策略不产生额外计算开销。

## 4.2 路径后处理

JPS 搜索得到的路径由栅格中心点序列组成，存在两个主要问题：路径包含冗余的中间点，且由离散的水平和垂直段组成导致平滑性不足。本节提出一种两阶段后处理流水线来解决这些问题。

### 4.2.1 安全距离感知的路径简化

传统的路径简化方法（如拐角裁剪）直接连接路径上的非相邻节点，跳过中间点以减少路径点数量。然而，这类方法通常仅检查连接线段是否穿越障碍物，忽略了路径点与障碍物之间的安全裕度，在障碍物密集区域可能导致路径过于贴近障碍物。为此，本节提出一种安全距离感知的路径简化算法，其核心思想是在简化过程中保持路径与障碍物之间的最小安全距离。

该算法采用四步流程处理输入路径：

**第一步：拐点提取。** 从路径中识别方向发生变化的节点（即拐点）。对于路径中的每个中间节点，计算其与前后节点的连接方向。若前向连接方向与后向连接方向不同，则该节点标记为拐点。路径的起点和终点始终保留。

**第二步：贪心前向扫描。** 从当前拐点 $C_i$ 出发，正向扫描所有后续拐点 $C_j$（$j > i$），调用 `IsLineFree` 逐一验证线段 $C_i \to C_j$ 是否满足安全距离约束，并记录**最远的**可达拐点 $j^{*}$。该拐点成为新的锚点，其间的所有拐点被跳过。这种贪心策略在保证安全性的前提下最大化了单步简化程度。

**第三步：中间点探索。** 贪心步骤确定最远可达拐点 $j^{*}$ 后，算法检查被跳过的拐点 $k$（$i < k < j^{*}$）能否通过安全线段直达更远处的拐点 $j'$（$j' > j^{*}$）。对每一对满足安全约束的 $(k, j')$，算法计算并比较两条路径的代价。若存在这样的中间拐点，则可能得到比纯贪心更短的简化路径。

**第四步：路径比较。** 比较两条路径的长度：路径 A（贪心）为 $d(C_i, C_{j^{*}}) + d(C_{j^{*}}, C_{j'})$，即先到达 $j^{*}$ 再继续到 $j'$；路径 B（经中间点）为 $d(C_i, C_k) + d(C_k, C_{j'})$，即跳过 $j^{*}$ 改经 $k$ 路由。若路径 B 更短，则保留中间拐点 $C_k$；否则仅保留贪心结果 $C_{j^{*}}$。这一比较确保中间点探索不会以牺牲路径效率为代价。

安全性检查函数 `IsLineFree(p1, p2, map, dSafe)` 的实现基于逐栅格采样：从 $p_1$ 到 $p_2$ 沿线段密集采样，对每个采样点计算其与最近障碍物栅格边界的距离。只要任意采样点的距离小于安全阈值 $d_{\text{safe}}$，即判定该连接不安全。点到栅格边界的距离计算公式为：

$$
d_x = \max\left(0,\; |p_r - c_r| - 0.5\right), \quad
d_y = \max\left(0,\; |p_c - c_c| - 0.5\right)
\tag{2}
$$

$$
d(p, \text{cell}) = \sqrt{d_x^2 + d_y^2}
\tag{3}
$$

其中，$p = (p_r, p_c)$ 为连续坐标下的采样点，$c = (c_r, c_c)$ 为障碍物栅格的中心坐标，栅格占据矩形区域 $[c_r - 0.5, c_r + 0.5] \times [c_c - 0.5, c_c + 0.5]$。$\max(0, \cdot)$ 算子处理采样点在某坐标轴上已落入栅格水平或垂直范围内的情况。当 $d(p, \text{cell}) < d_{\text{safe}}$ 时，表示采样点已进入障碍物栅格的安全禁区。安全距离阈值 $d_{\text{safe}}$ 设置为机器人半径 $r_{\text{robot}} = 0.2$ 加上 $0.2$ 的裕度（即 $d_{\text{safe}} = 0.4$），保证路径中心线到障碍物的距离不小于机器人半径，从而保证无碰撞。算法 2 给出了路径简化算法的伪代码。

---

**Algorithm 2: Safety-distance-aware path simplification**

**Input:** grid path $P = [P_1, \ldots, P_N]$, occupancy grid, safety margin $d_{\text{safe}}$

**Output:** simplified path $P_s$ ($|P_s| \leq N$)

1.  $C \gets$ the corner points of $P$ where the direction changes, plus the two endpoints;
2.  $P_s \gets [C_1]$; $\; i \gets 1$;
3.  **while** $i < |C|$ **do**
4.  $\quad j^{*} \gets$ the farthest corner reachable from $C_i$ by a collision-free segment; $\;$ *(greedy forward scan)*
5.  $\quad$ **if** $j^{*} = |C|$ **then** append $C_{|C|}$ to $P_s$ and break;
6.  $\quad (k^{*}, j') \gets$ among all skipped corners $k$ ($i < k < j^{*}$) and farther corners $j$ ($j^{*} < j \leq |C|$), the pair with the largest $j$ such that the segment $C_k \to C_j$ is collision-free;
7.  $\quad$ **if** $k^{*}$ exists and $d(C_i, C_{k^{*}}) + d(C_{k^{*}}, C_{j'}) < d(C_i, C_{j^{*}}) + d(C_{j^{*}}, C_{j'})$ **then**
8.  $\quad\quad$ append $C_{k^{*}}$ and $C_{j'}$ to $P_s$, and set $i \gets j'$; $\;$ *(route through the intermediate corner)*
9.  $\quad$ **else**
10. $\quad\quad$ append $C_{j^{*}}$ to $P_s$, and set $i \gets j^{*}$;
11. $\quad$ **end**
12. **end**
13. **return** $P_s$;

---

安全性检查函数 $\textsc{IsLineFree}$ 的伪代码如算法 3 所示。搜索半径 $d_{\max}$ 设置为 $\lceil d_{\text{safe}} + 0.5 \rceil$，因为超出此距离的栅格不可能与采样点的安全距离产生交集。

---

**Algorithm 3: IsLineFree** (collision check with safety margin)

**Input:** segment endpoints $p_1$, $p_2$ (grid coordinates), occupancy grid, safety margin $d_{\text{safe}}$

**Output:** true if the segment keeps distance $\geq d_{\text{safe}}$ from every obstacle, false otherwise

1.  $\ell \gets \max(|p_1.r - p_2.r|,\; |p_1.c - p_2.c|)$;
2.  $N_s \gets \max(\lceil 10 \cdot \ell \rceil, 30)$; $\;$ *(number of dense sample points)*
3.  $d_{\max} \gets \lceil d_{\text{safe}} + 0.5 \rceil$; $\;$ *(search radius in cells)*
4.  **for** $k = 0$ **to** $N_s$ **do**
5.  $\quad p \gets p_1 + (k / N_s) \cdot (p_2 - p_1)$; $\;$ *(sample point in continuous coordinates)*
6.  $\quad$ **for each** cell $(r, c)$ within Chebyshev distance $d_{\max}$ of $p$ **do**
7.  $\quad\quad$ **if** $(r, c)$ is occupied **then**
8.  $\quad\quad\quad d \gets$ the distance from $p$ to the boundary of cell $(r, c)$; $\;$ *(see Eq. (2)–(3))*
9.  $\quad\quad\quad$ **if** $d < d_{\text{safe}}$ **then return** false;
10. $\quad\quad$ **end**
11. $\quad$ **end**
12. **end**
13. **return** true;

---

### 4.2.2 弧长参数化三次样条平滑

路径简化后的路径虽已去除冗余点，但仍由直线段连接组成，在拐点处存在曲率突变，不适合全向机器人的平滑运动。本节采用三次样条插值对简化路径进行平滑处理。

传统的三次样条插值直接以路径点的索引作为参数进行拟合，当路径点间距不均匀时会导致曲线形状失真。为解决这一问题，本节采用弧长参数化方法，以路径点之间的累积欧氏距离作为参数，使参数空间与实际几何空间保持一致。平滑过程分为三个步骤：

**步骤一：稀疏段致密化。** 首先检测路径中相邻点间距超过 2 个栅格单位的线段。对于这些稀疏段，在两端点之间沿线段线性插值 $\lfloor \ell / 2 \rfloor$ 个中间点。致密化的目的是为样条曲线提供足够的控制点，防止曲线在长直段处因控制点不足而产生的过冲（Runge 现象），避免曲线偏离预期折线甚至裁剪障碍物角点。

**步骤二：弧长参数化。** 设致密化后的路径点序列为 $\{P_0, P_1, \ldots, P_M\}$，其中 $P_i = (x_i, y_i)$ 为连续坐标。计算各点的累积弦长（弧长）参数：

$$
s_0 = 0, \quad s_i = s_{i-1} + \|P_i - P_{i-1}\|_2, \quad i = 1, 2, \ldots, M
\tag{4}
$$

然后分别以 $s$ 为自变量对 $x(s)$ 和 $y(s)$ 进行三次样条插值，得到参数化曲线 $(x(s), y(s))$。三次样条在每个子区间 $[s_i, s_{i+1}]$ 上为三次多项式，保证一阶和二阶导数连续，从而确保曲线的曲率连续性。

**步骤三：等弧长重采样。** 以密度 $\rho = 10$（每段插入 10 个点）沿弧长参数 $s$ 均匀采样，生成最终的平滑路径点序列。由于 $s$ 与实际几何距离近似线性关系，等弧长采样保证了路径点在空间中的均匀分布，避免了传统等参数采样在曲率大处点密、曲率小处点疏的问题。算法 4 给出了平滑算法的伪代码。

---

**Algorithm 4: Arc-length parameterized cubic spline smoothing**

**Input:** grid path $P = [P_1, \ldots, P_N]$, density $\rho$ (default $10$)

**Output:** smoothed continuous path $Q = [Q_1, \ldots, Q_M]$ in $[x, y]$

1.  convert each grid point $P_i = (r_i, c_i)$ to continuous coordinates $(x_i, y_i) = (c_i - 0.5,\; r_i - 0.5)$;
2.  **for each** consecutive pair whose segment length exceeds $2$ **do**
3.  $\quad$ insert $\lfloor \text{length} / 2 \rfloor$ equally spaced intermediate points along the segment; $\;$ *(densification to prevent spline overshoot)*
4.  **end**
5.  compute the cumulative chord length $t_1 = 0$, $t_i = t_{i-1} + \|P_i - P_{i-1}\|_2$; $\;$ *(arc-length parameter)*
6.  remove duplicate $t$ values to keep strict monotonicity;
7.  fit cubic splines $x(t)$ and $y(t)$ through the densified points;
8.  sample $x(t)$ and $y(t)$ at $\rho$ points per segment to obtain $Q$;
9.  **return** $Q$;

---

### 4.2.3 流水线集成

上述后处理操作以流水线方式集成：JPS 搜索（算法 1）输出原始栅格路径 → SimplifyPath（算法 2）去除冗余点 → SmoothPath（算法 4）生成平滑的连续坐标路径。整个流水线在 SimulationManager 中一次性完成，输出的路径既满足安全性约束，又具有良好的平滑性，可直接用于机器人的运动跟踪。

这种两阶段设计的合理性在于：路径简化和样条平滑分别解决了路径质量的两个不同维度。简化减少了路径点数量（降低了后续计算的复杂度），同时通过安全距离约束保证了路径的安全裕度；平滑则将离散的栅格路径转化为连续曲线，使机器人能够以平滑的运动轨迹跟踪路径。若跳过简化直接进行平滑，样条曲线需要处理大量冗余的控制点，不仅计算成本更高，还可能因控制点过于密集而产生不稳定的曲线形状。消融实验（表 6）的结果也验证了这一设计的有效性。
