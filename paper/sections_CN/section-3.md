# 3 问题形式化

本节将障碍环境多目标点遍历路径规划问题进行形式化定义。首先建立基于栅格的环境模型，然后定义具有固定起点和终点的开放旅行商问题，最后给出完整的优化目标。

## 3.1 栅格地图与障碍物表示

运行环境建模为$n \times n$单元的二维方形栅格地图。每个栅格单元被分类为自由空间或障碍物：

$$\mathbf{M}(r, c) = \begin{cases}
    0, & \text{栅格}(r, c)\text{为自由空间} \\
    1, & \text{栅格}(r, c)\text{被占用（障碍物）}
\end{cases} \quad (1)$$

其中$r \in \{1, \dots, n\}$和$c \in \{1, \dots, n\}$分别为行索引和列索引。假设地图在规划前静态且完全已知。

相邻自由栅格之间的移动遵循8邻域连通性：从栅格$(r, c)$可向其周围8个相邻栅格$(r + \Delta r, c + \Delta c)$移动，其中$\Delta r, \Delta c \in \{-1, 0, 1\}$，$(\Delta r, \Delta c) \neq (0, 0)$，前提是目标栅格为自由空间。移动代价区分正交步和对角步：

$$\text{cost}(\Delta r, \Delta c) = \begin{cases}
    1, & |\Delta r| + |\Delta c| = 1 \quad \text{（正交步）} \\
    \sqrt{2}, & |\Delta r| + |\Delta c| = 2 \quad \text{（对角步）}
\end{cases} \quad (2)$$

对于对角移动，相邻的两个正交栅格也必须同时为自由空间；否则机器人将穿越障碍物角点，这对非点状机器人来说在物理上不可行。

栅格坐标$(r, c)$通过将栅格中心置于其几何中心处映射为连续世界坐标$(x, y)$：

$$x = c - 0.5, \quad y = r - 0.5 \quad (3)$$

此映射将连续坐标系的原点置于栅格左下角（栅格$(1,1)$映射为$(0.5, 0.5)$），与占用栅格地图中机器人定位的标准约定一致。

## 3.2 具有固定起点和终点的开放旅行商问题

设任务规格包含一个起点$S$、$K$个中间目标点$\mathcal{T} = \{T_1, T_2, \dots, T_K\}$和一个终点$G$。所有点以栅格坐标$[r, c]$指定。完整点集定义为：

$$\mathcal{P} = \{p_1, p_2, \dots, p_N\}, \quad N = K + 2 \quad (4)$$

索引约定为：$p_1 = S$（起点），$p_i = T_{i-1}$（$i = 2, \dots, K+1$，目标点），$p_N = G$（终点）。起点和终点不重合，占据任何可行解的边界位置。

一个有效路径由访问顺序$\boldsymbol{\pi} = (\pi_1, \pi_2, \dots, \pi_N)$定义，该顺序是索引集$\{1, 2, \dots, N\}$的排列，满足边界约束：

$$\pi_1 = 1 \quad \text{（起点）}, \qquad \pi_N = N \quad \text{（终点）} \quad (5)$$

中间索引$\{\pi_2, \dots, \pi_{N-1}\}$构成$\{2, \dots, N-1\}$的一个排列，表示目标点的有序访问。可行路径的总数为$K!$，对应中间目标点的所有排列且端点固定。这是**开放旅行商问题**的形式化，区别于经典闭环旅行商问题——后者的路径构成环路且起始节点可任意选取 [3], [4]。

## 3.3 优化目标

对于任意有序点对$(p_i, p_j)$，设$\mathcal{P}(p_i, p_j)$表示从$p_i$到$p_j$在栅格地图$\mathbf{M}$中的最短无碰撞路径，由全局路径规划器计算。该路径为栅格单元序列：

$$\mathcal{P}(p_i, p_j) = [p_i = q_1, q_2, \dots, q_m = p_j], \quad q_k \in \{(r, c) \mid \mathbf{M}(r, c) = 0\} \quad (6)$$

路径长度为序列中各边代价之和：

$$L\big(\mathcal{P}(p_i, p_j)\big) = \sum_{k=1}^{m-1} \text{cost}(q_{k+1} - q_k) \quad (7)$$

其中$\text{cost}(\cdot)$由式(2)定义。若$p_i$和$p_j$之间不存在无碰撞路径（两点被障碍物阻隔），则$\mathcal{P}(p_i, p_j) = \varnothing$，$L(\varnothing) = \infty$。

路径$\boldsymbol{\pi}$的总代价为$N-1$段连续路径长度之和：

$$C(\boldsymbol{\pi}) = \sum_{k=1}^{N-1} L\big(\mathcal{P}(p_{\pi_k}, p_{\pi_{k+1}})\big) \quad (8)$$

障碍环境多目标点遍历问题表述为：

$$\boldsymbol{\pi}^* = \arg\min_{\boldsymbol{\pi} \in \Pi} \; C(\boldsymbol{\pi}) \quad (9)$$

$$\text{s.t.} \quad \pi_1 = 1,\; \pi_N = N,\; \{\pi_2, \dots, \pi_{N-1}\} = \{2, \dots, N-1\}$$

$$\mathcal{P}(p_{\pi_k}, p_{\pi_{k+1}}) \neq \varnothing \quad \forall k \in \{1, \dots, N-1\}$$

其中$\Pi$为所有满足边界约束的$\{1, \dots, N\}$排列的集合。式(9)中的优化耦合了两个相互作用的子问题：（1）为每段路径寻找最短无碰撞路径（几何问题，由全局规划器求解）；（2）确定中间目标点的最优排序（组合问题，由旅行商求解器求解）。代价矩阵元素$\mathbf{D}(i, j) = L(\mathcal{P}(p_i, p_j))$构成两个子问题之间的接口，将完整的障碍约束距离拓扑编码为$N \times N$矩阵。该解耦是精确的：对于给定的代价矩阵$\mathbf{D}$，式(9)的优化退化为开放旅行商问题；对于给定的访问顺序$\boldsymbol{\pi}$，每段路径是独立的全局规划问题。

在特殊情况$K = 0$（无中间目标点）下，问题退化为单对最短路径查询：$\boldsymbol{\pi}^* = (1, N)$，$C(\boldsymbol{\pi}^*) = L(\mathcal{P}(S, G))$，即经典单对路径规划问题。
