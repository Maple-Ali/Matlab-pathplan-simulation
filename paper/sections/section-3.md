# 3. Problem Formulation

This section formalizes the obstacle-aware multi-point traversal path planning problem. We first define the grid-based environment model, then formulate the open Traveling Salesman Problem with fixed start and end points, and finally state the complete optimization objective.

## 3.1 Grid Map and Obstacle Representation

The operating environment is modeled as a two-dimensional square grid map of size $n \times n$ cells. Each cell is classified as either free space or an obstacle:

$$\mathbf{M}(r, c) = \begin{cases}
    0, & \text{cell } (r, c) \text{ is free} \\
    1, & \text{cell } (r, c) \text{ is occupied (obstacle)}
\end{cases} \quad (1)$$

where $r \in \{1, \dots, n\}$ and $c \in \{1, \dots, n\}$ are the row and column indices, respectively. The map is assumed to be static and fully known prior to planning.

Movement between adjacent free cells follows 8-neighbor connectivity: a transition is allowed from cell $(r, c)$ to any of its eight surrounding cells $(r + \Delta r, c + \Delta c)$ with $\Delta r, \Delta c \in \{-1, 0, 1\}$, $(\Delta r, \Delta c) \neq (0, 0)$, provided that the target cell is free. The movement cost distinguishes between cardinal and diagonal steps:

$$\text{cost}(\Delta r, \Delta c) = \begin{cases}
    1, & \text{if } |\Delta r| + |\Delta c| = 1 \quad \text{(cardinal)} \\
    \sqrt{2}, & \text{if } |\Delta r| + |\Delta c| = 2 \quad \text{(diagonal)}
\end{cases} \quad (2)$$

For diagonal moves, both adjacent cardinal cells must also be free; otherwise the robot would cut through the corner of an obstacle, which is physically infeasible for a non-point agent.

A grid coordinate $(r, c)$ maps to a continuous-world coordinate $(x, y)$ by placing the cell center at:

$$x = c - 0.5, \quad y = r - 0.5 \quad (3)$$

This mapping positions the origin of the continuous coordinate system at the bottom-left corner of the grid (cell $(1,1)$ maps to $(0.5, 0.5)$), consistent with the standard convention for robot localization in occupancy grid maps.

## 3.2 Open TSP with Fixed Start and End Points

Let the mission specification consist of a start point $S$, a set of $K$ intermediate target points $\mathcal{T} = \{T_1, T_2, \dots, T_K\}$, and a goal point $G$. All points are specified as grid coordinates $[r, c]$. The complete point set is defined as:

$$\mathcal{P} = \{p_1, p_2, \dots, p_N\}, \quad N = K + 2 \quad (4)$$

with the index convention $p_1 = S$ (start), $p_i = T_{i-1}$ for $i = 2, \dots, K+1$ (targets), and $p_N = G$ (goal). The start and goal are distinct and occupy fixed positions at the boundaries of any feasible solution.

A valid tour is defined by a visit order $\boldsymbol{\pi} = (\pi_1, \pi_2, \dots, \pi_N)$ that is a permutation of the index set $\{1, 2, \dots, N\}$ subject to the boundary constraints:

$$\pi_1 = 1 \quad \text{(start)}, \qquad \pi_N = N \quad \text{(goal)} \quad (5)$$

The $K = N - 2$ intermediate indices $\{\pi_2, \dots, \pi_{N-1}\}$ form a permutation of $\{2, \dots, N-1\}$, representing the ordered visitation of the target points. The total number of feasible tours is $K!$, corresponding to all permutations of the intermediate targets with fixed endpoints. This is the **open TSP** formulation, in contrast to the classical closed TSP where the tour forms a cycle and the start node is arbitrary.

## 3.3 Optimization Objective

For any ordered pair of points $(p_i, p_j)$, let $\mathcal{P}(p_i, p_j)$ denote the shortest collision-free path from $p_i$ to $p_j$ in the grid map $\mathbf{M}$, computed by a global path planner. The path is a sequence of grid cells:

$$\mathcal{P}(p_i, p_j) = [p_i = q_1, q_2, \dots, q_m = p_j], \quad q_k \in \{(r, c) \mid \mathbf{M}(r, c) = 0\} \quad (6)$$

The length of a path is the sum of edge costs along the sequence:

$$L\big(\mathcal{P}(p_i, p_j)\big) = \sum_{k=1}^{m-1} \text{cost}(q_{k+1} - q_k) \quad (7)$$

where $\text{cost}(\cdot)$ is defined by Eq. (2). If no collision-free path exists between $p_i$ and $p_j$ (the points are disconnected by obstacles), then $\mathcal{P}(p_i, p_j) = \varnothing$ and $L(\varnothing) = \infty$.

The total cost of a tour $\boldsymbol{\pi}$ is the sum of the lengths of the $N-1$ consecutive segment paths:

$$C(\boldsymbol{\pi}) = \sum_{k=1}^{N-1} L\big(\mathcal{P}(p_{\pi_k}, p_{\pi_{k+1}})\big) \quad (8)$$

The obstacle-aware multi-point traversal problem is then stated as:

$$\boldsymbol{\pi}^* = \arg\min_{\boldsymbol{\pi} \in \Pi} \; C(\boldsymbol{\pi}) \quad (9)$$

$$\text{s.t.} \quad \pi_1 = 1,\; \pi_N = N,\; \{\pi_2, \dots, \pi_{N-1}\} = \{2, \dots, N-1\}$$

$$\mathcal{P}(p_{\pi_k}, p_{\pi_{k+1}}) \neq \varnothing \quad \forall k \in \{1, \dots, N-1\}$$

where $\Pi$ is the set of all permutations of $\{1, \dots, N\}$ satisfying the boundary constraints. The optimization in Eq. (9) couples two interacting sub-problems: (i) finding the shortest collision-free path for each segment (a geometric problem solved by the global planner), and (ii) determining the optimal ordering of intermediate targets (a combinatorial problem solved by the TSP solver). The cost matrix entries $\mathbf{D}(i, j) = L(\mathcal{P}(p_i, p_j))$ form the interface between these two sub-problems, encoding the full obstacle-constrained distance topology as an $N \times N$ matrix. The decoupling is exact: for a given cost matrix $\mathbf{D}$, the optimization in Eq. (9) reduces to the open TSP, and for a given visit order $\boldsymbol{\pi}$, each segment path is an independent global planning problem.

In the special case $K = 0$ (no intermediate targets), the problem degenerates to a single shortest-path query: $\boldsymbol{\pi}^* = (1, N)$ and $C(\boldsymbol{\pi}^*) = L(\mathcal{P}(S, G))$, which reduces to the classical single-pair path planning problem.
