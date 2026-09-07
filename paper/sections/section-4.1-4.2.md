# 4. Methodology

## 4.1 Improved A* Global Path Planner

The first layer of our framework computes pairwise obstacle-aware path costs between every pair of points in the visitation set. We adopt A* as the base planner and introduce Jump Point Search (JPS) as the core algorithmic improvement. JPS exploits the symmetry inherent in uniform-cost grid maps to prune large portions of the search tree, expanding only "jump points"—nodes where the optimal path changes direction or encounters a forced choice—while skipping all intermediate nodes that can be reached by a straight-line traversal. The open set is implemented as a binary min-heap with a tie-breaking strategy that biases expansion toward the goal direction.

### 4.1.1 Jump Point Search

**Motivation.** Standard A* expands every node in the open set uniformly, examining all 8 neighbors at each expansion step. On uniform-cost grid maps, many of these expansions are redundant: when moving in a straight line (horizontally, vertically, or diagonally) through open space, every intermediate node along the line leads to the same successor. Standard A* expands each of these intermediate nodes individually, wasting computation. Jump Point Search identifies this symmetry and "jumps" along straight lines until it encounters a point where the path must make a decision—either because an obstacle creates a forced neighbor, or because the goal is reached.

**Mechanism.** The JPS algorithm modifies the standard A* expansion step. Instead of examining all 8 neighbors, each node is expanded by examining a set of pruned search directions, then "jumping" along each direction until a jump point is found.

**Neighbor pruning.** When expanding a node $(r, c)$ that was reached from its parent via direction $(p_{dr}, p_{dc})$, the set of successor directions is pruned based on the parent direction:

- **Start node** ($p_{dr} = 0, p_{dc} = 0$): all 8 directions are considered.
- **Straight move** (one of $p_{dr}$ or $p_{dc}$ is zero): the natural successor is the continuation direction. Additionally, if an obstacle blocks a lateral cell and the diagonal cell beyond it is free, that diagonal becomes a forced direction.
- **Diagonal move** (both $p_{dr}$ and $p_{dc}$ nonzero): three natural successors (the diagonal continuation and its two straight-line components). Additionally, if an obstacle blocks the lateral cell behind the diagonal, the reflected diagonal becomes forced.

The pruning rules ensure that only directions that could potentially lead to an optimal path are explored, while all others are provably dominated.

**Jump function.** For each pruned direction $(dr, dc)$, the `jump` function advances from the current node one step at a time along that direction:

1. If the next cell is an obstacle or out of bounds, return empty (dead end).
2. If the next cell is the goal, return it as a jump point.
3. If the next cell has a forced neighbor (i.e., an obstacle creates a forced directional choice), return it as a jump point.
4. For diagonal moves, recursively check the two component straight-line directions for jump points. If either produces a jump point, return the current cell as a jump point.

If none of these conditions are met, the jump continues in the same direction. This process terminates in $\mathcal{O}(n)$ steps per jump (bounded by the map size), and typically finds a jump point in far fewer steps.

**Path cost computation.** The cost of a jump from $(r, c)$ to $(r_j, c_j)$ along direction $(dr, dc)$ is accumulated step-by-step: each straight step costs $1$ and each diagonal step costs $\sqrt{2}$. This preserves the exact grid movement cost and ensures path optimality.

**Path reconstruction.** Because JPS skips intermediate nodes, the parent pointers record only the jump-point chain. The full grid path is reconstructed by linear interpolation between consecutive jump points: for each pair of jump points, the path is filled in by stepping one cell at a time along the sign of the direction vector. A post-processing step corrects diagonal corner-crossing artifacts: when a diagonal step would cut through the corner of an obstacle, the path is rerouted through the adjacent free cell to form an L-shaped detour.

**Fallback.** In rare cases where JPS fails to find a path (e.g., in narrow maze-like environments where the symmetry assumptions break down), the algorithm falls back to standard A* with a binary heap to guarantee a solution is found when one exists.

**Complexity.** On open grid maps, JPS expands significantly fewer nodes than standard A* because it skips all intermediate nodes along straight-line traversals. In the best case (a clear straight line from start to goal), JPS finds the path in a single jump. In the worst case (a maze with no straight-line symmetry), JPS degrades to standard A* performance. The binary heap operations remain $\mathcal{O}(\log m)$ per expansion, where $m$ is the open-set size.

### 4.1.2 Tie-Breaking Strategy

**Motivation.** When multiple jump points in the open set share identical $f$-values, the standard A* selection order is determined by implementation-specific iteration order, which is essentially arbitrary. However, among nodes with equal $f$, those with smaller $h$ have already incurred greater actual cost $g = f - h$ and lie closer to the goal. Preferring these nodes biases the search toward the goal direction without affecting the optimality guarantee.

**Mechanism.** When two nodes $n_1$ and $n_2$ share the same $f$-value, the node with a smaller $h$-value is expanded preferentially. The comparison rule for heap ordering is defined as:

$$\text{compare}(n_1, n_2) = \begin{cases}
    n_1, & f(n_1) < f(n_2) \\
    n_2, & f(n_1) > f(n_2) \\
    n_1, & f(n_1) = f(n_2) \land h(n_1) < h(n_2) \\
    n_2, & f(n_1) = f(n_2) \land h(n_1) > h(n_2)
\end{cases} \quad (1)$$

The comparison is embedded directly in the binary heap's bubble-up and bubble-down operations.

**Physical meaning.** From the evaluation function $f(n) = g(n) + h(n)$, when two nodes have identical $f$-values, a larger $g$ implies a smaller $h$. A larger $g$ means the node lies farther from the start, while a smaller $h$ means it is closer to the goal. Preferring such nodes biases the search toward the goal direction, reducing futile exploration in regions far from the goal. Since $h$ is already computed at node expansion time, this tie-breaking incurs no additional computational overhead.

---

## 4.2 Path Post-Processing Pipeline

The raw grid path produced by the global planner contains two artifacts inherited from grid discretization: (i) redundant colinear waypoints and staircase-shaped zigzag segments that inflate the waypoint count without reducing path length, and (ii) piecewise-linear segments connected by sharp turns that are kinematically inefficient for physical robots. We address these with a two-stage post-processing pipeline: safety-distance-aware path simplification followed by arc-length parameterized cubic spline smoothing.

### 4.2.1 Safety-Distance-Aware Path Simplification

**Motivation.** Grid-constrained paths contain intermediate waypoints at every grid cell transition. Many of these waypoints are colinear with their neighbors and can be removed without altering the geometric trace of the path. More importantly, corridors and open areas produce staircase patterns—sequences of orthogonal segments that a single straight line could replace, potentially shortening the effective path. However, naive line-of-sight pruning that does not consult the obstacle map risks connecting two waypoints through an obstacle corner, violating the collision-free guarantee. A robust simplification must verify obstacle clearance for every pruned segment.

**Mechanism.** The simplification algorithm operates in four steps:

**Step 1 — Corner extraction.** The raw path is first reduced to its turning points (corners), defined as grid cells where the movement direction changes. For a grid path where each step moves to an adjacent cell, the direction of step $i$ is $(r_{i+1} - r_i, c_{i+1} - c_i)$. A cell $i$ (for $2 \leq i \leq N-1$) is classified as a corner if its incoming direction differs from its outgoing direction:

$$(r_i - r_{i-1}, c_i - c_{i-1}) \neq (r_{i+1} - r_i, c_{i+1} - c_i)$$

The start and end points are always included. This preprocessing step dramatically reduces the number of candidate waypoints (often by $80$--$90\%$ on grid paths), making the subsequent greedy search far more efficient.

**Step 2 — Greedy forward scan.** Starting from the first corner as the current anchor, the algorithm scans all subsequent corners $j$ in forward order and selects the **farthest** corner that is directly reachable (i.e., the line-of-sight segment passes the safety check). The selected corner becomes the new anchor, and the process repeats. This is a forward scan (not the back-to-front scan used in the previous version), which is more natural when operating on the pre-filtered corner set.

**Step 3 — Intermediate point exploration.** After the greedy step identifies the farthest directly reachable corner $j$ from the current anchor $i$, the algorithm checks whether any skipped corner $k$ (between $i$ and $j$) can reach an even farther corner $j'$ (beyond $j$) via a direct line-of-sight connection. For each such pair $(k, j')$ that passes the safety check, the algorithm computes and compares two path costs:

- **Path A** (greedy): $d(i, j) + d(j, j')$ — reach $j$ first, then continue to $j'$.
- **Path B** (via intermediate): $d(i, k) + d(k, j')$ — skip $j$ entirely and route through $k$.

If Path B is shorter, the intermediate corner $k$ is retained in the output, potentially producing a shorter overall simplified path than pure greedy selection.

**Step 4 — Path comparison and selection.** The algorithm selects the shorter of the two paths (greedy vs. intermediate) and advances the anchor accordingly. This 4-step process produces a simplified path that is at most as long as the pure greedy result, and often shorter in environments where the greedy scan overshoots a useful intermediate waypoint.

**Safety check.** The core $\text{IsLineFree}$ check verifies that every point along a segment maintains at least a safety margin $d_{\text{safe}}$ from all obstacle cells. For a segment spanning $\ell = \max(|r_1 - r_2|, |c_1 - c_2|)$ grid units, we sample $N_s = \max(\lceil 10\ell \rceil, 30)$ equally spaced points. At each sample point $\mathbf{p} = (r, c)$ in continuous coordinates, all grid cells within a search radius of $d_{\max} = \lceil d_{\text{safe}} + 0.5 \rceil$ are examined.

The exact point-to-cell-boundary distance calculation is illustrated in Figure 1. A grid cell at $(r_{\text{cell}}, c_{\text{cell}})$ occupies the axis-aligned rectangle $[r_{\text{cell}} - 0.5, r_{\text{cell}} + 0.5] \times [c_{\text{cell}} - 0.5, c_{\text{cell}} + 0.5]$. The distance from a continuous sample point to the boundary of this rectangle is:

$$d_x = \max\left(0,\; |r - r_{\text{cell}}| - 0.5\right) \quad (2)$$

$$d_y = \max\left(0,\; |c - c_{\text{cell}}| - 0.5\right) \quad (3)$$

$$d(\mathbf{p}, \text{cell}) = \sqrt{d_x^2 + d_y^2} \quad (4)$$

The $\max(0, \cdot)$ operator handles the case where the sample point lies within the cell's horizontal or vertical extent along one axis. If any sample point is within $d_{\text{safe}}$ of any occupied cell, the segment is rejected.

*[Figure 1: Illustration of the exact point-to-cell-boundary distance computation. The diagram shows a sample point $\mathbf{p}$ near a grid cell, with $d_x$ and $d_y$ labeled as the horizontal and vertical distances to the cell boundary. The shaded region indicates the safety margin $d_{\text{safe}}$ around the segment.]*

The pseudocode for the simplification algorithm is given in Algorithm 1.

---

\textbf{Algorithm 1: Safety-Distance-Aware Path Simplification} \\
\hline
\textbf{Input:} $\text{path}[N \times 2]$ original grid path $[r, c]$, $\text{occGrid}$ occupancy grid ($n \times n$), $n$ map size, $d_{\text{safe}}$ safety margin \\
\textbf{Output:} $\text{simplePath}[M \times 2]$ simplified path ($M \leq N$) \\
\hline
1: \quad $\text{corners} \gets \textsc{ExtractCorners}(\text{path})$ \quad // turning points + endpoints \\
2: \quad $\text{simplePath} \gets [\text{corners}(1)]$; \quad $i \gets 1$ \\
3: \quad \textbf{while } $i < |\text{corners}|$ \textbf{ do} \\
4: \quad \quad // Greedy: farthest directly reachable corner \\
5: \quad \quad $\text{farthest} \gets \max\{\, j > i : \textsc{IsLineFree}(\text{corners}(i), \text{corners}(j)) \,\}$ \\
6: \quad \quad // Intermediate exploration: skipped corner $k$ reaching a farther corner $j$ \\
7: \quad \quad $(k^*, j^*) \gets \arg\max_{i<k<\text{farthest}<j \leq |\text{corners}|} \{\, j : \textsc{IsLineFree}(\text{corners}(k), \text{corners}(j)) \,\}$ \\
8: \quad \quad \textbf{if } $k^*$ exists and $d(i,k^*) + d(k^*,j^*) < d(i,\text{farthest}) + d(\text{farthest},j^*)$ \textbf{ then} \\
9: \quad \quad \quad $\text{simplePath} \gets [\text{simplePath};\; \text{corners}(k^*);\; \text{corners}(j^*)]$; \quad $i \gets j^*$ \\
10: \quad \quad \textbf{else} \\
11: \quad \quad \quad $\text{simplePath} \gets [\text{simplePath};\; \text{corners}(\text{farthest})]$; \quad $i \gets \text{farthest}$ \\
12: \quad \quad \textbf{end if} \\
13: \quad \textbf{end while} \\
14: \quad \textbf{return } $\text{simplePath}$ \\
\hline

---

The $\text{IsLineFree}$ subroutine (Algorithm 2) implements the dense sampling and exact distance check described above. The search radius $d_{\max}$ is set to $\lceil d_{\text{safe}} + 0.5 \rceil$ because grid cells farther than this cannot possibly have their boundary within the safety distance of any sample point.

---

\textbf{Algorithm 2: IsLineFree} \\
\hline
\textbf{Input:} $p_1, p_2$ segment endpoints $[r, c]$, $\text{occGrid}$, $n$, $d_{\text{safe}}$ \\
\textbf{Output:} $\text{free}$ (boolean) \\
\hline
1: \quad $\ell \gets \max(|p_1.r - p_2.r|, |p_1.c - p_2.c|)$ \\
2: \quad $N_s \gets \max(\lceil 10 \cdot \ell \rceil, 30)$ \quad // Dense sampling \\
3: \quad $d_{\max} \gets \lceil d_{\text{safe}} + 0.5 \rceil$ \\
4: \quad \textbf{for } $k = 0$ \textbf{ to } $N_s$ \textbf{ do} \\
5: \quad \quad $\alpha \gets k / N_s$; \quad $r \gets p_1.r + \alpha(p_2.r - p_1.r)$; \quad $c \gets p_1.c + \alpha(p_2.c - p_1.c)$ \\
6: \quad \quad $r_0 \gets \text{round}(r)$; \quad $c_0 \gets \text{round}(c)$ \\
7: \quad \quad \textbf{for } $dr = -d_{\max}$ \textbf{ to } $d_{\max}$ \textbf{ do} \\
8: \quad \quad \quad $r_{\text{cell}} \gets r_0 + dr$ \\
9: \quad \quad \quad \textbf{if } $r_{\text{cell}} < 1$ \textbf{ or } $r_{\text{cell}} > n$ \textbf{ then continue} \\
10: \quad \quad \quad \textbf{for } $dc = -d_{\max}$ \textbf{ to } $d_{\max}$ \textbf{ do} \\
11: \quad \quad \quad \quad $c_{\text{cell}} \gets c_0 + dc$ \\
12: \quad \quad \quad \quad \textbf{if } $c_{\text{cell}} < 1$ \textbf{ or } $c_{\text{cell}} > n$ \textbf{ then continue} \\
13: \quad \quad \quad \quad \textbf{if } $\neg \text{occGrid}(r_{\text{cell}}, c_{\text{cell}})$ \textbf{ then continue} \\
14: \quad \quad \quad \quad $d_x \gets \max(0, |r - r_{\text{cell}}| - 0.5)$ \\
15: \quad \quad \quad \quad $d_y \gets \max(0, |c - c_{\text{cell}}| - 0.5)$ \\
16: \quad \quad \quad \quad \textbf{if } $\sqrt{d_x^2 + d_y^2} < d_{\text{safe}}$ \textbf{ then return false} \\
17: \quad \quad \quad \textbf{end for} \\
18: \quad \quad \textbf{end for} \\
19: \quad \textbf{end for} \\
20: \quad \textbf{return true} \\
\hline

---

**Role in the framework.** In the TSP cost matrix computation (Section 4.4), an `enableSimplify` flag controls whether pairwise costs derive from raw grid paths or simplified paths. Using simplified-path Euclidean costs produces a cost matrix closer to the true continuous-path length, improving the TSP solver's ability to discriminate between alternative visitation orders. The corner extraction step (Step 1) is particularly effective when used with JPS output: JPS produces paths with fewer waypoints than standard A*, so the corner set is already small, and the greedy + intermediate exploration further reduces it to the minimal safe subset.

### 4.2.2 Arc-Length Parameterized Cubic Spline Smoothing

**Motivation.** The simplified path, while obstacle-safe and waypoint-minimal, remains piecewise-linear with discontinuous first derivatives at waypoint transitions. A physical robot following such a trajectory must decelerate and re-accelerate at each corner, increasing energy consumption and travel time. Cubic spline interpolation produces a $C^2$-continuous curve suitable for smooth trajectory tracking by local planners. However, directly fitting a cubic spline to sparsely distributed waypoints can cause the spline to overshoot between distant points—an artifact of the Runge phenomenon in polynomial interpolation—potentially clipping obstacle corners even when all waypoints are safe.

**Mechanism.** The smoothing procedure operates in three steps.

**Step 1 — Grid-to-continuous coordinate conversion.** Grid-indexed waypoints $[r, c]$ are mapped to continuous world coordinates by centering each cell at its geometric center:

$$x = c - 0.5, \quad y = r - 0.5 \quad (5)$$

**Step 2 — Sparse segment densification.** Before fitting the spline, we scan the waypoint sequence for consecutive pairs whose Euclidean distance exceeds $2$ units. For each such pair, $\lfloor \text{dist} / 2 \rfloor$ intermediate points are linearly interpolated along the segment. This densification provides the spline with sufficient knots to stay close to the intended polyline, preventing oscillation artifacts without altering the geometric path.

**Step 3 — Arc-length parameterized cubic spline.** The cumulative chordal distance along the (densified) waypoint sequence defines a monotonic parameter:

$$t_1 = 0, \quad t_k = \sum_{i=2}^{k} \sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2} \quad (6)$$

Duplicate parameter values are removed via the `unique` operation to ensure strict monotonicity. A cubic spline with not-a-knot end conditions is then fitted separately to the $x$ and $y$ sequences:

$$\hat{x}(t) = \text{spline}(t, \{x_i\}, t_{\text{interp}}), \quad \hat{y}(t) = \text{spline}(t, \{y_i\}, t_{\text{interp}}) \quad (7)$$

where $t_{\text{interp}}$ is a uniformly spaced array with density $\rho = 10$ points per original segment. The arc-length parameterization ensures that the spline evolves at an approximately uniform spatial rate along the path.

---

\textbf{Algorithm 3: Arc-Length Parameterized Cubic Spline Smoothing} \\
\hline
\textbf{Input:} $\text{path}[N \times 2]$ input path $[r, c]$, $\rho$ interpolation density (default $10$) \\
\textbf{Output:} $\text{smoothPath}[M \times 2]$ smoothed continuous path $[x, y]$ \\
\hline
1: \quad // Step 1: Grid to continuous coordinates \\
2: \quad $x \gets \text{path}(:,2) - 0.5$; \quad $y \gets \text{path}(:,1) - 0.5$ \\
3: \quad // Step 2: Sparse segment densification \\
4: \quad $\text{newX} \gets [x(1)]$; \quad $\text{newY} \gets [y(1)]$ \\
5: \quad \textbf{for } $i = 2$ \textbf{ to } $\text{length}(x)$ \textbf{ do} \\
6: \quad \quad $\Delta x \gets x(i) - x(i-1)$; \quad $\Delta y \gets y(i) - y(i-1)$ \\
7: \quad \quad $\ell \gets \sqrt{\Delta x^2 + \Delta y^2}$ \\
8: \quad \quad \textbf{if } $\ell > 2$ \textbf{ then} \\
9: \quad \quad \quad $n \gets \lfloor \ell / 2 \rfloor$ \\
10: \quad \quad \quad \textbf{for } $k = 1$ \textbf{ to } $n$ \textbf{ do} \\
11: \quad \quad \quad \quad $\alpha \gets k / (n + 1)$ \\
12: \quad \quad \quad \quad $\text{newX} \gets [\text{newX}, \; x(i-1) + \alpha \cdot \Delta x]$ \\
13: \quad \quad \quad \quad $\text{newY} \gets [\text{newY}, \; y(i-1) + \alpha \cdot \Delta y]$ \\
14: \quad \quad \quad \textbf{end for} \\
15: \quad \quad \textbf{end if} \\
16: \quad \quad $\text{newX} \gets [\text{newX}, \; x(i)]$; \quad $\text{newY} \gets [\text{newY}, \; y(i)]$ \\
17: \quad \textbf{end for} \\
18: \quad // Step 3: Arc-length parameterization and spline fitting \\
19: \quad $t \gets [0; \text{cumsum}(\sqrt{\Delta\text{newX}^2 + \Delta\text{newY}^2})]$ \\
20: \quad Remove duplicate $t$ values (keep first occurrence of each) \\
21: \quad \textbf{if } $\text{length}(t_{\text{unique}}) < 3$ \textbf{ then return } $[\text{newX}', \text{newY}']$ \\
22: \quad $t_{\text{interp}} \gets \text{linspace}(t_{\text{unique}}(1), t_{\text{unique}}(\text{end}), (K-1) \cdot \rho + 1)$ \\
23: \quad $\hat{x} \gets \text{spline}(t_{\text{unique}}, x_{\text{unique}}, t_{\text{interp}})$ \\
24: \quad $\hat{y} \gets \text{spline}(t_{\text{unique}}, y_{\text{unique}}, t_{\text{interp}})$ \\
25: \quad \textbf{return } $[\hat{x}', \hat{y}']$ \\
\hline

---

**Role in the framework.** Smoothing is applied as the final stage of the path-processing pipeline, operating on the already-safe simplified polyline. The resulting continuous trajectory serves directly as the reference path for local planners during robot execution. Since smoothing is a purely geometric operation that does not re-check obstacle clearance, the collision-free guarantee is inherited entirely from the simplification stage that precedes it.

### 4.2.3 Pipeline Integration

The complete path-processing pipeline follows a fixed order:

1. **Global planning**: A* with JPS produces a raw grid path (or standard A* as fallback).
2. **Simplification** (optional, enabled by default for TSP cost computation): corner extraction → greedy + intermediate exploration → minimal safe waypoint subset.
3. **Smoothing** (optional): produces a $C^2$-continuous trajectory in continuous world coordinates.

*[Figure 2: Three-panel comparison showing the same path through pipeline stages. Panel (a): raw JPS grid path. Panel (b): after simplification—corners extracted, redundant waypoints removed. Panel (c): after smoothing—the final continuous $C^2$ curve.]*

The ordering is deliberate and non-interchangeable: simplification must precede smoothing because it operates in grid space where the occupancy grid is defined, establishing the safety guarantee. Smoothing operates in continuous coordinates on the already-verified safe polyline.
