# 4. Methodology

## 4.1 Improved A* Global Path Planner

The first layer of our framework computes pairwise obstacle-aware path costs between every pair of points in the visitation set. We adopt A* as the base planner for its optimality guarantee under an admissible heuristic, and introduce two algorithmic improvements: an adaptive exponential weighted heuristic that trades optimality for search speed in a principled, distance-dependent manner, and a tie-breaking strategy that biases expansion toward the goal direction. The open set is implemented as a binary min-heap, reducing the per-iteration extraction cost from $\mathcal{O}(m)$ to $\mathcal{O}(\log m)$ where $m$ is the open-set size; this is a standard data-structure optimization and is not discussed further.

### 4.1.1 Adaptive Exponential Weighted Heuristic

**Motivation.** Standard A* uses the evaluation function $f(n) = g(n) + h(n)$, where $g(n)$ is the accumulated path cost from the start node to node $n$, and $h(n)$ is a heuristic estimate of the remaining cost from $n$ to the goal. When $h(n)$ is admissible (i.e., never overestimates the true cost), A* guarantees optimality. Weighted A* introduces a constant inflation factor $\varepsilon > 1$ such that $f(n) = g(n) + \varepsilon \cdot h(n)$, trading optimality for speed by making the search greedier. However, a constant weight is coarse-grained: when the agent is far from the goal, aggressive weighting helps escape large open regions quickly; when the agent nears the goal, precision becomes more important than speed and the weight should approach unity. No single constant value optimally serves both regimes.

**Mechanism.** We propose an Adaptive Exponential Weighted Heuristic (AEWH) whose weight decays smoothly with the remaining distance to the goal:

$$w(d) = 1 + \alpha \cdot \exp\left(-\beta \cdot \left(1 - \frac{d}{d_0}\right)\right) \quad (1)$$

where $d = h(n) = \sqrt{(r_n - r_{\text{goal}})^2 + (c_n - c_{\text{goal}})^2}$ is the Euclidean distance from the current node $n$ to the goal, and $d_0 = \sqrt{(r_{\text{start}} - r_{\text{goal}})^2 + (c_{\text{start}} - c_{\text{goal}})^2}$ is the start-to-goal distance, serving as the normalization baseline. The parameters $\alpha$ (default $0.3$) and $\beta$ (default $3.0$) control the maximum extra weight and the decay rate, respectively.

The evaluation function becomes:

$$f(n) = g(n) + w(d) \cdot h(n) \quad (2)$$

The weight function exhibits dual-regime behavior:

- **Far from goal** ($d \approx d_0$): the exponential term evaluates to $e^0 = 1$, giving $w \approx 1 + \alpha = 1.3$. The search is maximally greedy, rapidly expanding toward the goal along promising directions.
- **Near goal** ($d \to 0$): the exponential term approaches $e^{-\beta} \approx 0.05$, giving $w \to 1 + 0.3 \times 0.05 \approx 1.015$, essentially recovering the standard admissible heuristic and preserving near-optimality in the critical final approach.

The exponential form in Eq. (1) has two desirable properties. First, the rate of weight decay is proportional to the distance itself, making the transition from greedy to precise self-adapting without explicit switching logic. Second, the use of the normalized distance $(1 - d/d_0)$ makes the function scale-invariant: the same $\alpha$ and $\beta$ values apply across maps of different physical dimensions.

*[Figure 1: Weight function $w(d)$ plotted against normalized distance $d/d_0$, showing the smooth decay from $1+\alpha$ at $d/d_0 = 1$ to approximately $1$ at $d/d_0 \to 0$.]*

### 4.1.2 Tie-Breaking Strategy

**Motivation.** When multiple nodes in the open set share identical $f$-values, the standard A* selection order is determined by implementation-specific iteration order, which is essentially arbitrary. However, among nodes with equal $f$, those with smaller $h$ have already incurred greater actual cost $g = f - h$ and lie closer to the goal. Preferring these nodes biases the search toward the goal direction without affecting the optimality guarantee.

**Mechanism.** When two nodes $n_1$ and $n_2$ share the same $f$-value, the node with a smaller $h$-value is expanded preferentially. The comparison rule for heap ordering is defined as:

$$\text{compare}(n_1, n_2) = \begin{cases}
    n_1, & f(n_1) < f(n_2) \\
    n_2, & f(n_1) > f(n_2) \\
    n_1, & f(n_1) = f(n_2) \land h(n_1) < h(n_2) \\
    n_2, & f(n_1) = f(n_2) \land h(n_1) > h(n_2)
\end{cases} \quad (3)$$

The comparison is embedded directly in the binary heap's bubble-up and bubble-down operations.

**Physical meaning.** From the evaluation function $f(n) = g(n) + w \cdot h(n)$, when two nodes have identical $f$-values, a larger $g$ implies a smaller $h$. A larger $g$ means the node lies farther from the start, while a smaller $h$ means it is closer to the goal. Preferring such nodes biases the search toward the goal direction, reducing futile exploration in regions far from the goal. Since $h$ is already computed at node expansion time, this tie-breaking incurs no additional computational overhead.

---

## 4.2 Path Post-Processing Pipeline

The raw grid path produced by A* (or any grid-based planner) contains two artifacts inherited from grid discretization: (i) redundant colinear waypoints and staircase-shaped zigzag segments that inflate the waypoint count without reducing path length, and (ii) piecewise-linear segments connected by sharp turns that are kinematically inefficient for physical robots. We address these with a two-stage post-processing pipeline: safety-distance-aware path simplification followed by arc-length parameterized cubic spline smoothing.

### 4.2.1 Safety-Distance-Aware Path Simplification

**Motivation.** Grid-constrained A* paths contain intermediate waypoints at every grid cell transition. Many of these waypoints are colinear with their neighbors and can be removed without altering the geometric trace of the path. More importantly, corridors and open areas produce staircase patterns—sequences of orthogonal segments that a single straight line could replace, potentially shortening the effective path. However, naive line-of-sight pruning that does not consult the obstacle map risks connecting two waypoints through an obstacle corner, violating the collision-free guarantee. A robust simplification must verify obstacle clearance for every pruned segment.

**Mechanism.** The simplification algorithm follows a greedy back-to-front line-of-sight strategy. From the current anchor waypoint $i$, it scans candidate waypoints $j$ from the path end backward to $i+1$. The first (and therefore farthest) candidate $j$ for which the straight-line segment is collision-free is accepted—its waypoint is appended to the output and becomes the new anchor. This is repeated until the anchor reaches the path end. Because the scan processes candidates in descending order of distance, each accepted jump removes the maximum possible number of intermediate waypoints, producing a minimal waypoint subset of the original path.

The core of the algorithm is the $\text{IsLineFree}$ check, which verifies that every point along the segment maintains at least a safety margin $d_{\text{safe}}$ from all obstacle cells. For a segment spanning $\ell = \max(|r_1 - r_2|, |c_1 - c_2|)$ grid units, we sample $N_s = \max(\lceil 10\ell \rceil, 30)$ equally spaced points. At each sample point $\mathbf{p} = (r, c)$ in continuous coordinates, all grid cells within a search radius of $d_{\max} = \lceil d_{\text{safe}} + 0.5 \rceil$ are examined.

The key technical detail is the exact point-to-cell-boundary distance calculation, illustrated in Figure 2. A grid cell at $(r_{\text{cell}}, c_{\text{cell}})$ occupies the axis-aligned rectangle $[r_{\text{cell}} - 0.5, r_{\text{cell}} + 0.5] \times [c_{\text{cell}} - 0.5, c_{\text{cell}} + 0.5]$. The distance from a continuous sample point to the boundary of this rectangle is:

$$d_x = \max\left(0,\; |r - r_{\text{cell}}| - 0.5\right) \quad (4)$$

$$d_y = \max\left(0,\; |c - c_{\text{cell}}| - 0.5\right) \quad (5)$$

$$d(\mathbf{p}, \text{cell}) = \sqrt{d_x^2 + d_y^2} \quad (6)$$

The $\max(0, \cdot)$ operator handles the case where the sample point lies within the cell's horizontal or vertical extent along one axis. For example, if the sample point is vertically aligned with the cell but horizontally outside, $d_y = 0$ and the distance reduces to the horizontal penetration $d_x$. If any sample point is within $d_{\text{safe}}$ of any occupied cell, the segment is rejected.

*[Figure 2: Illustration of the exact point-to-cell-boundary distance computation. The diagram shows a sample point $\mathbf{p}$ near a grid cell, with $d_x$ and $d_y$ labeled as the horizontal and vertical distances to the cell boundary. The shaded region indicates the safety margin $d_{\text{safe}}$ around the segment.]*

The pseudocode for the simplification algorithm is given in Algorithm 1.

---

\textbf{Algorithm 1: Safety-Distance-Aware Path Simplification} \\
\hline
\textbf{Input:} $\text{path}[N \times 2]$ original grid path $[r, c]$, $\text{occGrid}$ occupancy grid ($n \times n$), $n$ map size, $d_{\text{safe}}$ safety margin \\
\textbf{Output:} $\text{simplePath}[M \times 2]$ simplified path ($M \leq N$) \\
\hline
1: \quad \textbf{if } $N \leq 2$ \textbf{ then return } $\text{path}$ \\
2: \quad $\text{simplePath} \gets [\text{path}(1,:)]$; \quad $i \gets 1$ \\
3: \quad \textbf{while } $i < N$ \textbf{ do} \\
4: \quad \quad \textbf{for } $j = N$ \textbf{ down to } $i+1$ \textbf{ do} \\
5: \quad \quad \quad \textbf{if } $\text{IsLineFree}(\text{path}(i,:), \text{path}(j,:), \text{occGrid}, n, d_{\text{safe}})$ \textbf{ then} \\
6: \quad \quad \quad \quad $\text{simplePath} \gets [\text{simplePath}; \; \text{path}(j,:)]$ \\
7: \quad \quad \quad \quad $i \gets j$; \quad \textbf{break} \\
8: \quad \quad \quad \textbf{end if} \\
9: \quad \quad \textbf{end for} \\
10: \quad \textbf{end while} \\
11: \quad \textbf{return } $\text{simplePath}$ \\
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

**Role in the framework.** In the TSP cost matrix computation (Section 4.4), an `enableSimplify` flag controls whether pairwise costs derive from raw grid paths or simplified paths. Using simplified-path Euclidean costs produces a cost matrix closer to the true continuous-path length, improving the TSP solver's ability to discriminate between alternative visitation orders. For multi-robot scenarios, $d_{\text{safe}}$ is set to the robot radius plus $0.2$ grid units, tying the safety margin directly to the physical robot footprint.

### 4.2.2 Arc-Length Parameterized Cubic Spline Smoothing

**Motivation.** The simplified path, while obstacle-safe and waypoint-minimal, remains piecewise-linear with discontinuous first derivatives at waypoint transitions. A physical robot following such a trajectory must decelerate and re-accelerate at each corner, increasing energy consumption and travel time. Cubic spline interpolation produces a $C^2$-continuous curve suitable for smooth trajectory tracking by local planners. However, directly fitting a cubic spline to sparsely distributed waypoints can cause the spline to overshoot between distant points—an artifact of the Runge phenomenon in polynomial interpolation—potentially clipping obstacle corners even when all waypoints are safe.

**Mechanism.** The smoothing procedure operates in three steps.

**Step 1 — Grid-to-continuous coordinate conversion.** Grid-indexed waypoints $[r, c]$ are mapped to continuous world coordinates by centering each cell at its geometric center:

$$x = c - 0.5, \quad y = r - 0.5 \quad (7)$$

**Step 2 — Sparse segment densification.** Before fitting the spline, we scan the waypoint sequence for consecutive pairs whose Euclidean distance exceeds $2$ units. For each such pair, $\lfloor \text{dist} / 2 \rfloor$ intermediate points are linearly interpolated along the segment. This densification provides the spline with sufficient knots to stay close to the intended polyline, preventing oscillation artifacts without altering the geometric path. The threshold of $2$ grid units is chosen because it corresponds to the maximum segment length over which a cubic spline with not-a-knot end conditions remains well-behaved on grid paths.

**Step 3 — Arc-length parameterized cubic spline.** The cumulative chordal distance along the (densified) waypoint sequence defines a monotonic parameter:

$$t_1 = 0, \quad t_k = \sum_{i=2}^{k} \sqrt{(x_i - x_{i-1})^2 + (y_i - y_{i-1})^2} \quad (8)$$

Duplicate parameter values arising from coincident or near-coincident waypoints are removed via the `unique` operation to ensure strict monotonicity. If fewer than $3$ unique parameter values remain, the path is returned without spline fitting (linear interpolation serves as the fallback).

A cubic spline with not-a-knot end conditions is then fitted separately to the $x$ and $y$ sequences as functions of $t$:

$$\hat{x}(t) = \text{spline}(t, \{x_i\}, t_{\text{interp}}), \quad \hat{y}(t) = \text{spline}(t, \{y_i\}, t_{\text{interp}}) \quad (9)$$

where $t_{\text{interp}}$ is a uniformly spaced array with density $\rho = 10$ points per original segment, i.e., $\text{length}(t_{\text{interp}}) = (K - 1) \cdot \rho + 1$ for $K$ unique waypoints. The arc-length parameterization ensures that the spline evolves at an approximately uniform spatial rate along the path, producing a physically meaningful reference trajectory where equal increments in $t$ correspond to equal distances along the path.

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

**Role in the framework.** Smoothing is applied as the final stage of the path-processing pipeline, operating on the already-safe simplified polyline (or on the raw A* path when simplification is disabled). The resulting continuous trajectory serves directly as the reference path for local planners during robot execution. Since smoothing is a purely geometric operation that does not re-check obstacle clearance, the collision-free guarantee is inherited entirely from the simplification stage that precedes it. This separation of concerns—simplification handles safety, smoothing handles kinematics—keeps each stage focused and verifiable independently.

### 4.2.3 Pipeline Integration

The complete path-processing pipeline follows a fixed order:

1. **Global planning**: A* (or alternative planner) produces a raw grid path.
2. **Simplification** (optional, enabled by default for TSP cost computation): reduces waypoints while preserving obstacle clearance with safety margin $d_{\text{safe}}$.
3. **Smoothing** (optional): produces a $C^2$-continuous trajectory in continuous world coordinates.

*[Figure 3: Three-panel comparison showing the same path through pipeline stages. Panel (a): raw A* grid path with staircase artifacts. Panel (b): after simplification—redundant waypoints removed, line-of-sight connections visible. Panel (c): after smoothing—the final continuous $C^2$ curve ready for robot execution.]*

The ordering is deliberate and non-interchangeable: simplification must precede smoothing because it operates in grid space where the occupancy grid is defined, establishing the safety guarantee. Smoothing operates in continuous coordinates on the already-verified safe polyline. Reversing this order would require expensive continuous-space collision checking, as the spline might deviate from the safe polyline without additional constraints.
