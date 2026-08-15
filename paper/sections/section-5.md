# 5. Experiments

This section presents a comprehensive experimental evaluation of the proposed framework. We conduct four sets of experiments: (i) a comparison of the improved A* planner and path post-processing pipeline against baseline global planners on manually constructed grid maps, (ii) a comparison of the improved ACO solver against baseline TSP metaheuristics on the TSPLIB kroA150 benchmark, (iii) an ablation study isolating the contribution of each ACO component, and (iv) an end-to-end validation on a self-built obstacle-rich multi-point traversal scenario.

## 5.1 Experimental Setup

All experiments were conducted on a computer with [TO FILL: CPU model, RAM, operating system]. The algorithms were implemented in MATLAB [TO FILL: version].

**Parameter settings.** Unless otherwise specified in the ablation study (Section 5.4), the following parameter values were used throughout all experiments:

*[Table 1: Complete parameter settings for all algorithmic modules.]*

| Module | Parameter | Symbol | Value |
|--------|-----------|--------|-------|
| A* (AEWH) | Max extra weight | $\alpha$ | 0.3 |
| A* (AEWH) | Decay rate | $\beta$ | 3.0 |
| Path simplification | Safety margin | $d_{\text{safe}}$ | 0.4 |
| Path smoothing | Interpolation density | $\rho$ | 10 |
| ACO | Number of ants | $n_{\text{ants}}$ | 40 |
| ACO | Maximum iterations | $n_{\text{iter}}$ | 800 |
| ACO | Pheromone weight | $\alpha_{\text{ACO}}$ | 1 |
| ACO | Heuristic weight | $\beta_{\text{ACO}}$ | 2.0 |
| ACO | Evaporation rate | $\rho_{\text{ACO}}$ | 0.25 |
| ACO | Deposit constant | $Q$ | 225 |
| ACO | Greedy probability | $q_0$ | 0.45 |
| ACO | Candidate list size | $k$ | 50 |
| ACO | LS transition interval | $[x_L, x_R]$ | $[0, 100]$ |
| ACO | LS ratio range | $[y_{\min}, y_{\max}]$ | $[0, 0.3]$ |
| ACO | S-curve steepness | $a$ | 2.0 |
| ACO | Elite ratio | $r_{\text{elite}}$ | 0.6 |
| ACO | Random cap | $r_{\text{cap}}$ | 0.15 |
| ACO | CV threshold | — | 0.001 |
| ACO | Stagnation limit | — | 150 |
| ACO | Min iterations | — | 30 |

**Evaluation metrics.** The following metrics are reported across experiments:

- **Path length**: Total Euclidean length of the planned path (grid units). For grid paths, the Manhattan-plus-diagonal metric (Eq. 2) is used.
- **Expanded nodes**: Number of nodes extracted from the open set during A* search. Measures search efficiency independently of hardware.
- **Computation time**: Wall-clock runtime in seconds.
- **TSP cost**: Total cost of the ordered tour (Eq. 8).
- **Gap to optimal**: $(C - C_{\text{opt}}) / C_{\text{opt}} \times 100\%$, where $C_{\text{opt}}$ is the known optimal TSP cost for benchmark instances.
- **Convergence iterations**: Number of iterations until the ACO terminates (by adaptive stopping or reaching the maximum).
- **Success rate**: Fraction of runs where a feasible collision-free tour was found.

## 5.2 Global Path Planning Comparison

### 5.2.1 Map 1 — Search Efficiency Comparison

**Objective.** Compare the search efficiency of the improved A* planner (AStar_v1) against baseline global planners: standard A*, Dijkstra, and RRT.

**Map design.** *[Description: A manually constructed grid map of size [TO FILL: e.g., 50×50], containing [TO FILL: sparse obstacle layout — describe the key features, e.g., scattered rectangular obstacles creating multiple homotopy classes of paths between distant points]. The map is designed to test search efficiency in open areas with sparse obstacles, where the adaptive weighting and tie-breaking of AStar_v1 have the greatest impact on expanded node count.]*

*[Figure 6: Map 1 layout — occupancy grid with start and goal points marked. Obstacles shown in black, free space in white.]*

**Procedure.** For each planner, we compute the shortest path between [TO FILL: number of point pairs, e.g., 10 pre-selected start-goal pairs] distributed across the map. Each planner is run once per pair (deterministic algorithms). For RRT, which is stochastic, [TO FILL: e.g., 10 runs per pair with mean ± std reported].

**Results.** *[Table 2: Path planning comparison on Map 1.]*

| Planner | Expanded Nodes (mean ± std) | Path Length (mean ± std) | Time (ms, mean ± std) |
|---------|----------------------------|-------------------------|----------------------|
| Dijkstra | [TO FILL] | [TO FILL] | [TO FILL] |
| Standard A* | [TO FILL] | [TO FILL] | [TO FILL] |
| RRT | [TO FILL] | [TO FILL] | [TO FILL] |
| AStar_v1 (ours) | [TO FILL] | [TO FILL] | [TO FILL] |

*[Figure 7: Side-by-side path comparison on Map 1. Panel (a): Dijkstra — extensive exploration, high expanded count. Panel (b): Standard A* — directionally focused but uniform weight. Panel (c): RRT — fast but meandering paths. Panel (d): AStar_v1 — compact exploration footprint with AEWH directing search toward goal.]*

**Analysis.** *[TO FILL after data collection: Discuss relative performance. Expected: AStar_v1 expands significantly fewer nodes than Dijkstra and standard A* due to the combination of adaptive weighting (Section 4.1.1) and tie-breaking (Section 4.1.2), while producing paths of comparable length. RRT produces paths faster but with higher path length variance.]*

### 5.2.2 Map 2 — Path Simplification and Smoothing Validation

**Objective.** Quantify the contribution of the path post-processing pipeline (SimplifyPath + SmoothPath, Section 4.2) in reducing waypoint count and improving path smoothness.

**Map design.** *[Description: A manually constructed grid map of size [TO FILL], containing [TO FILL: corridor-like environment with turns — describe features such as L-shaped corridors, narrow passages, and open areas that produce significant staircase artifacts in raw A* paths]. The map is designed to demonstrate the staircase-removal and smoothing effects of the pipeline.]*

*[Figure 8: Map 2 layout with an example start-goal pair. The raw A* path exhibits pronounced staircase patterns along the diagonal corridor.]*

**Procedure.** We run AStar_v1 between [TO FILL: number of point pairs] on Map 2. For each resulting path, we apply three variants: (i) raw A* path only (no post-processing), (ii) A* + SimplifyPath, and (iii) A* + SimplifyPath + SmoothPath. We report the number of waypoints, total path length, and a smoothness metric (average absolute curvature) for each variant.

**Results.** *[Table 3: Path post-processing comparison on Map 2.]*

| Variant | Waypoints (mean ± std) | Path Length (mean ± std) | Avg. Curvature (mean ± std) |
|---------|------------------------|-------------------------|----------------------------|
| Raw A* | [TO FILL] | [TO FILL] | [TO FILL] |
| + SimplifyPath | [TO FILL] | [TO FILL] | [TO FILL] |
| + SimplifyPath + SmoothPath | [TO FILL] | [TO FILL] | [TO FILL] |

*[Figure 9: Three-panel comparison of the same path through the pipeline stages. Panel (a): raw A* grid path with staircase artifacts. Panel (b): after SimplifyPath — waypoint count reduced, line-of-sight connections visible. Panel (c): after SmoothPath — smooth continuous $C^2$ curve.]*

**Analysis.** *[TO FILL after data collection: Expected: SimplifyPath reduces waypoint count by [XX]% while slightly reducing path length. SmoothPath produces the lowest curvature while preserving the geometric trace of the simplified path.]*

## 5.3 TSP Solver Comparison on TSPLIB Benchmarks

**Objective.** Evaluate the improved ACO solver (TSP_ACO_v2_3) against three baseline TSP metaheuristics—standard Ant Colony Optimization (ACO), Genetic Algorithm (GA), and Simulated Annealing (SA)—on a standard TSPLIB benchmark instance.

**Dataset.** The TSPLIB kroA150 instance, consisting of 150 cities with Euclidean 2D coordinates. The cost matrix is computed as the Euclidean distance between cities, yielding a known optimal tour cost of $26{,}524$. To model the open TSP with fixed start and end, city 1 serves as both start and goal: the cost matrix is augmented to size $151 \times 151$, with a copy of city 1 appended as the destination.

**Baseline implementations.** The standard ACO baseline is the basic Ant System with uniform initialization, roulette-wheel selection (no pseudo-random rule), uniform evaporation and deposit, and no local search, MMAS bounds, or adaptive stopping. The GA baseline uses order-based crossover (OX) and tournament selection. The SA baseline uses 2-opt neighborhood moves with an exponential cooling schedule.

**Procedure.** Each solver is run independently for [TO FILL: e.g., 30] trials with different random seeds. All solvers are given the same cost matrix. For the stochastic baselines (GA, SA), the population size and iteration budget are set to match the ACO's computational budget where feasible. For ACO_v2_3, the adaptive stopping mechanism (Section 4.3.6) may terminate the search before the maximum iteration limit; for the baselines, the search runs for a fixed budget equivalent to the average ACO_v2_3 runtime.

**Results.** *[Table 4: TSP solver comparison on kroA150.]*

| Solver | Best Cost | Worst Cost | Avg Cost ± Std | Median | Gap to Optimal (%) | Avg Time (s) |
|--------|-----------|------------|----------------|--------|--------------------|--------------|
| Standard ACO | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| GA | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| SA | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| ACO_v2_3 (ours) | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |

*[Figure 10: Convergence curves — Best Cost vs. Iteration for all solvers. Median curve (solid line) with 95% confidence interval (shaded band). A horizontal dashed line marks the known optimal (26,524). For ACO_v2_3, the curve shows only iterations until adaptive termination; for baselines, the full budget is shown.]*

*[Figure 11: Convergence curves — Best Cost vs. Time for all solvers, with the same median/CI/optimal conventions as Figure 10. This view accounts for per-iteration runtime differences between solvers.]*

*[Figure 12: Cost distribution for each solver. Upper panel: histogram of best costs across 30 trials. Lower panel: boxplot (horizontal) with best-cost scatter overlaid.]*

**Analysis.** *[TO FILL after data collection: Expected: ACO_v2_3 achieves lower mean cost and smaller variance than all baselines, with a gap to optimal of [XX]% vs. [XX]% (best baseline). The convergence curves show that ACO_v2_3 converges faster in both iteration count (due to pseudo-random rule and elite deposit) and wall-clock time (despite the overhead of VND local search, which is offset by candidate list acceleration). The cost distribution is tighter for ACO_v2_3 (lower standard deviation), indicating more reliable performance across random seeds.]*

## 5.4 Ablation Study of ACO Components

**Objective.** Isolate the individual contribution of each mechanism in the improved ACO solver by systematically disabling one component at a time.

**Dataset.** The same TSPLIB kroA150 benchmark used in Section 5.3, ensuring direct comparability between the full algorithm and each ablated variant.

**Ablation groups.** Six experimental conditions are tested, as summarized in Table 5. Group 1 is the complete algorithm (all mechanisms enabled). Groups A through E each disable exactly one mechanism, with all other settings held constant at the values in Table 1.

*[Table 5: Ablation study groups and their configurations.]*

| Group | Description | Mechanism Disabled | Configuration Change |
|-------|-------------|--------------------|----------------------|
| 1 | Complete | None | Full ACO_v2_3 (Table 1) |
| A | No pseudo-random | ACS pseudo-random rule (§4.3.3) | `enablePseudoRandom = 0` (pure roulette) |
| B | No VND local search | VND (§4.3.4) | VND calls removed; ants use construction-only tours |
| C | No S-curve scheduling | Progressive LS scheduling (§4.3.5) | Fixed `optRatio = 0.3` (constant 30% LS) |
| D | No candidate list | Candidate list acceleration (§4.3.4) | `kCand = N` (full O$(K^2)$ VND, no pruning) |
| E | No MMAS bounds | MMAS pheromone limits (§4.3.2) | Pheromone bounds disabled (v2_3AS variant) |

**Procedure.** Each group is run for [TO FILL: e.g., 30] independent trials with different random seeds. The performance of each group is compared to Group 1 (complete) in terms of solution quality, convergence speed, and computation time.

**Results.** *[Table 6: Ablation study results on kroA150.]*

| Group | Best | Worst | Avg ± Std | Gap vs. Full (%) | Avg Iters | Avg Time (s) |
|-------|------|-------|-----------|-------------------|-----------|--------------|
| 1 (Full) | [TO FILL] | [TO FILL] | [TO FILL] | — | [TO FILL] | [TO FILL] |
| A | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| B | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| C | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| D | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |
| E | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] | [TO FILL] |

*[Figure 13: Overlaid convergence curves (median best cost vs. iteration) for all 6 ablation groups. Each curve is the median across 30 trials. The complete algorithm (Group 1) is highlighted with a thicker line.]*

*[Figure 14: Bar chart comparing average final cost across ablation groups, with error bars showing ±1 standard deviation. Groups are ordered by decreasing performance. A horizontal dashed line marks the known optimal (26,524).]*

*[Table 7: Pairwise statistical comparison between each ablation group and Group 1 (complete). Wilcoxon rank-sum test; p-values and effect sizes (Cliff's delta) reported.]*

| Comparison | p-value | Cliff's $\delta$ | Significant ($\alpha = 0.05$)? |
|------------|---------|------------------|-------------------------------|
| Full vs. A | [TO FILL] | [TO FILL] | [TO FILL] |
| Full vs. B | [TO FILL] | [TO FILL] | [TO FILL] |
| Full vs. C | [TO FILL] | [TO FILL] | [TO FILL] |
| Full vs. D | [TO FILL] | [TO FILL] | [TO FILL] |
| Full vs. E | [TO FILL] | [TO FILL] | [TO FILL] |

**Analysis.** *[TO FILL after data collection: Discuss which component contributes most to solution quality (likely VND local search, Group B, showing the largest cost increase), which contributes most to speed (likely candidate list, Group D, showing longer runtimes without quality loss), and the relative importance of the S-curve (Group C), pseudo-random rule (Group A), and MMAS bounds (Group E).]*

## 5.5 End-to-End Validation on Obstacle-Rich Scenarios

**Objective.** Demonstrate the complete framework—improved A* + path post-processing + improved ACO + SmoothPath—on a realistic obstacle-rich map with multiple target points and distinct start and end locations. This experiment validates the claim that the proposed framework solves the obstacle-aware multi-point traversal problem as an integrated system.

**Scenario design.** *[Description: A self-built grid map of size [TO FILL: e.g., 80×80], containing complex obstacles [TO FILL: e.g., irregularly shaped and distributed obstacles representing buildings, walls, and barriers, creating a realistic indoor/outdoor navigation environment]. The mission specifies a start point S [coordinates], K = [TO FILL: e.g., 10–15] target points distributed across the map, and a goal point G [coordinates] distinct from S.]*

*[Figure 15: Obstacle-rich scenario map. Obstacles shown in black. Start point marked with a green circle, goal point with a red square, and the K target points numbered in order. The planned optimal tour is shown as a colored polyline with directional arrows indicating the visit sequence.]*

**Procedure.** The full pipeline is executed: (1) the cost matrix is constructed by running AStar_v1 on all $\mathcal{O}(N^2)$ point pairs with `enableSimplify = true`; (2) ACO_v2_3 solves for the optimal visit order; (3) each segment path is simplified and smoothed via the post-processing pipeline (Section 4.2). The total computation time is broken down by stage.

**Results.** *[Table 8: End-to-end scenario results.]*

| Metric | Value |
|--------|-------|
| Total cost | [TO FILL] |
| Number of targets $K$ | [TO FILL] |
| Cost matrix construction time | [TO FILL] s |
| TSP solving time | [TO FILL] s |
| Post-processing time | [TO FILL] s |
| Total wall-clock time | [TO FILL] s |

*[Table 9: Optimal visit order with per-segment path lengths.]*

| Segment | From | To | Path Length | Simplified? |
|---------|------|----|-------------|-------------|
| 1 | Start | Target $T_a$ | [TO FILL] | Yes |
| 2 | Target $T_a$ | Target $T_b$ | [TO FILL] | Yes |
| $\vdots$ | $\vdots$ | $\vdots$ | $\vdots$ | $\vdots$ |
| $K+1$ | Target $T_z$ | Goal | [TO FILL] | Yes |

*[Figure 16: Sequential snapshots of the planned traversal showing the robot's ordered path through the obstacle field. Each panel highlights the current segment in a distinct color, with previously traversed segments shown in gray. The smooth continuous curves produced by the post-processing pipeline are clearly visible.]*

**Analysis.** *[TO FILL after data collection: Discuss the quality of the resulting tour in the context of the obstacle layout. Highlight cases where the TSP solver selected a non-obvious ordering that avoids costly detours around obstacles, demonstrating the value of the obstacle-aware cost matrix over a naive Euclidean-distance-based ordering. Comment on the practical feasibility of the total computation time for offline mission planning.]*
