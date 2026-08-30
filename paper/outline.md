# Paper Outline: Obstacle-Aware Multi-Point Traversal Path Planning via Improved A* and Ant Colony Optimization

> **Target**: General-impact SCI journal (English)
> **Paper type**: Algorithmic
> **One-sentence argument**: *In obstacle-constrained multi-point navigation, we show that encoding obstacle information into a TSP cost matrix via an improved A* planner and solving the resulting open TSP with a candidate-list-accelerated MMAS-VND ant colony optimizer produces shorter, safer traversal paths than decoupled or standard approaches, supported by ablation studies isolating each component's contribution on TSPLIB benchmarks and end-to-end validation on self-built obstacle maps, with the boundary that the framework assumes static known obstacles and a 2D grid representation.*

---

## Section Structure

```
1. Introduction
2. Related Work
   2.1 Global Path Planning in Grid Environments
   2.2 Metaheuristics for the Traveling Salesman Problem
   2.3 Multi-Point Traversal with Obstacle Constraints
3. Problem Formulation
   3.1 Grid Map and Obstacle Representation
   3.2 Open TSP with Fixed Start and End Points
   3.3 Optimization Objective
4. Methodology
   4.1 Improved A* Global Path Planner
       4.1.1 Binary Heap Priority Queue
       4.1.2 Adaptive Exponential Weighted Heuristic
       4.1.3 Tie-Breaking Strategy
   4.2 Path Post-Processing Pipeline
       4.2.1 Safety-Distance-Aware Path Simplification
       4.2.2 Arc-Length Parameterized Cubic Spline Smoothing
   4.3 Improved Ant Colony Optimization for Open TSP
       4.3.1 Open TSP Encoding and Pheromone Model
       4.3.2 MMAS Pheromone Update with Dynamic Bounds
       4.3.3 ACS-Style Pseudo-Random Transition Rule
       4.3.4 VND Local Search with Candidate List Acceleration
       4.3.5 S-Curve Progressive Local Search Scheduling
       4.3.6 Adaptive Stopping Criterion
   4.4 Collaborative Optimization Framework
5. Experiments
   5.1 Experimental Setup
   5.2 Global Path Planning Comparison
   5.3 TSP Solver Comparison on TSPLIB Benchmarks
   5.4 Ablation Study of ACO Components
   5.5 End-to-End Validation on Obstacle-Rich Scenarios
6. Conclusion
```

---

## Detailed Section-by-Section Outline

### 1. Introduction

**Paragraph-level job map:**

| Para | Job | Content |
|------|-----|---------|
| 1 | Context | Real-world multi-point traversal applications (autonomous shuttles, logistics, inspection robots). Fundamental challenge: robot must (a) find collision-free paths between points AND (b) decide the optimal visitation order. |
| 2 | Gap | Traditional TSP assumes Euclidean/straight-line distances — no obstacle model. Pure path planning (A*, RRT) handles obstacles but solves only single-pair routing. No existing framework unifies both for general grid-map obstacle environments. |
| 3 | Approach summary | We propose a two-layer framework: (i) an improved A* global planner with adaptive exponential heuristic + path simplification/smoothing pipeline encodes obstacle constraints into pairwise costs; (ii) an improved ACO solver integrating MMAS, candidate-list-accelerated VND, and S-curve progressive local search optimizes the open-TSP visitation order via a virtual node method that converts the open TSP into a closed TSP. |
| 4 | Contributions | Three-layer contributions: (1) Problem-level: obstacle-aware unified framework; (2) Collaboration-level: path-simplification-aware cost matrix for accurate TSP costing; (3) Component-level: AEWH for A*, virtual node method + CL-VND + S-curve for ACO. |
| 5 | Paper structure | Roadmap of sections 2–6. |

---

### 2. Related Work

**2.1 Global Path Planning in Grid Environments**
- Classical: Dijkstra, A* and its weighted/anytime/bidirectional variants
- Sampling-based: RRT, PRM
- Post-processing: path pruning (line-of-sight simplification), spline-based smoothing
- Gap: existing planners lack integration with multi-point ordering

**2.2 Metaheuristics for the Traveling Salesman Problem**
- Exact methods (branch-and-bound, DP) — exponential complexity
- EAs: GA, SA — problem-agnostic but converge slowly on large instances
- ACO evolution: AS → ACS (pseudo-random rule) → MMAS (pheromone bounds)
- VND integration with ACO: prior work uses full O(n²) VND without acceleration
- Gap: standard ACO/VND combinations are computationally heavy for 100+ node open TSP

**2.3 Multi-Point Traversal with Obstacle Constraints**
- Dubins TSP, TSP with neighborhoods, Coverage path planning — specific geometry assumptions
- Two-layer approaches exist (visibility graph + TSP, PRM + TSP) but use simplistic cost models
- Gap: no systematic framework with obstacle-aware cost + optimized solver

---

### 3. Problem Formulation

**3.1 Grid Map and Obstacle Representation**
- n × n occupancy grid: `M(r, c) ∈ {0, 1}`, 1 = obstacle
- Continuous coordinate mapping: `(r, c) → (x = c − 0.5, y = r − 0.5)`
- 8-neighbor connectivity, edge costs: cardinal = 1, diagonal = √2
- [FORMULA: coordinate transformation]

**3.2 Open TSP with Fixed Start and End Points**
- Set of points: `P = {S, T₁, ..., T_K, G}`, total `N = K + 2`
- Index convention: 1 = start S, 2,...,K+1 = targets, N = goal G
- Visit order: `π = (1, π₂, ..., π_{K+1}, N)` where `(π₂, ..., π_{K+1}) ∈ Perm(2, ..., K+1)`
- [FORMULA: permutation encoding]

**3.3 Optimization Objective**
- Pairwise path: `P(i, j) = A*(i, j, M)` — shortest collision-free grid path
- Path length: `L(P(i, j)) = Σ_{edges} cost(e)`
- Total tour cost: `C(π) = Σ_{k=1}^{N−1} L(P(π_k, π_{k+1}))`
- Optimization: `π* = argmin_π C(π)`
- s.t. all segment paths are collision-free
- [FORMULA: complete objective]

---

### 4. Methodology

**4.1 Improved A* Global Path Planner**

**4.1.1 Binary Heap Priority Queue** (Motivation → Design → Advantage)
- Motivation: Standard A* uses linear scan of open set → O(n²) per extraction, dominating runtime on large maps
- Design: Binary min-heap storing `[f, h, row, col]`, with `heapPos` index matrix for O(1) membership check
- Operations: bubbleUp (insert/decrease-key), bubbleDown (extract-min), both O(log m)
- [PSEUDOCODE: bubbleUp and bubbleDown]

**4.1.2 Adaptive Exponential Weighted Heuristic** (Motivation → Design → Advantage)
- Motivation: Fixed-weight A* either over-explores (weight=1) or risks suboptimality (weight>1)
- Design: w(d) = 1 + α · exp(−β · (1 − d/d₀)), f(n) = g(n) + w(d) · h(n)
  - d = Euclidean distance from node n to goal
  - d₀ = start-to-goal distance (normalization)
  - α = 0.3 (max extra weight), β = 3.0 (decay rate)
- Behavior: Far from goal → w ≈ 1+α (greedy exploration); Near goal → w → 1 (precise A*)
- [FORMULA: w(d) function and f(n) evaluation]
- [FIGURE: Weight function curve — w vs. normalized distance d/d₀]

**4.1.3 Tie-Breaking Strategy**
- Primary key: smaller f, Secondary key: smaller h (goal-directional bias)
- [PSEUDOCODE: Complete improved A* algorithm]

---

**4.2 Path Post-Processing Pipeline**

**4.2.1 Safety-Distance-Aware Path Simplification** (Motivation → Design → Advantage)
- Motivation: Raw A* paths contain redundant zigzag waypoints from grid discretization; direct pruning without obstacle checks risks collision
- Design: Greedy back-to-front line-of-sight pruning
  - From anchor point i, scan backward for farthest j where `isLineFree(i, j) = true`
  - `isLineFree`: dense sampling (≥10 pts/grid-unit), exact point-to-cell-boundary distance:
    - dx = max(0, |r_sample − r_cell| − 0.5), dy = max(0, |c_sample − c_cell| − 0.5)
    - dist = √(dx² + dy²), reject if dist < safetyMargin
- [FORMULA: point-to-cell-boundary distance]
- [PSEUDOCODE: SimplifyPath algorithm]
- [FIGURE: Line-of-sight pruning with safety margin illustration]

**4.2.2 Arc-Length Parameterized Cubic Spline Smoothing** (Motivation → Design → Advantage)
- Motivation: Simplified path is still piecewise-linear; non-smooth trajectories cause unnecessary acceleration/deceleration for physical robots
- Design:
  1. Sparse segment densification: insert intermediate points for segments > 2 units (prevents spline corner-cutting)
  2. Arc-length parameterization: t = cumsum(√(Δx² + Δy²))
  3. Cubic spline: spline(t, x) and spline(t, y) → continuous [x, y] output
- [PSEUDOCODE: SmoothPath algorithm]
- [FIGURE: Three-stage pipeline — raw → simplified → smoothed path]

---

**4.3 Improved Ant Colony Optimization for Open TSP**

**4.3.1 Virtual Node Method for Open TSP Encoding**
- Virtual node V = N+1 connected to start (0 cost) and goal (0 cost), BIG penalty to intermediate points
- Extended cost matrix D_ext: (N+1)×(N+1), converts open TSP to closed TSP
- Pheromone matrix τ: (N+1) × (N+1), heuristic η_ij = 1 / d_ij (with BIG for V↔start/goal)
- Path extraction: locate V in best cycle, remove V, orient sequence start→goal
- Tour cost: `C = cost(1, mid₁) + Σ cost(mid_k, mid_{k+1}) + cost(mid_last, N)`
- [FORMULA: tour cost from internal permutation]

**4.3.2 MMAS Pheromone Update with Dynamic Bounds** (Motivation → Design → Advantage)
- Motivation: Standard AS suffers from premature convergence or stagnation
- Design: Two-tier deposit (iteration-best + global-best elite) at weight Q/C
- Dynamic bounds: τ_max = 1/(ρ · C_global), τ_min = τ_max / (N−2)
- [FORMULA: evaporation + deposit + clamp]

**4.3.3 ACS-Style Pseudo-Random Transition Rule** (Motivation → Design → Advantage)
- Motivation: Pure probabilistic selection converges slowly; pure greedy traps early
- Design: With probability q₀ = 0.45, select argmax(τ^α · η^β); otherwise roulette wheel
- [FORMULA: transition probability]

**4.3.4 VND Local Search with Candidate List Acceleration** (Motivation → Design → Advantage)
- Motivation: VND (2-opt → relocate → swap → cycle) on closed cycles improves solution quality but O(n²) per neighborhood is prohibitive for large instances
- Design: Precompute kCand = 9 nearest neighbors per node on extended cost matrix D_ext; prune each VND operator:
  - 2-opt: only if new edge involves candidate pair
  - Relocate: only if moved node is candidate of insertion endpoints
  - Swap: only if swapped nodes are mutual candidates (including wrap-around edge handling)
- Complexity: O(n²) → O(n · k)
- [PSEUDOCODE: Candidate-list-accelerated VND]
- [FLOWCHART: VND neighborhood sequence — Mermaid]

**4.3.5 S-Curve Progressive Local Search Scheduling** (Motivation → Design → Advantage)
- Motivation: Early iterations need exploration; later iterations need exploitation. Fixed LS ratio suboptimal.
- Design: S-curve: optRatio(t) = y_min + (y_max − y_min) · tᵃ / (tᵃ + (1−t)ᵃ)
- LS ant selection: optEliteRatio (70%) elite + remaining random exploration
- [FORMULA: S-curve function]
- [FIGURE: S-curve plot — optRatio vs. iteration]

**4.3.6 Adaptive Stopping Criterion**
- Condition 1: CV(ant costs) < 0.001 → population homogeneity
- Condition 2: best cost unchanged ≥ 70 generations → stagnation
- Guard: both only active after minIter = 30

**4.3.7 Algorithm Summary**
- [PSEUDOCODE: Complete TSP_ACO_v2_4 algorithm]
- [FLOWCHART: Overall ACO flow — Mermaid]

---

**4.4 Collaborative Optimization Framework**
- Two-layer architecture:
  - **Cost matrix layer**: Improved A* + SimplifyPath computes pairwise obstacle-aware costs, cached via containers.Map
  - **Ordering layer**: Improved ACO optimizes permutation on cost matrix
- Key synergy: `enableSimplify` flag → cost matrix computed from simplified path lengths, closer to true continuous travel distance
- [PSEUDOCODE: TSPsolver integration]
- [FLOWCHART: Full system pipeline — Mermaid]
- [FIGURE: System architecture overview diagram]

---

### 5. Experiments

**5.1 Experimental Setup**
- Hardware: [TO FILL — CPU, RAM, MATLAB version]
- Common parameters: [TABLE — AStar_v1 params, SimplifyPath params, SmoothPath params, ACO_v2_4 params]
- Metrics: path length, expanded nodes, computation time, TSP cost, gap-to-optimal(%), iteration count

**5.2 Global Path Planning Comparison**
- Purpose: Validate AStar_v1 + post-processing superiority
- Map 1 (Efficiency test): [DESCRIPTION — open area with sparse obstacles, designed to benchmark search efficiency]
  - Baselines: Standard A*, Dijkstra, RRT, AStar_v1
  - [TABLE: expanded nodes, path length, runtime]
  - [FIGURE: path overlay comparison]
- Map 2 (Simplification/Smoothing test): [DESCRIPTION — corridor with turns, designed to show zigzag reduction]
  - Stages: Raw A* → +SimplifyPath → +SmoothPath
  - [TABLE: waypoint count, path length, smoothness metric]
  - [FIGURE: three-panel pipeline comparison]

**5.3 TSP Solver Comparison on TSPLIB Benchmarks**
- Purpose: Validate ACO_v2_4 against state-of-the-practice
- Dataset: TSPLIB kroA150 (150 cities, optimal = 26524)
- Baselines: Standard ACO, GA, SA
- N runs per algorithm: [TO FILL — suggested 30]
- [TABLE: Best/Worst/Avg/Std/Median, gap%, avg runtime]
- [FIGURE: convergence — cost vs. iteration, median + 95% CI]
- [FIGURE: convergence — cost vs. time, median + 95% CI]
- [FIGURE: cost distribution — histogram + boxplot]

**5.4 Ablation Study of ACO Components**
- Purpose: Isolate each component's contribution
- Dataset: kroA150 (same as 5.3)
- 6 groups:

| Group | Description | Disabled Component |
|-------|-------------|-------------------|
| Full | Complete ACO_v2_4 | None |
| A | No pseudo-random | enablePseudoRandom = 0 (pure roulette) |
| B | No VND | VND removed (pure ACO construction) |
| C | No S-curve | Fixed optRatio = 0.3 (no progressive scheduling) |
| D | No candidate list | kCand = inf (full O(n²) VND) |
| E | No MMAS bounds | Pheromone unconstrained |

- [TABLE: per-group Best/Worst/Avg/Std, iterations, time]
- [FIGURE: overlaid convergence curves (6 groups)]
- [FIGURE: bar chart of final cost with error bars]
- [TABLE: pairwise statistical test (Wilcoxon) vs. Full]

**5.5 End-to-End Validation on Obstacle-Rich Scenarios**
- Purpose: Demonstrate full system on realistic obstacle maps
- Scenario: [TO DESIGN — self-built map, K target points, distinct start/end]
  - [DESCRIPTION: Map size, obstacle layout mimicking buildings/walls, application narrative]
- Method: Full pipeline (AStar_v1 + SimplifyPath-cost + ACO_v2_4 + SmoothPath)
- [TABLE: ordered visitation sequence, per-segment path lengths, total cost]
- [FIGURE: Full map — obstacles, numbered targets, visit order arrows, smoothed paths]
- [TABLE: computation time breakdown — cost matrix vs. TSP solve vs. post-processing]

---

### 6. Conclusion

- Summary of three-layer framework and key mechanisms
- Recap of experimental findings: (1) AEWH reduces expanded nodes; (2) safety-aware simplification + spline smoothing produces safe, smooth paths; (3) CL-VND + S-curve + MMAS synergy achieves near-optimal TSP solutions efficiently
- Practical implications for real-world multi-point navigation
- Limitations: static known obstacles, 2D grid, fixed candidate-list size
- Future work: dynamic obstacles + online replanning, multi-robot extension, learning-based parameter adaptation

---

## Writing Sequence (per user instruction)

1. Outline approval ← **CURRENT**
2. Section 4 (Methodology) — core, most complex
3. Section 3 (Problem Formulation) — mathematical foundation
4. Section 5 (Experiments) — experiment descriptions
5. Section 1 (Introduction) — coherence with full paper
6. Section 2 (Related Work) — after intro context
7. Section 6 (Conclusion) — final synthesis
8. Abstract — after all sections finalized

---

## Figures & Tables Checklist (for user to prepare)

| ID | Description | Section |
|----|-------------|---------|
| Fig-01 | Weight function w(d) curve | 4.1.2 |
| Fig-02 | Line-of-sight pruning + safety margin illustration | 4.2.1 |
| Fig-03 | Three-stage path pipeline (raw → simplified → smoothed) | 4.2.2 |
| Fig-04 | S-curve optRatio plot | 4.3.5 |
| Fig-05 | System architecture overview | 4.4 |
| Fig-06 | Path comparison on Map 1 (A*/Dijkstra/RRT/AStar_v1) | 5.2 |
| Fig-07 | Three-panel pipeline comparison on Map 2 | 5.2 |
| Fig-08 | TSP convergence curves (cost vs. iteration) | 5.3 |
| Fig-09 | TSP convergence curves (cost vs. time) | 5.3 |
| Fig-10 | TSP cost distribution (histogram + boxplot) | 5.3 |
| Fig-11 | Ablation convergence overlaid curves | 5.4 |
| Fig-12 | Ablation bar chart | 5.4 |
| Fig-13 | End-to-end obstacle scenario full map | 5.5 |
| Tab-01 | Complete parameter settings | 5.1 |
| Tab-02 | Path planning comparison results | 5.2 |
| Tab-03 | TSP solver comparison results | 5.3 |
| Tab-04 | Ablation per-group results | 5.4 |
| Tab-05 | Ablation statistical tests | 5.4 |
| Tab-06 | End-to-end scenario results | 5.5 |

---

## User Action Items

- [ ] Review and approve this outline
- [ ] Prepare Map 1 (efficiency comparison) for Section 5.2
- [ ] Prepare Map 2 (simplification/smoothing validation) for Section 5.2
- [ ] Run kroA150 comparison experiments (ACO_v2_4 vs baseline ACO/GA/SA, N≥30 runs)
- [ ] Run all 6 ablation groups on kroA150
- [ ] Design and build obstacle-rich end-to-end scenario map for Section 5.5
- [ ] Provide hardware/software specs for Section 5.1
- [ ] Confirm parameter values for the full parameters table
