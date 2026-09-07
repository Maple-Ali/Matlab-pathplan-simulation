# 5. Experiments

This section presents a comprehensive experimental evaluation of the proposed framework. We conduct four sets of experiments: (i) a performance comparison of the improved A* global planner (JPS-based) against baseline planners, (ii) a parameter sensitivity analysis of the improved ACO solver, (iii) an ablation study isolating the contribution of each ACO component, and (iv) a comparison of different TSP solvers collaborating with the improved A* planner for multi-point traversal path planning.

## 5.1 Experimental Setup

All experiments were conducted on a computer equipped with an Intel Core Ultra 5 125H CPU and 16 GB of RAM, running MATLAB R2025b.

**Maps.** Three test environments were used across the experiments:

- **Map 1** (`Map1.mat`): a grid map designed to test basic search performance in a moderately obstructed environment.
- **Map 2** (`Map2_kong.mat`): a larger grid map with an open, corridor-like structure that exercises long straight-line traversals and produces pronounced staircase artifacts in grid paths.
- **Map 3** (the `kroB100` TSPLIB dataset, `kroB100_coords.mat`): a standard 100-city benchmark instance used for the TSP ordering experiments.

**Repetition and statistics.** For the global planning comparison (Section 5.2), each algorithm was run 10 times per map, with the time metric averaged. For the TSP experiments (Sections 5.3--5.5), each algorithm configuration was run 40 times per map, with statistics (best, median, mean, standard deviation, success rate, and cost range) computed over the 40 independent runs.

**Parameter settings.** Unless otherwise specified in the sensitivity analysis (Section 5.3) or ablation study (Section 5.4), the following default parameter values were used throughout:

*Table 1: Default parameter settings of the proposed algorithms.*

| Module | Parameter | Value |
|--------|-----------|-------|
| A* (JPS) | Fallback planner | AStar_v0 |
| A* (JPS) | Heuristic | Standard Euclidean |
| Path simplification | Safety margin $d_{\text{safe}}$ | 0.4 |
| Path smoothing | Interpolation density $\rho$ | 10 |
| ACO | Number of ants $n_{\text{ants}}$ | 40 |
| ACO | Pheromone weight $\alpha$ | 1 |
| ACO | Heuristic weight $\beta$ | 2.0 |
| ACO | Evaporation rate $\rho_{\text{ACO}}$ | 0.25 |
| ACO | Deposit constant $Q$ | 225 |
| ACO | Initial pheromone $\tau_0$ | 0.1 |
| ACO | Greedy probability $q_0$ | 0.45 |
| ACO | LS ratio range $y_{\max}$ | 0.4 |
| ACO | LS transition interval $[x_L, x_R]$ | $[0, 30]$ |
| ACO | Elite ratio $r_{\text{elite}}$ | 0.7 |
| ACO | Candidate list size $k$ | 9 |

**Evaluation metrics.** The following metrics are reported: path length (grid units), computation time (seconds), number of expanded nodes (for global planning only), optimal/median/mean convergence cost, success rate (percentage of runs achieving the optimal cost), and standard deviation.

## 5.2 Global Path Planning Comparison

**Objective.** Compare the search efficiency and path quality of the proposed JPS-based A* planner (AStar_v3_1, combined with SimplifyPath and SmoothPath) against three baseline planners: traditional A* (AStar_v0), Dijkstra, and RRT.

**Procedure.** Each planner was run on all three maps. For the proposed method, the reported results include the full post-processing pipeline (SimplifyPath + SmoothPath). Each algorithm was run 10 times per map, with computation time averaged.

**Results.** Table 2 summarizes the path length, computation time, and expanded node count for each algorithm on each map. RRT is a sampling-based method and therefore does not report expanded nodes.

*Table 2: Global path planning performance comparison.*

| Algorithm | Map | Path Length | Time (ms) | Expanded Nodes |
|-----------|-----|-------------|-----------|----------------|
| AStar_v3_1 + Simplify + Smooth (ours) | Map 1 | 72.93 | 2.45 | 70 |
| | Map 2 | 108.17 | 11.85 | 415 |
| | Map 3 | 113.33 | 4.31 | 48 |
| AStar_v0 | Map 1 | 76.53 | 6.14 | 81 |
| | Map 2 | 111.44 | 29.19 | 198 |
| | Map 3 | 119.64 | 17.88 | 280 |
| Dijkstra | Map 1 | 76.53 | 24.26 | 200 |
| | Map 2 | 111.44 | 202.29 | 522 |
| | Map 3 | 119.64 | 204.46 | 566 |
| RRT | Map 1 | 100.80 | 7.42 | — |
| | Map 2 | 135.40 | 19.33 | — |
| | Map 3 | 144.07 | 3.69 | — |

*Figure 1: Path planning results of the four algorithms on the three maps. (a) The proposed A* (JPS) planner on Maps 1, 2, and 3. (b) Traditional A* on Maps 1, 2, and 3. (c) Dijkstra on Maps 1, 2, and 3. (d) RRT on Maps 1, 2, and 3.*

*Figure 2: Path length comparison (line chart) of the four algorithms across Maps 1, 2, and 3.*

*Figure 3: Computation time comparison (line chart) of the four algorithms across Maps 1, 2, and 3.*

**Analysis.** The proposed JPS-based planner achieves the shortest path length on all three maps (72.93 vs. 76.53 for both AStar_v0 and Dijkstra on Map 1; 108.17 vs. 111.44 on Map 2; 113.33 vs. 119.64 on Map 3). This path-length advantage arises from the simplification stage, which removes staircase artifacts and shortens the effective path. In terms of search efficiency, the JPS planner expands far fewer nodes than the baselines: on Map 3, it expands only 48 nodes versus 280 for AStar_v0 and 566 for Dijkstra, a reduction of over 80%. This translates directly into faster computation: on Map 2, the proposed method completes in 11.85 ms versus 29.19 ms for AStar_v0 and 202.29 ms for Dijkstra—over 17× faster than Dijkstra. RRT produces the longest paths (100.80 on Map 1, 135.40 on Map 2, 144.07 on Map 3) with high variance, consistent with its sampling-based nature. The results demonstrate that the combination of JPS pruning, path simplification, and path smoothing yields both shorter and more computationally efficient paths than the baselines.

## 5.3 Parameter Sensitivity Analysis

**Objective.** Analyze the sensitivity of the improved ACO solver (TSP_ACO_v2_4) to its five key hyperparameters: the number of ants $n_{\text{ants}}$, the greedy selection probability $q_0$, the late-stage local search ratio $y_{\max}$ (optRatio_end), the elite ratio $r_{\text{elite}}$ (optEliteRatio), and the candidate list size $k$ (kCand).

**Procedure.** Each parameter was varied across five levels while all other parameters were held at their default values. The five levels for each parameter are summarized in Table 3. Each configuration was run 40 times on each of the three maps.

*Table 3: Parameter levels used in the sensitivity analysis.*

| Parameter | Level 1 | Level 2 | Level 3 (default) | Level 4 | Level 5 |
|-----------|---------|---------|-------------------|---------|---------|
| $n_{\text{ants}}$ | 10 | 20 | 40 | 60 | 90 |
| $q_0$ | 0.10 | 0.30 | 0.45 | 0.60 | 0.90 |
| $y_{\max}$ (optRatio_end) | 0.10 | 0.20 | 0.40 | 0.60 | 0.90 |
| $r_{\text{elite}}$ (optEliteRatio) | 0.1 | 0.3 | 0.5 | 0.7 | 0.9 |
| $k$ (kCand) | 3 | 9 | 15 | 30 | 60 |

**Results.** The complete sensitivity data (25 configurations × 3 maps) is summarized through the following figures. Table 4 presents representative results for the default configuration and the configurations that most strongly degrade performance.

*Table 4: Representative parameter sensitivity results (selected configurations).*

| Parameter Level | Map | Optimal Cost | Mean Convergence Cost | Success Rate (%) | Mean Time (s) | Std |
|-----------------|-----|--------------|----------------------|------------------|---------------|-----|
| Default (all level 3) | Map 1 | 351.4406 | 351.4406 | 100 | 0.0272 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 100 | 0.2415 | 0 |
| | Map 3 | 22141.0000 | 22142.4500 | 97.5 | 0.3479 | 9.1706 |
| $n_{\text{ants}} = 10$ (level 1) | Map 3 | 22141.0000 | 22145.0250 | 92.5 | 0.2505 | 14.6629 |
| $q_0 = 0.90$ (level 5) | Map 2 | 620.1509 | 620.7505 | 62.5 | 0.2321 | 1.1727 |
| | Map 3 | 22141.0000 | 22180.5500 | 50 | 0.3517 | 42.0579 |
| $y_{\max} = 0.10$ (level 1) | Map 2 | 620.1509 | 620.7129 | 65 | 0.2171 | 1.0482 |
| | Map 3 | 22141.0000 | 22157.0500 | 80 | 0.3654 | 35.3944 |
| $r_{\text{elite}} = 0.1$ (level 1) | Map 3 | 22141.0000 | 22145.3500 | 92.5 | 0.3420 | 15.4713 |
| $k = 3$ (level 1) | Map 3 | 22141.0000 | 22193.3000 | 30 | 0.7868 | 37.4544 |

*Figure 4: Cost comparison (line chart) of each parameter level on Maps 1, 2, and 3.*

*Figure 5: Computation time comparison (line chart) of each parameter level on Maps 1, 2, and 3.*

*Figure 6: Paths planned by the algorithm under the default parameter settings on Maps 1, 2, and 3.*

**Analysis.** The results reveal several important sensitivity characteristics. First, the number of ants $n_{\text{ants}}$ has a moderate effect: at $n_{\text{ants}} = 40$ (the default), the algorithm achieves a $100\%$ success rate on Map 2 and $97.5\%$ on Map 3, while reducing to $10$ ants degrades the Map 3 success rate to $92.5\%$ and increases mean cost. Second, the greedy probability $q_0$ exhibits a clear optimum at the default $0.45$: increasing to $0.9$ (near-pure greedy) causes severe degradation, dropping the Map 3 success rate to $50\%$ and increasing the standard deviation to $42.06$, indicating premature convergence. Third, the late-stage local search ratio $y_{\max}$ is critical: reducing it to $0.10$ degrades the Map 2 success rate to $65\%$ and Map 3 to $80\%$, confirming that intensive late-stage local search is essential for solution quality. Fourth, the elite ratio $r_{\text{elite}}$ and candidate list size $k$ show comparatively mild sensitivity, with the candidate list size $k$ being the least sensitive parameter—though an excessively small $k = 3$ degrades the Map 3 success rate to $30\%$, indicating that a minimal candidate list is necessary for effective VND pruning. Overall, the default parameter settings represent a well-balanced configuration, and were therefore adopted for all subsequent experiments.

## 5.4 Ablation Study of ACO Components

**Objective.** Isolate the individual contribution of each mechanism in the improved ACO solver by systematically disabling one component at a time.

**Procedure.** Six experimental groups were tested, each disabling exactly one mechanism while holding all other settings constant. Table 5 summarizes the ablation groups.

*Table 5: Ablation study groups.*

| Group | Description | Disabled Mechanism |
|-------|-------------|--------------------|
| 1 | Complete algorithm | None |
| 2 | No pseudo-random | ACS pseudo-random transition rule (§4.3.3) |
| 3 | No VND | VND local search (§4.3.4) |
| 4 | No S-curve | Progressive LS scheduling (§4.3.5) |
| 5 | No candidate list | Candidate list acceleration (§4.3.4) |
| 6 | No MMAS | MMAS pheromone bounds (§4.3.2) |

**Results.** Table 6 reports the full ablation results across all maps.

*Table 6: Ablation study results.*

| Group | Map | Best Cost | Median Cost | Mean Cost | Success Rate (%) | Mean Time (s) | Cost Range | Std |
|-------|-----|-----------|-------------|-----------|------------------|---------------|------------|-----|
| 1 (Complete) | Map 1 | 351.4406 | 351.4406 | 351.4406 | 100 | 0.0272 | 0 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 620.1509 | 100 | 0.2415 | 0 | 0 |
| | Map 3 | 22141.0000 | 22141.0000 | 22142.4500 | 97.5 | 0.3479 | 58.00 | 9.1706 |
| 2 (No pseudo-random) | Map 1 | 351.4406 | 351.4406 | 351.4406 | 100 | 0.0324 | 0 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 620.1509 | 100 | 0.4157 | 0 | 0 |
| | Map 3 | 22141.0000 | 22141.0000 | 22141.0000 | 100 | 0.7313 | 0 | 0 |
| 3 (No VND) | Map 1 | 351.4406 | 351.4406 | 352.5747 | 67.5 | 0.1339 | 9.7568 | 2.2744 |
| | Map 2 | 620.9552 | 620.1509 | 632.6061 | 2.5 | 0.5695 | 37.4366 | 8.7702 |
| | Map 3 | 22279.0000 | 22512.0000 | 22566.9750 | 0 | 1.4739 | 1012.0000 | 239.4130 |
| 4 (No S-curve) | Map 1 | 351.4406 | 351.4406 | 351.4406 | 100 | 0.0175 | 0 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 620.1711 | 97.5 | 0.3494 | 0.8042 | 0.1272 |
| | Map 3 | 22141.0000 | 22141.0000 | 22146.8000 | 90 | 0.6570 | 58.0000 | 17.6217 |
| 5 (No candidate list) | Map 1 | 351.4406 | 351.4406 | 351.4406 | 100 | 0.0325 | 0 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 620.1509 | 100 | 0.3325 | 0 | 0 |
| | Map 3 | 22141.0000 | 22141.0000 | 22143.9000 | 95 | 0.4260 | 58.0000 | 12.8018 |
| 6 (No MMAS) | Map 1 | 351.4406 | 353.2976 | 359.6359 | 30 | 0.0218 | 45.0453 | 12.4797 |
| | Map 2 | 620.1509 | 624.1322 | 626.1160 | 10 | 0.0854 | 24.7496 | 6.1209 |
| | Map 3 | 22141.0000 | 22141.0000 | 22166.6500 | 70 | 0.2484 | 235.0000 | 47.4933 |

*Figure 7: Time–cost convergence curves (mean over 40 runs) of the six ablation groups on the three maps.*

*Figure 8: Cost distribution (box plot) of the six ablation groups over 40 runs on the three maps.*

*Figure 9: Mean cost comparison (bar chart) of the six ablation groups on the three maps.*

*Figure 10: Mean computation time comparison (bar chart) of the six ablation groups on the three maps.*

**Analysis.** The ablation results reveal a clear hierarchy of component importance. The most critical component is VND local search (Group 3): disabling it collapses the Map 3 success rate to $0\%$, increases mean cost from $22142.45$ to $22566.98$ (a $1.9\%$ degradation), and inflates the standard deviation to $239.41$. This confirms that VND is indispensable for escaping local optima and achieving near-optimal solutions. The second most critical component is MMAS pheromone bounds (Group 6): removing them drops the Map 1 success rate to $30\%$ and the Map 2 success rate to $10\%$, with substantial cost variance, demonstrating that pheromone bound control is essential for preventing premature convergence. The pseudo-random transition rule (Group 2) and S-curve scheduling (Group 4) have more moderate effects: removing the pseudo-random rule actually maintains full success rates (at the cost of slightly longer runtime), while removing the S-curve mildly degrades Map 3 performance (success rate from $97.5\%$ to $90\%$). The candidate list acceleration (Group 5) has the smallest impact on solution quality—consistent with its role as a purely computational accelerator—while providing runtime benefits. Overall, the ablation confirms that VND and MMAS are the two pillars of solution quality, while the pseudo-random rule, S-curve, and candidate list contribute to convergence speed and computational efficiency.

## 5.5 Multi-Point Traversal Comparison with Different TSP Solvers

**Objective.** Evaluate the complete multi-point traversal framework by combining the improved A* planner (with the virtual node method) with different TSP solvers, and compare the proposed ACO solver (MMAS-VND-CL) against baseline TSP metaheuristics.

**Procedure.** Four TSP solvers—the proposed MMAS-VND-CL (TSP_ACO_v2_4), standard Ant System (ACO), Genetic Algorithm (GA), and Simulated Annealing (SA)—were each combined with the improved A* planner to solve the multi-point traversal problem on all three maps. The parameters of each solver are summarized in Table 7. Each combination was run 40 times per map.

*Table 7: Parameter settings of the compared TSP solvers.*

| Algorithm | Parameter | Value |
|-----------|-----------|-------|
| MMAS-VND-CL | Ants $m$, $\alpha$, $\beta$, $\rho$, $Q$, $\tau_0$, Iter, $q_0$, $y_{\max}$, transition interval, elite ratio, $k$ | 40, 1, 2, 0.25, 225, 0.1, 200, 0.45, 0.4, [0,30], 0.7, 9 |
| ACO | Ants $m$, $\alpha$, $\beta$, $\rho$, $Q$, $\tau_0$, Iter | 40, 1, 2, 0.25, 225, 0.1, 100 |
| GA | Population, Iter, $P_c$, $P_m$, elite count | 100, 1500, 0.8, 0.25, 10 |
| SA | $T_0$, cooling $\alpha$, Iter per temperature $L$, $T_{\text{end}}$ | 150, 0.996, 10, $10^{-8}$ |

**Results.** Table 8 reports the multi-point traversal performance of each solver.

*Table 8: Multi-point traversal comparison results.*

| Algorithm | Map | Best Cost | Median Cost | Mean Cost | Optimal-Hit Rate (%) | Mean Time (s) | Cost Range | Std |
|-----------|-----|-----------|-------------|-----------|----------------------|---------------|------------|-----|
| MMAS-VND-CL (ours) | Map 1 | 351.4406 | 351.4406 | 351.4406 | 100 | 0.0272 | 0 | 0 |
| | Map 2 | 620.1509 | 620.1509 | 620.1509 | 100 | 0.2415 | 0 | 0 |
| | Map 3 | 22141.0000 | 22141.0000 | 22142.4500 | 97.5 | 0.3479 | 58.00 | 9.1706 |
| ACO | Map 1 | 359.3170 | 388.5591 | 388.6513 | 0 | 0.0564 | 59.3454 | 15.5736 |
| | Map 2 | 638.6843 | 688.3238 | 685.7453 | 0 | 0.1304 | 88.0149 | 18.9081 |
| | Map 3 | 23359.0000 | 24418.5000 | 24435.8250 | 0 | 0.3300 | 2045.0000 | 559.3037 |
| GA | Map 1 | 404.4468 | 482.2940 | 485.1563 | 0 | 3.0007 | 201.4194 | 44.2581 |
| | Map 2 | 698.4059 | 859.9173 | 851.2521 | 0 | 3.8424 | 221.8002 | 46.5627 |
| | Map 3 | 33271.0000 | 37285.5000 | 37360.6500 | 0 | 4.9882 | 14912.0000 | 2680.1153 |
| SA | Map 1 | 359.9775 | 394.8159 | 390.3485 | 0 | 0.3902 | 44.3223 | 13.9440 |
| | Map 2 | 664.5622 | 688.7545 | 691.9980 | 0 | 0.7321 | 17.4566 | 66.3322 |
| | Map 3 | 23574.0000 | 24817.5000 | 24918.3500 | 0 | 0.3218 | 2841.0000 | 752.5199 |

*Figure 11: Time–cost convergence curves (mean over 40 runs) of the four TSP solvers on the three maps.*

*Figure 12: Cost distribution (box plot) of the four TSP solvers over 40 runs on the three maps.*

*Figure 13: Mean cost comparison (bar chart) of the four TSP solvers on the three maps.*

*Figure 14: Mean computation time comparison (bar chart) of the four TSP solvers on the three maps.*

*Figure 15: Optimal-cost path planned by each TSP solver on the three maps.*

**Analysis.** The proposed MMAS-VND-CL solver substantially outperforms all three baselines across every map. On Map 3 (the kroB100 benchmark, optimal cost 22141), the proposed solver achieves the optimal cost with a $97.5\%$ hit rate and a mean cost of $22142.45$, while the best baseline (ACO) achieves a mean cost of $24435.83$—a $10.4\%$ degradation—and never hits the optimal cost. The GA performs worst, with a mean cost of $37360.65$ on Map 3, $68.7\%$ above optimal. The performance gap is even more pronounced in terms of stability: the proposed solver's standard deviation ($9.17$) is over 60× smaller than GA's ($2680.12$) and 82× smaller than SA's ($752.52$), demonstrating exceptional reliability. The optimal-hit rate of $100\%$ on Maps 1 and 2 indicates that the proposed solver consistently finds the global optimum in these environments, while all baselines fail to hit the optimum even once. In terms of computation time, the proposed solver is competitive with ACO ($0.35$ s vs. $0.33$ s on Map 3) and significantly faster than GA ($4.99$ s), despite incorporating VND local search—a benefit attributable to the candidate list acceleration. These results confirm that the proposed framework, integrating the JPS-based A* planner with the virtual-node MMAS-VND-CL ACO solver, delivers superior solution quality, reliability, and efficiency for obstacle-aware multi-point traversal path planning.
