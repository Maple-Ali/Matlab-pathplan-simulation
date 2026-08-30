## 4.3 Improved Ant Colony Optimization for Open TSP

The second layer of our framework solves the open Traveling Salesman Problem on the cost matrix produced by the global planning layer. Given $N$ points consisting of a fixed start $S$, $K$ intermediate target points $\{T_1, \dots, T_K\}$, and a fixed goal $G$, the objective is to find the permutation of the $K$ targets that minimizes the total path cost. We build on the Ant Colony Optimization (ACO) metaheuristic and introduce six integrated mechanisms: (i) a virtual node method that converts the open TSP into a standard closed TSP, enabling full exploitation of closed-TSP search operators, (ii) Max-Min Ant System (MMAS) pheromone management with dynamically updated bounds, (iii) an Ant Colony System (ACS)-style pseudo-random proportional transition rule balancing exploration and exploitation, (iv) Variable Neighborhood Descent (VND) local search accelerated by a $k$-nearest-neighbor candidate list, (v) an S-curve progressive scheduling strategy that gradually shifts the search from exploration-dominated to exploitation-dominated, and (vi) an adaptive stopping criterion.

### 4.3.1 Virtual Node Method for Open TSP Encoding

**Motivation.** In a standard closed TSP, the solution is a Hamiltonian cycle and any city can serve as the starting point. In our problem, the start $S$ and goal $G$ are physically distinct locations determined by the mission specification—for instance, a shuttle's departure depot and its final destination. These two points must appear at fixed positions (first and last) in every candidate solution. A direct encoding that treats all $N$ points as symmetrically permutable would generate invalid tours mixing start/goal into the middle sequence, while a boundary-fixed encoding that restricts start and goal to the ends of the tour excludes them from the pheromone model and local search, limiting the algorithm's ability to optimize the full solution.

**Mechanism.** We adopt the virtual node (dummy node) method to convert the open TSP into a standard closed TSP. We introduce a virtual node $V = N + 1$ and construct an extended cost matrix $\mathbf{D}_{\text{ext}}$ of size $(N+1) \times (N+1)$:

$$\mathbf{D}_{\text{ext}}(i, j) = \begin{cases}
    0, & \{i, j\} = \{V, 1\} \text{ (virtual node ↔ start)} \\
    0, & \{i, j\} = \{V, N\} \text{ (virtual node ↔ goal)} \\
    \text{BIG}, & i = V \text{ or } j = V, \text{ and } \{i, j\} \cap \{1, N\} = \varnothing \\
    \mathbf{D}(i, j), & \text{otherwise}
\end{cases} \quad (10)$$

where $\text{BIG} = 2(N+1) \cdot \max_{i,j} \mathbf{D}(i,j)$ is a penalty value strictly larger than any feasible tour cost, ensuring the virtual node is never directly connected to any intermediate target. The zero-cost edges between $V$ and the start/goal make the virtual node effectively "invisible" to the path cost, while the penalty edges to intermediate points force the virtual node to be adjacent only to the start and goal in any near-optimal cycle.

The ACO solver operates on the extended cost matrix $\mathbf{D}_{\text{ext}}$ as a standard closed TSP over $N+1$ nodes: all ants construct complete Hamiltonian cycles visiting all $N+1$ nodes (including the virtual node), and the pheromone matrix $\boldsymbol{\tau}$ covers the full $(N+1) \times (N+1)$ space. After the solver converges, the open-path visit order is extracted from the best cycle by:

1. Locating the virtual node $V$ in the cycle.
2. Removing $V$ and reading the remaining $N$ nodes in cycle order.
3. Orienting the sequence so that the start ($1$) appears first and the goal ($N$) appears last.

This conversion has three advantages over a boundary-fixed encoding: (a) the start and goal nodes participate fully in the pheromone model and local search, rather than being excluded as fixed boundaries; (b) all standard closed-TSP operators (2-opt, relocate, swap) apply uniformly without special boundary handling; and (c) the cost evaluation is a simple cycle summation over $\mathbf{D}_{\text{ext}}$, avoiding the need for separate boundary-cost terms.

The heuristic visibility matrix $\boldsymbol{\eta}$ is defined over the extended space. For pairs involving the virtual node, the zero-cost edges to start/goal are assigned a large heuristic value $\eta_{V,1} = \eta_{1,V} = \eta_{V,N} = \eta_{N,V} = \text{BIG}$, strongly attracting the virtual node toward its required neighbors. For other pairs:

$$\eta_{ij} = \begin{cases}
    1 / \mathbf{D}_{\text{ext}}(i,j), & \mathbf{D}_{\text{ext}}(i,j) > 0 \text{ and } \mathbf{D}_{\text{ext}}(i,j) < \infty \\
    0, & \text{otherwise}
\end{cases} \quad (11)$$

The cost of a closed cycle $\boldsymbol{\sigma} = (\sigma_1, \sigma_2, \dots, \sigma_{N+1})$ is:

$$C(\boldsymbol{\sigma}) = \sum_{k=1}^{N} \mathbf{D}_{\text{ext}}(\sigma_k, \sigma_{k+1}) + \mathbf{D}_{\text{ext}}(\sigma_{N+1}, \sigma_1) \quad (12)$$

### 4.3.2 MMAS Pheromone Update with Dynamic Bounds

**Motivation.** In the basic Ant System, all ants deposit pheromone on their complete tours, which can cause the search to prematurely concentrate on suboptimal solutions when a few ants happen to find moderately good paths early. Conversely, excessive evaporation without bounds can cause pheromone values to approach zero on all but the dominant edges, trapping the colony in local optima. The Max-Min Ant System (MMAS) addresses both issues by (a) limiting pheromone to an explicit interval $[\tau_{\min}, \tau_{\max}]$, preventing both domination and extinction, and (b) focusing deposit on the best-performing solutions.

**Mechanism.** At each iteration, all pheromone values first undergo evaporation:

$$\tau_{ij} \leftarrow (1 - \rho) \cdot \tau_{ij} \quad \forall i, j \in \{1, \dots, N+1\} \quad (13)$$

where $\rho = 0.25$ is the evaporation rate. After evaporation, pheromone is deposited on the edges of two solution levels:

**Iteration-best deposit.** The best ant of the current generation deposits pheromone on all edges of its cycle:

$$\tau_{ij} \leftarrow \tau_{ij} + \frac{Q}{C_{\text{iter}}} \quad \forall (i, j) \in \text{edges}(\boldsymbol{\sigma}_{\text{iter-best}}) \quad (14)$$

**Global-best elite deposit.** The best cycle found since the start of the search also receives a deposit, reinforcing the globally dominant solution:

$$\tau_{ij} \leftarrow \tau_{ij} + \frac{Q}{C_{\text{global}}} \quad \forall (i, j) \in \text{edges}(\boldsymbol{\sigma}_{\text{global-best}}) \quad (15)$$

where $Q = 225$ is the deposit constant and $C$ denotes the cycle cost. The deposit amount is inversely proportional to cycle cost, so shorter cycles deposit stronger pheromone. Both deposit levels are applied to the symmetric counterparts $(j, i)$ simultaneously, maintaining an undirected pheromone model.

After deposit, all pheromone values are clamped to the interval $[\tau_{\min}, \tau_{\max}]$, whose bounds are updated dynamically based on the current global best:

$$\tau_{\max} = \frac{1}{\rho \cdot C_{\text{global}}} \quad (16)$$

$$\tau_{\min} = \frac{\tau_{\max}}{N+1} = \frac{1}{\rho \cdot (N+1) \cdot C_{\text{global}}} \quad (17)$$

with a hard floor of $\tau_{\min} \geq 10^{-6}$ to prevent complete pheromone extinction on rarely-used edges. The dynamic update of bounds is crucial: as the search discovers better cycles (decreasing $C_{\text{global}}$), $\tau_{\max}$ increases, allowing the algorithm to express greater confidence in its current best solution. Conversely, the ratio $\tau_{\min}/\tau_{\max} = 1/(N+1)$ maintains a constant relative gap between the minimum and maximum pheromone, preserving a baseline exploration probability for all edges regardless of the absolute scale.

The initial pheromone matrix is set to a uniform value $\tau_0 = 0.1$, representing a neutral prior before any search experience is accumulated.

### 4.3.3 ACS-Style Pseudo-Random Transition Rule

**Motivation.** In the standard Ant System, each ant selects its next node through pure probabilistic roulette-wheel selection over all unvisited candidates. This method is exploration-heavy and can be slow to converge on structured problems where the distance heuristic provides reliable guidance. The Ant Colony System (ACS) addresses this with a pseudo-random proportional rule: with a controlled probability, the ant exploits greedily by selecting the best candidate; otherwise, it explores via the standard probabilistic mechanism. This balances the need for convergence (exploitation) with the need to avoid local optima (exploration).

**Mechanism.** When an ant at its current node $u$ selects the next node among the set of unvisited candidates $\mathcal{U}$, it first computes a score for each candidate:

$$\text{score}(v) = (\tau_{uv})^{\alpha} \cdot (\eta_{uv})^{\beta} \quad \forall v \in \mathcal{U} \quad (18)$$

where $\alpha = 1$ and $\beta = 2.0$ are the pheromone and heuristic weight exponents, respectively. The transition decision then follows:

$$v_{\text{next}} = \begin{cases}
    \arg\max_{v \in \mathcal{U}} \; \text{score}(v), & \text{with probability } q_0 \\
    \text{roulette}(\mathcal{U}, \text{score}), & \text{with probability } 1 - q_0
\end{cases} \quad (19)$$

where $q_0 = 0.45$ is the exploitation probability. In the exploitation case, the ant deterministically chooses the candidate with the highest score—equivalent to a greedy best-first step. In the exploration case, the ant performs roulette-wheel selection, where each candidate $v$ is selected with probability $\text{score}(v) / \sum_{u \in \mathcal{U}} \text{score}(u)$. If all scores are zero (all remaining nodes are unreachable), a random unvisited candidate is selected as a fallback.

The value $q_0 = 0.45$ means that on average, $45\%$ of construction steps use greedy selection, accelerating convergence, while $55\%$ maintain diversity through random exploration. This ratio is motivated by the structured nature of obstacle-constrained cost matrices, where pairwise distances often exhibit spatial locality and the heuristic $\eta$ carries reliable information.

### 4.3.4 VND Local Search with Candidate List Acceleration

**Motivation.** The cycles constructed by individual ants, while guided by pheromone and heuristic information, are not guaranteed to be locally optimal. Applying local search to the best ants of each generation refines solutions by systematically exploring neighboring permutations. Variable Neighborhood Descent (VND) applies multiple neighborhood structures in sequence, each capturing a different type of cycle transformation, and cycles until no structure yields further improvement. However, standard VND examines the full $\mathcal{O}(K^2)$ neighborhood for each operator, which becomes a computational bottleneck for large problem instances. Furthermore, most of these examined moves are fruitless—a candidate pair of distant nodes is unlikely to produce an improving 2-opt swap.

**Mechanism.** Our VND operates on closed cycles over all $N+1$ nodes (including the virtual node) and employs three neighborhood operators applied in fixed order:

1. **2-opt**: Reverses a contiguous subsequence of the cycle by replacing edges $(u, u^+)$ and $(v, v^+)$ with $(u, v)$ and $(u^+, v^+)$. This removes path crossings in Euclidean-like cost structures.

2. **Relocate**: Extracts a single node from its current position and reinserts it at a different position in the cycle, shifting the intervening nodes. This corrects localized misorderings where a node belongs in a different region of the cycle.

3. **Swap**: Exchanges the positions of two nodes while keeping all other nodes fixed. This handles cases where two nodes are assigned to each other's optimal positions.

The three operators are applied in a First-Improvement (FI) strategy within a VND loop: 2-opt is applied until no improvement remains, then Relocate, then Swap. If any operator produces an improvement, the loop resets to 2-opt and repeats. The search terminates when all three operators consecutively fail to improve the solution.

**Candidate list acceleration.** The key efficiency improvement is the use of a precomputed candidate list to prune unpromising neighborhood evaluations. For each of the $N+1$ nodes in the extended problem, we precompute the $k = 9$ nearest neighbors based on the extended cost matrix $\mathbf{D}_{\text{ext}}$ and store them as a boolean matrix $\mathbf{C}$ where $\mathbf{C}(i, j) = \text{true}$ if node $j$ is among the $k$ nearest neighbors of node $i$. The three VND operators are then pruned as follows:

- **2-opt pruning**: A candidate 2-opt move replacing edges $(u, u^+)$ and $(v, v^+)$ with $(u, v)$ and $(u^+, v^+)$ is evaluated only if $\mathbf{C}(u, v) = \text{true}$ or $\mathbf{C}(u^+, v^+) = \text{true}$—that is, at least one of the proposed new edges connects a pair of candidate neighbors.

- **Relocate pruning**: Moving node $v$ to insert after node $p$ (between $p$ and $p^+$) is evaluated only if $\mathbf{C}(p, v) = \text{true}$ or $\mathbf{C}(v, p^+) = \text{true}$—the relocated node must be a candidate neighbor of at least one of its new adjacent nodes.

- **Swap pruning**: Swapping nodes $a$ and $b$ is evaluated only if $\mathbf{C}(a, b) = \text{true}$—the two swapped nodes must be mutual candidate neighbors.

The pruning is conservative by design: a move is skipped only when neither of its new edges involves a candidate pair, which is a strong but not absolute signal that the move is unlikely to improve the cycle. The small candidate list size ($k = 9$) is chosen to aggressively filter evaluations while retaining the most promising local moves; this is effective because the extended cost matrix exhibits strong spatial structure, and only nearby nodes are likely to participate in improving edge swaps.

**Complexity.** Each VND operator without pruning examines $\mathcal{O}((N+1)^2)$ candidate moves. With candidate list pruning, each node participates only in $\mathcal{O}(k)$ neighborhood checks, reducing the per-operator complexity to $\mathcal{O}((N+1) \cdot k)$. For typical problem sizes ($N \approx 100$, $k = 9$), this represents a roughly $10\times$ reduction; the gap widens quadratically with $N$.

---

\textbf{Algorithm 4: Candidate-List-Accelerated VND Local Search} \\
\hline
\textbf{Input:} $\boldsymbol{\sigma}[1 \dots N+1]$ closed cycle, $\mathbf{D}_{\text{ext}}[(N+1) \times (N+1)]$ extended cost matrix, $\mathbf{C}$ candidate boolean matrix \\
\textbf{Output:} improved $\boldsymbol{\sigma}$, its cost \\
\hline
1: \quad $\text{cost} \gets \textsc{CycleCost}(\boldsymbol{\sigma}, \mathbf{D}_{\text{ext}})$ \\
2: \quad \textbf{repeat} \\
3: \quad \quad $\text{improved} \gets \text{false}$ \\
4: \quad \quad $[\boldsymbol{\sigma}, \text{cost}, \text{ok}] \gets \textsc{TwoOptClosed}(\boldsymbol{\sigma}, \text{cost}, \mathbf{C})$ \quad // Candidate-pruned 2-opt \\
5: \quad \quad $\text{improved} \gets \text{improved} \lor \text{ok}$ \\
6: \quad \quad $[\boldsymbol{\sigma}, \text{cost}, \text{ok}] \gets \textsc{RelocateClosed}(\boldsymbol{\sigma}, \text{cost}, \mathbf{C})$ \quad // Candidate-pruned relocate \\
7: \quad \quad $\text{improved} \gets \text{improved} \lor \text{ok}$ \\
8: \quad \quad $[\boldsymbol{\sigma}, \text{cost}, \text{ok}] \gets \textsc{SwapClosed}(\boldsymbol{\sigma}, \text{cost}, \mathbf{C})$ \quad // Candidate-pruned swap \\
9: \quad \quad $\text{improved} \gets \text{improved} \lor \text{ok}$ \\
10: \quad \textbf{until } $\neg \text{improved}$ \\
11: \quad $\text{cost} \gets \textsc{CycleCost}(\boldsymbol{\sigma}, \mathbf{D}_{\text{ext}})$ \quad // Final precise recalculation \\
12: \quad \textbf{return } $\boldsymbol{\sigma}, \text{cost}$ \\
\hline

---

The neighborhood operators themselves (TwoOptClosed, RelocateClosed, SwapClosed) each use a First-Improvement strategy—the first move found that reduces the cycle cost is immediately accepted, and the operator returns. The closed-cycle topology requires special handling for the swap operator: when swapping two adjacent nodes across the cycle-closing edge (i.e., the last and first elements of the cycle), the standard linear adjacency logic must account for the wrap-around connection.

*[Flowchart: VND neighborhood search sequence — showing the 2-opt → Relocate → Swap → cycle loop with candidate list pruning at each operator.]*

```mermaid
flowchart TB
    A["Input: initial cycle"] --> B["2-opt with CL pruning"]
    B -->|"improvement found"| B
    B -->|"no improvement"| C["Relocate with CL pruning"]
    C -->|"improvement found"| B
    C -->|"no improvement"| D["Swap with CL pruning"]
    D -->|"improvement found"| B
    D -->|"no improvement"| E["Output: locally optimal cycle"]
```

### 4.3.5 S-Curve Progressive Local Search Scheduling

**Motivation.** The computational cost of VND local search, even with candidate list acceleration, means that applying it to every ant at every iteration wastes resources in early generations when the colony has not yet identified promising regions of the search space. Conversely, in later generations when the pheromone has converged toward high-quality solutions, intensive local search is crucial for fine-tuning. A fixed fraction of ants undergoing local search throughout the run is suboptimal: too low and convergence stalls; too high and early computation is squandered on low-quality cycles.

**Mechanism.** We schedule the fraction of ants that receive VND local search according to an S-curve (sigmoid) function over the iteration budget:

$$\text{optRatio}(t) = y_{\min} + (y_{\max} - y_{\min}) \cdot \frac{t^a}{t^a + (1 - t)^a} \quad (20)$$

where $t = (\text{iter} - x_L) / (x_R - x_L)$ is the normalized iteration position within the transition interval $[x_L, x_R] = [0, 30]$. The parameters are: $y_{\min} = 0.0$ (no local search at the very start), $y_{\max} = 0.4$ (at most $40\%$ of ants receive local search in late iterations), and $a = 2.0$ (steepness of the S-curve). Before $x_L$, $\text{optRatio} = y_{\min}$; after $x_R$, $\text{optRatio} = y_{\max}$; within the interval, the ratio follows the smooth sigmoid transition.

The S-curve provides three desirable properties: (i) in early iterations ($\text{optRatio} \approx 0$), the colony explores broadly without the computational cost of local search, relying purely on construction-level diversification; (ii) in the transition phase, the fraction of optimized ants increases gradually, allowing the pheromone to adapt to the improved solutions; (iii) in late iterations ($\text{optRatio} = 0.4$), a substantial fraction of ants receives local search, while some ants continue to explore via construction alone, maintaining diversity. The transition interval $[0, 30]$ is intentionally short: the S-curve reaches its maximum early, reflecting the observation that intensive local search is beneficial from the outset once a basic pheromone structure has been established (after approximately 30 iterations).

*[Figure 4: S-curve $\text{optRatio}(t)$ plotted against iteration number, showing the three regimes: flat at $0$ before iteration 0, sigmoid transition from iterations 0 to 30, flat at $0.4$ after iteration 30.]*

**LS ant selection.** Once the number of ants to optimize, $n_{\text{opt}} = \lceil n_{\text{ants}} \cdot \text{optRatio} \rceil$, is determined, the selection of which specific ants to optimize follows a mixed elite-random strategy:

$$n_{\text{elite}} = \lceil n_{\text{opt}} \cdot r_{\text{elite}} \rceil \quad (21)$$

$$n_{\text{random}} = n_{\text{opt}} - n_{\text{elite}} \quad (22)$$

where $r_{\text{elite}} = 0.7$ (the top $70\%$ of the LS budget goes to the best-ranked ants) and the remaining $n_{\text{random}}$ ants are randomly selected from the non-elite pool. This selection ensures that the best solutions are refined (elite exploitation) while a minority of randomly chosen ants' cycles are optimized, potentially discovering alternative high-quality regions of the search space that the elite pool has overlooked (controlled exploration).

### 4.3.6 Adaptive Stopping Criterion

**Motivation.** The maximum iteration count $n_{\text{iter}} = 800$ is chosen conservatively to accommodate hard problem instances. On easier instances, the colony may converge much earlier, and continuing the run wastes computation without improving the solution. Conversely, terminating too early risks leaving the best solution undiscovered. An adaptive stopping mechanism should detect genuine convergence while guarding against premature termination due to temporary stalling.

**Mechanism.** Two complementary convergence indicators are monitored:

**Condition 1 — Population homogeneity.** When the ant colony has converged to a narrow region of the search space, the cost variance across ants becomes very small. We measure this via the coefficient of variation (CV) of ant costs at each iteration:

$$\text{CV} = \frac{\sigma_{\text{costs}}}{\mu_{\text{costs}}} \quad (23)$$

where $\sigma_{\text{costs}}$ and $\mu_{\text{costs}}$ are the standard deviation and mean of all ant cycle costs in the current generation. If $\text{CV} < 0.001$, the population is considered homogeneous—all ants are producing cycles of nearly identical quality, and further iterations are unlikely to discover new solutions.

**Condition 2 — Best-solution stagnation.** Even when the population remains diverse, the global-best cycle may be trapped in a local optimum. We track the number of consecutive iterations without improvement to the global-best cost. If this stagnation counter reaches $70$ generations, the search is terminated.

**Minimum guard.** Both conditions are active only after $\text{minIter} = 30$ iterations, ensuring that the colony has had sufficient time to develop meaningful pheromone structures before convergence monitoring begins. This prevents premature termination during the initial exploration phase when cost variance is naturally high and the global best is still improving rapidly.

---

### 4.3.7 Integration Summary

The complete ACO procedure integrates the six mechanisms described above into a single iterative search loop. The algorithm operates on the extended cost matrix $\mathbf{D}_{\text{ext}}$ over $N+1$ nodes (including the virtual node). Each iteration proceeds as follows:

1. The S-curve (Eq. 20) determines the fraction of ants that will receive local search.
2. All $40$ ants construct complete Hamiltonian cycles over $N+1$ nodes using the ACS-style pseudo-random transition rule (Eqs. 18--19).
3. A mixed elite-random selection (Eqs. 21--22) chooses which ants undergo VND local search with candidate list acceleration (Algorithm 4).
4. The global-best cycle is updated and the adaptive stopping criteria (Eq. 23) are checked.
5. Pheromone is evaporated (Eq. 13) and deposited on the iteration-best and global-best cycles (Eqs. 14--15), followed by dynamic MMAS bound clamping (Eqs. 16--17).

The search terminates when either the population cost coefficient of variation drops below $0.001$ or the global-best cycle stagnates for $70$ consecutive generations, with both checks active only after a minimum of $30$ iterations. Upon termination, the open-path visit order is extracted from the best cycle by locating and removing the virtual node $V$, then orienting the remaining sequence so that the start ($1$) appears first and the goal ($N$) appears last.

*[Flowchart: Overall improved ACO procedure.]*

```mermaid
flowchart TB
    A["Input: costMatrix[N×N], N\nExtend to Dext[(N+1)×(N+1)]\nwith virtual node V"] --> B["Initialize pheromone τ, heuristic η, candidate list C"]
    B --> C["For each iteration"]
    C --> D["Compute S-curve optRatio"]
    D --> E["40 ants construct cycles\n(ACS pseudo-random rule, Eq. 19)"]
    E --> F["Select LS ants\n(elite 70% + random 30%)"]
    F --> G["VND on closed cycles\nwith candidate list (Algorithm 4)"]
    G --> H["Update global best"]
    H --> I{"Adaptive stop?\n(CV<0.001 or stagnation≥70)"}
    I -->|"yes"| J["Extract path from cycle\n(remove virtual node V)"]
    I -->|"no"| K["Pheromone evaporation (Eq. 13)"]
    K --> L["Pheromone deposit (Eqs. 14-15)"]
    L --> M["MMAS clamp (Eqs. 16-17)"]
    M --> C
```
