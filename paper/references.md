# 参考文献清单（供论文引用）

> 按论文章节归类。**标注 ★ 的为较新文献（2021 年后）**，优先选用；未标注者为奠基性经典文献，必须引用以建立学术脉络。

---

## A. 全局路径规划 / 跳点搜索（对应 Related Work §2.1）

### 奠基性经典文献
- **Dijkstra, E. W.** (1959). A note on two problems in connexion with graphs. *Numerische Mathematik*, 1(1), 269–271.
- **Hart, P. E., Nilsson, N. J., & Raphael, B.** (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107. 【A* 算法原始文献】
- **LaValle, S. M.** (1998). Rapidly-exploring random trees: A new tool for path planning. *TR 98-11, Iowa State University*. 【RRT 原始文献】
- **Harabor, D., & Grastien, A.** (2011). Online graph pruning for pathfinding on grid maps. *Proceedings of AAAI-11*. 【JPS 原始文献】
- **Harabor, D., & Grastien, A.** (2014). Improving jump point search. *Proceedings of ICAPS-14*. 【JPS 改进版】

### ★ 较新文献
- ★ **Improved JPS and DWA path planning algorithm for mobile robots** (2026). *Expert Systems with Applications*. 【JPS 与局部规划器融合，与本文路径规划+后处理思路相关】
- ★ **Research on Mobile Robot Path Planning Based on an Improved Bidirectional Jump Point Search Algorithm** (2025). *Electronics*, 14(8), 1669. DOI: `10.3390/electronics14081669`. 【双向 JPS 改进】
- ★ **Adaptive weight optimization based jump point Theta* algorithm in mobile robot path planning for intricate environments** (2025). *Discover Applied Sciences*, 7, 11. DOI: `10.1007/s42452-025-07932-z`. 【JPS 与任意角度 Theta* 融合，自适应权重】

---

## B. 蚁群优化 / 旅行商问题 / MMAS / VND（对应 Related Work §2.2）

### 奠基性经典文献
- **Dorigo, M., Maniezzo, V., & Colorni, A.** (1996). Ant system: optimization by a colony of cooperating agents. *IEEE Transactions on Systems, Man, and Cybernetics, Part B*, 26(1), 29–41. 【蚁群系统原始文献】
- **Dorigo, M., & Gambardella, L. M.** (1997). Ant colony system: a cooperative learning approach to the traveling salesman problem. *IEEE Transactions on Evolutionary Computation*, 1(1), 53–66. 【ACS 原始文献】
- **Stützle, T., & Hoos, H. H.** (2000). MAX–MIN ant system. *Future Generation Computer Systems*, 16(8), 889–914. 【MMAS 原始文献】
- **Hansen, P., & Mladenović, N.** (2001). Variable neighborhood search: Principles and applications. *European Journal of Operational Research*, 130(3), 449–467. 【VNS/VND 原始文献】
- **Kirkpatrick, S., Gelatt, C. D., & Vecchi, M. P.** (1983). Optimization by simulated annealing. *Science*, 220(4598), 671–680. 【SA 原始文献】
- **Holland, J. H.** (1975). *Adaptation in Natural and Artificial Systems*. University of Michigan Press. 【GA 原始文献】

### ★ 较新文献
- ★ **Kyriakakis, N. A., Marinaki, M., & Marinakis, Y.** (2021). A hybrid ant colony optimization–variable neighborhood descent approach for the cumulative capacitated vehicle routing problem. *Computers & Operations Research*, 134, 105397. 【★★ 高度相关：MMAS-VND 与 ACS-VND 混合框架，与本文核心方法最接近】
- ★ **Matrix-Based Ant Colony System for Traveling Salesman Problem** (2024). *IEEE SMC*. 【矩阵化 ACS 并行加速，大规模 TSP】
- ★ **nLKH-ACS: A Niching Lin-Kernighan–Helsgaun-Based Ant Colony System for Multisolution Traveling Salesman Problems** (2024). *IEEE Transactions on Evolutionary Computation*. 【ACS + 局部搜索 niching，2024 最新】
- ★ **CAACS: A Carbon Aware Ant Colony System** (2024/2025). arXiv:2407.09404 / *Scientific Reports*. 【ACS 变体，广义 TSP】
- ★ **Ant Colony Optimization for Multiple Traveling Salesmen Problem with Revisitable Cities** (2024). *IEEE*. 【ACO + 2-opt 精英局部搜索，多 TSP 变体】

---

## C. 多目标点遍历 / 带障碍 TSP（对应 Related Work §2.3）

### 奠基性经典文献
- **Wu, C.-G., Liang, Y.-C., Lee, H.-P., Lu, C., & Lin, C.** (2004). Solving constrained traveling salesman problems by genetic algorithms. *Progress in Natural Science*, 14(7), 631–637. 【约束 TSP：开放路由/固定端点/路径约束三类问题的奠基文献】

### ★ 较新文献
- ★ **SMUG Planner: A Safe Multi-Goal Planner for Mobile Robots in Challenging Environments** (2023). *IEEE Robotics and Automation Letters*. arXiv:2306.05309. 【★★ 高度相关：安全多目标点规划器，TSP + 碰撞安全，ANYmal 四足机器人部署】
- ★ **Fast marching firework method for multi-goal mobile robot path planning in complex obstacle maps** (2025). *Intelligent Service Robotics*. DOI: `10.1007/s11370-025-00654-6`. 【★★ 高度相关：复杂障碍地图多目标点路径规划，2025 最新】
- ★ **A survey on approximability of traveling salesman problems using the TSP-T3CO definition scheme** (2025). *Annals of Operations Research*. DOI: `10.1007/s10479-025-06641-5`. 【TSP 变体统一分类框架（含 start/end/circuit 属性），适合形式化引用】

---

## D. 路径平滑 / 安全距离 / 避障（对应 Methodology §4.2）

### ★ 较新文献
- ★ **Enhancing the safety and smoothness of path planning through an integration of Dijkstra's algorithm and piecewise cubic Bezier optimization** (2025). *Expert Systems with Applications*. 【Dijkstra + 分段三次 Bezier，安全+平滑，与本文 SimplifyPath + SmoothPath 管线直接对应】
- ★ **Adaptive A*/NSGA-II framework for multi-goal navigation of mobile robots** (2025). *Measurement Science and Technology*. DOI: `10.1088/1361-6501/ae9209`. 【多目标导航 + B 样条平滑 + 安全距离约束】

---

## 使用建议

1. **优先引用 ★ 较新文献**（2021+）以体现研究前沿性，尤其在 Related Work §2.3（多目标点遍历）中，SMUG Planner (2023) 和 Fast Marching Firework (2025) 是最直接的对比对象。
2. **奠基性经典文献**不可省略——它们是算法理论来源（JPS、MMAS、ACS、VND 的原始出处），审稿人预期看到这些引用。
3. **最高度相关的文献**（用 ★★ 标注）：
   - Kyriakakis et al. (2021) — MMAS-VND 混合框架（本文 ACO 方法的直接理论前身）
   - SMUG Planner (2023) 和 Fast Marching Firework (2025) — 障碍环境多目标点遍历（本文问题定位的直接对比）

> ⚠️ 注意：部分文献的精确卷期页码需通过 DOI 或文献管理工具进一步核对后再正式列入 References。当前 DOI 均来自检索结果。
