# -*- coding: utf-8 -*-
"""Edit paper_CN.tex references: remove weak old refs, add verified new ones, fix authors.

Citation integrity rules:
- Only claim what titles/abstracts/metadata support.
- Remove refs whose claims they don't actually support (ref3 Wu 2004 for TSPO/visibility).
- Keep foundational classics (Dijkstra, A*, RRT, ACS, MMAS, VNS, JPS originals).
"""
from pathlib import Path
import re

path = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
text = path.read_text(encoding="utf-8")
orig_len = len(text)

# ---------- 1) Front-matter citation / wording edits ----------
replacements = [
    # Intro applications: add surveys + concrete application papers
    (
        "在无人公交接驳、物流仓储管理、基础设施巡检以及搜救行动等场景中具有广泛应用\\textcolor{blue}{\\cite{ref1,ref2}}。",
        "在无人公交接驳、物流仓储管理、基础设施巡检以及搜救行动等场景中具有广泛应用\\textcolor{blue}{\\cite{ref1,ref2,ref33,ref37,ref38}}。",
    ),
    # Euclidean-cost assumption: drop Wu 2004 (ref3), use verified TSP literature
    (
        "大量旅行商问题研究假设点对间的代价由欧氏距离或已知距离矩阵给出\\textcolor{blue}{\\cite{ref3,ref4}}，忽略了在障碍物丰富的环境，两点之间的最短无碰撞路径可能远长于直线距离这一事实。",
        "大量旅行商问题研究假设点对间的代价由欧氏距离或已知距离矩阵给出\\textcolor{blue}{\\cite{ref4,ref36}}，忽略了在障碍物丰富的环境中，两点之间的最短无碰撞路径可能远长于直线距离这一事实\\textcolor{blue}{\\cite{ref39}}。",
    ),
    # Mature planners sentence: keep classics, optionally note surveys later in related work
    (
        "但这些规划器并不优化多目标点的访问顺序\\textcolor{blue}{\\cite{ref5,ref6,ref7}}。",
        "但这些规划器并不优化多目标点的访问顺序\\textcolor{blue}{\\cite{ref5,ref6,ref7}}。",
    ),
    # Weighted A* / ARA*: drop Rios 2010 (ref9)
    (
        "加权A*与任意时间修复A*等变体在整个搜索过程中施加统一权重\\textcolor{blue}{\\cite{ref9,ref10}}",
        "加权A*与任意时间修复A*等变体在整个搜索过程中施加统一权重\\textcolor{blue}{\\cite{ref8,ref10}}",
    ),
    # Path post-processing: add Huo 2024 ACO+B-spline (real integration of planning+smoothing)
    (
        "以及三次样条、B样条、回旋曲线等平滑方法\\textcolor{blue}{\\cite{ref17,ref18}}",
        "以及三次样条、B样条、回旋曲线等平滑方法\\textcolor{blue}{\\cite{ref17,ref18,ref41}}",
    ),
    # Metaheuristics: drop simulated annealing + ref20 (old, non-central)
    (
        "更大规模通常采用元启发式方法，如遗传算法\\textcolor{blue}{\\cite{ref19}}、模拟退火\\textcolor{blue}{\\cite{ref20}}与蚁群优化\\textcolor{blue}{\\cite{ref21}}。",
        "更大规模通常采用元启发式方法，如遗传算法\\textcolor{blue}{\\cite{ref19}}与蚁群优化\\textcolor{blue}{\\cite{ref21,ref42}}。",
    ),
    # Hybrid local search: add Zhou 2022 adaptive ACO (verified hybrid ACO)
    (
        "将2-opt、可变邻域下降（VND）等局部搜索嵌入蚁群虽可显著提升解质量\\textcolor{blue}{\\cite{ref24,ref25}}",
        "将2-opt、可变邻域下降（VND）等局部搜索嵌入蚁群虽可显著提升解质量\\textcolor{blue}{\\cite{ref24,ref25,ref43}}",
    ),
    # Related work 2.1 opening: add path-planning survey
    (
        "全局路径规划旨在已知静态环境中寻找两点之间的无碰撞路径，经典图搜索算法是栅格规划的主流方法。",
        "全局路径规划旨在已知静态环境中寻找两点之间的无碰撞路径，经典图搜索算法是栅格规划的主流方法\\textcolor{blue}{\\cite{ref33,ref34,ref35}}。",
    ),
    # JPS system papers: add SoCS JPS system summary
    (
        "跳点搜索（JPS）利用栅格对称性仅扩展跳点，大幅压缩搜索树\\textcolor{blue}{\\cite{ref11,ref12}}",
        "跳点搜索（JPS）利用栅格对称性仅扩展跳点，大幅压缩搜索树\\textcolor{blue}{\\cite{ref11,ref12,ref44}}",
    ),
    # TSP problem depth: add GTSP/TSP survey
    (
        "旅行商问题是组合优化领域中研究最为深入的问题之一，精确算法仅适用于小规模实例（通常$K \\leq 20$）",
        "旅行商问题是组合优化领域中研究最为深入的问题之一\\textcolor{blue}{\\cite{ref36,ref34}}，精确算法仅适用于小规模实例（通常$K \\leq 20$）",
    ),
    # Related work 2.3: rewrite TSPO sentence to match verified literature
    (
        "路径规划与旅行商优化的交叉领域已在多个名义下得到探索\\textcolor{blue}{\\cite{ref4}}。带障碍物的旅行商问题（TSPO）在连续平面多边形障碍下经可见图求解\\textcolor{blue}{\\cite{ref3}}，Dubins变体考虑最小转弯半径约束，邻域变体将目标从点泛化为区域。",
        "路径规划与旅行商优化的交叉领域已在多个名义下得到探索\\textcolor{blue}{\\cite{ref4,ref36}}。在连续平面多边形障碍下，点对通行代价依赖避障几何而非欧氏度量\\textcolor{blue}{\\cite{ref39}}；Dubins变体考虑最小转弯半径约束，邻域或贴近变体则将目标从点泛化为区域\\textcolor{blue}{\\cite{ref40}}。",
    ),
    # Open vs closed TSP
    (
        "这是\\textbf{开放旅行商问题}的形式化，区别于经典闭环旅行商问题——后者的路径构成环路且起始节点可任意选取\\textcolor{blue}{\\cite{ref3,ref4}}。",
        "这是\\textbf{开放旅行商问题}的形式化，区别于经典闭环旅行商问题——后者的路径构成环路且起始节点可任意选取\\textcolor{blue}{\\cite{ref4,ref36}}。",
    ),
    # Virtual node: keep only Yao 2019 (drop Yao 2014 ref31)
    (
        "该方法类似于开放车辆路径问题中所采用的技术\\textcolor{blue}{\\cite{ref31,ref32}}。",
        "该方法类似于开放车辆路径问题中所采用的技术\\textcolor{blue}{\\cite{ref32}}。",
    ),
    (
        "该技术在开放车辆路径问题中被广泛采用\\textcolor{blue}{\\cite{ref31,ref32}}。",
        "该技术在开放车辆路径问题中被广泛采用\\textcolor{blue}{\\cite{ref32}}。",
    ),
]

for old, new in replacements:
    if old not in text:
        print("MISSING:\n ", old[:120])
        continue
    if old != new:
        text = text.replace(old, new, 1)
        print("OK replace:", old[:60], "->", new[:60])

# ---------- 2) Bibliography rewrite ----------
# Find bibliography block
bib_m = re.search(r"\\begin\{thebibliography\}\{(\d+)\}(.*?)\\end\{thebibliography\}", text, re.S)
if not bib_m:
    raise SystemExit("bibliography not found")

new_bib = r"""\begin{thebibliography}{44}

\bibitem{ref1}

C.~Chen, J.~Frey, P.~Arm, and M.~Hutter, ``SMUG planner: A safe multi-goal planner for mobile robots in challenging environments,'' \textit{IEEE Robot. Autom. Lett.}, vol.~8, no.~11, pp.~7170--7177, 2023, doi: 10.1109/LRA.2023.3311207.

\bibitem{ref2}

T.~T.~C. Giang, H.~T.~T. Binh, H.~V.~D. Luong, N.~H. Hoang, and D.~Q. Huy, ``Fast marching firework method for multi-goal mobile robot path planning in complex obstacle maps,'' \textit{Intell. Service Robot.}, vol.~18, no.~6, pp.~1467--1484, 2025, doi: 10.1007/s11370-025-00654-6.

\bibitem{ref4}

S.~Saller, J.~Koehler, and A.~Karrenbauer, ``A survey on approximability of traveling salesman problems using the TSP-T3CO definition scheme,'' \textit{Ann. Oper. Res.}, vol.~351, no.~3, pp.~2129--2190, 2025, doi: 10.1007/s10479-025-06641-5.

\bibitem{ref5}

P.~E. Hart, N.~J. Nilsson, and B. Raphael, ``A formal basis for the heuristic determination of minimum cost paths,'' \textit{IEEE Trans. Syst. Sci. Cybern.}, vol.~4, no.~2, pp.~100--107, 1968.

\bibitem{ref6}

E.~W. Dijkstra, ``A note on two problems in connexion with graphs,'' \textit{Numer. Math.}, vol.~1, no.~1, pp.~269--271, 1959.

\bibitem{ref7}

S.~M. LaValle, ``Rapidly-exploring random trees: A new tool for path planning,'' Dept. Comput. Sci., Iowa State Univ., Ames, IA, USA, Tech. Rep. 98-11, 1998.

\bibitem{ref8}

D.~T. Xuan, N.~T. Hung, and V.~T. Thang, ``A comprehensive review of improved A* path planning algorithms and their hybrid integrations,'' \textit{Automation}, vol.~6, no.~4, p.~52, 2025, doi: 10.3390/automation6040052.

\bibitem{ref10}

M. Likhachev, G. Gordon, and S. Thrun, ``ARA*: Anytime A* with provable bounds on sub-optimality,'' in \textit{Adv. Neural Inf. Process. Syst. (NIPS)}, 2003, pp.~767--774.

\bibitem{ref11}

D. Harabor and A. Grastien, ``Online graph pruning for pathfinding on grid maps,'' in \textit{Proc. 25th AAAI Conf. Artif. Intell. (AAAI-11)}, 2011, pp.~1114--1119.

\bibitem{ref12}

D. Harabor and A. Grastien, ``Improving jump point search,'' in \textit{Proc. 24th Int. Conf. Automated Planning Scheduling (ICAPS-14)}, 2014, pp.~128--135.

\bibitem{ref13}

C.~Gao, H.~Ding, D.~Zhang, and Y.~Wu, ``Improved JPS and DWA path planning algorithm for mobile robots,'' \textit{Mechatronics}, vol.~118, p.~103540, 2026, doi: 10.1016/j.mechatronics.2026.103540.

\bibitem{ref14}

R.~Guo, X.~Quan, and C.~Bao, ``Research on mobile robot path planning based on an improved bidirectional jump point search algorithm,'' \textit{Electronics}, vol.~14, no.~8, p.~1669, 2025, doi: 10.3390/electronics14081669.

\bibitem{ref15}

``Adaptive weight optimization based jump point Theta* algorithm in mobile robot path planning for intricate environments,'' \textit{Discover Appl. Sci.}, vol.~7, p.~11, 2025, doi: 10.1007/s42452-025-07932-z.

\bibitem{ref16}

I.~Munasinghe, A.~G. Perera, S.~Anavatti, and M.~Garratt, ``UGV path optimization in UAV-assisted environments using visibility-aware path simplification,'' \textit{J. Sens. Actuator Netw.}, vol.~15, no.~3, p.~41, 2026, doi: 10.3390/jsan15030041.

\bibitem{ref17}

J.~Ahmad and M.~N. Ab Wahab, ``Enhancing the safety and smoothness of path planning through an integration of Dijkstra's algorithm and piecewise cubic Bezier optimization,'' \textit{Expert Syst. Appl.}, vol.~289, p.~128315, 2025, doi: 10.1016/j.eswa.2025.128315.

\bibitem{ref18}

O.~E. Turan, O.~Misir, and M.~Ozden, ``Adaptive A*/NSGA-II framework for multi-goal navigation of mobile robots,'' \textit{Meas. Sci. Technol.}, 2026, doi: 10.1088/1361-6501/ae9209.

\bibitem{ref19}

J.~H. Holland, \textit{Adaptation in Natural and Artificial Systems}. Ann Arbor, MI, USA: Univ. Michigan Press, 1975.

\bibitem{ref21}

M. Dorigo, V. Maniezzo, and A. Colorni, ``Ant system: Optimization by a colony of cooperating agents,'' \textit{IEEE Trans. Syst., Man, Cybern. B}, vol.~26, no.~1, pp.~29--41, 1996.

\bibitem{ref22}

M. Dorigo and L.~M. Gambardella, ``Ant colony system: A cooperative learning approach to the traveling salesman problem,'' \textit{IEEE Trans. Evol. Comput.}, vol.~1, no.~1, pp.~53--66, 1997.

\bibitem{ref23}

T. St\"utzle and H.~H. Hoos, ``MAX--MIN ant system,'' \textit{Future Gener. Comput. Syst.}, vol.~16, no.~8, pp.~889--914, 2000.

\bibitem{ref24}

P. Hansen and N. Mladenovi\'c, ``Variable neighborhood search: Principles and applications,'' \textit{Eur. J. Oper. Res.}, vol.~130, no.~3, pp.~449--467, 2001.

\bibitem{ref25}

N.~A. Kyriakakis, M. Marinaki, and Y. Marinakis, ``A hybrid ant colony optimization--variable neighborhood descent approach for the cumulative capacitated vehicle routing problem,'' \textit{Comput. Oper. Res.}, vol.~134, p.~105397, 2021.

\bibitem{ref26}

F.~Uddin, N.~Riaz, A.~Manan, I.~Mahmood, O.-Y. Song, A.~J. Malik, and A.~A. Abbasi, ``An improvement to the 2-opt heuristic algorithm for approximation of optimal TSP tour,'' \textit{Appl. Sci.}, vol.~13, no.~12, p.~7339, 2023, doi: 10.3390/app13127339.

\bibitem{ref27}

X.~Li, J.-Y. Li, C.~Chen, Z.-H. Zhan, S.~Kwong, and J.~Zhang, ``Matrix-based ant colony system for traveling salesman problem,'' in \textit{Proc. IEEE Int. Conf. Syst., Man, Cybern. (SMC)}, 2024, pp.~1358--1363, doi: 10.1109/SMC54092.2024.10831943.

\bibitem{ref28}

T.~Huang, Z.-Q. Zhang, Y.-J. Gong, and J.~Liu, ``nLKH-ACS: A niching Lin-Kernighan--Helsgaun-based ant colony system for multisolution traveling salesman problems,'' \textit{IEEE Trans. Evol. Comput.}, vol.~29, no.~6, pp.~2596--2610, 2024, doi: 10.1109/TEVC.2024.3507777.

\bibitem{ref29}

M.~Lin and L.~P. Schaposnik, ``A carbon aware ant colony system for the sustainable generalized traveling salesman problem,'' \textit{Sci. Rep.}, vol.~15, p.~21928, 2025, doi: 10.1038/s41598-025-08276-9.

\bibitem{ref30}

B.~Sun, W.-J. Yu, Z.-J. Wang, and J.-Y. Li, ``Ant colony optimization for multiple traveling salesmen problem with revisitable cities,'' in \textit{Proc. IEEE Int. Conf. Mach. Intell. Theory Appl. (MiTA)}, 2024, pp.~1--6, doi: 10.1109/MiTA60795.2024.10751693.

\bibitem{ref32}

B. Yao, C. Chen, X. Song, and X. Yang, ``Fresh seafood delivery routing problem using an improved ant colony optimization,'' \textit{Ann. Oper. Res.}, vol.~273, no.~1--2, pp.~163--186, 2019, doi: 10.1007/s10479-017-2531-2.

\bibitem{ref33}

K.~Karur, N.~Sharma, C.~Dharmatti, and J.~E. Siegel, ``A survey of path planning algorithms for mobile robots,'' \textit{Vehicles}, vol.~3, no.~3, pp.~448--468, 2021, doi: 10.3390/vehicles3030027.

\bibitem{ref34}

J.~A. Abdulsaheb and D.~J. Kadhim, ``Classical and heuristic approaches for mobile robot path planning: A survey,'' \textit{Robotics}, vol.~12, no.~4, p.~93, 2023, doi: 10.3390/robotics12040093.

\bibitem{ref35}

K.~Katona, H.~A. Neamah, and P.~K\"or\"ondi, ``Obstacle avoidance and path planning methods for autonomous navigation of mobile robot,'' \textit{Sensors}, vol.~24, no.~11, p.~3573, 2024, doi: 10.3390/s24113573.

\bibitem{ref36}

P.~C. Pop, O.~Cosma, C.~Sabo, and C.~Pop Sitar, ``A comprehensive survey on the generalized traveling salesman problem,'' \textit{Eur. J. Oper. Res.}, vol.~314, no.~3, pp.~819--835, 2023, doi: 10.1016/j.ejor.2023.07.022.

\bibitem{ref37}

J.~Wang and M.~Q.-H. Meng, ``Real-time decision making and path planning for robotic autonomous luggage trolley collection at airports,'' \textit{IEEE Trans. Syst., Man, Cybern.: Syst.}, vol.~52, no.~4, pp.~2174--2183, 2022, doi: 10.1109/TSMC.2020.3048984.

\bibitem{ref38}

A.~Bolu and \"O.~Kor\c{c}ak, ``Adaptive task planning for multi-robot smart warehouse,'' \textit{IEEE Access}, vol.~9, pp.~27346--27358, 2021, doi: 10.1109/ACCESS.2021.3058190.

\bibitem{ref39}

J.~Faigl, ``On the performance of self-organizing maps for the non-Euclidean traveling salesman problem in the polygonal domain,'' \textit{Inf. Sci.}, vol.~181, no.~19, pp.~4214--4229, 2011, doi: 10.1016/j.ins.2011.05.019.

\bibitem{ref40}

B.~Behdani and J.~C. Smith, ``An integer-programming-based approach to the close-enough traveling salesman problem,'' \textit{INFORMS J. Comput.}, vol.~26, no.~3, pp.~415--432, 2014, doi: 10.1287/ijoc.2013.0574.

\bibitem{ref41}

F.~Huo, S.~Zhu, H.~Dong, and W.~Ren, ``A new approach to smooth path planning of Ackerman mobile robot based on improved ACO algorithm and B-spline curve,'' \textit{Robot. Auton. Syst.}, vol.~175, p.~104655, 2024, doi: 10.1016/j.robot.2024.104655.

\bibitem{ref42}

Y.~Wang and Z.~Han, ``Ant colony optimization for traveling salesman problem based on parameters optimization,'' \textit{Appl. Soft Comput.}, vol.~107, p.~107439, 2021, doi: 10.1016/j.asoc.2021.107439.

\bibitem{ref43}

X.~Zhou, H.~Ma, J.~Gu, H.~Chen, and W.~Deng, ``Parameter adaptation-based ant colony optimization with dynamic hybrid mechanism,'' \textit{Eng. Appl. Artif. Intell.}, vol.~114, p.~105139, 2022, doi: 10.1016/j.engappai.2022.105139.

\bibitem{ref44}

D. Harabor and A. Grastien, ``The JPS pathfinding system,'' in \textit{Proc. Int. Symp. Comb. Search (SoCS)}, vol.~3, no.~1, pp.~207--208, 2021, doi: 10.1609/socs.v3i1.18254.

\end{thebibliography}"""

text = text[: bib_m.start()] + new_bib + text[bib_m.end() :]

# Count bibitems
n_bib = len(re.findall(r"\\bibitem\{", text))
print("bibitem count:", n_bib)

# Verify no leftover cites to deleted keys
for bad in ["ref3}", "ref9}", "ref20}", "ref31}"]:
    if bad in text:
        # might appear in \cite{ref3} etc
        leftover = re.findall(r"\\cite\{[^}]*" + re.escape(bad[:-1]) + r"[,}][^}]*\}", text)
        print("LEFTOVER cite for", bad, ":", leftover)

# Check all cited keys exist
cited = set()
for m in re.finditer(r"\\cite\{([^}]+)\}", text):
    for k in m.group(1).split(","):
        cited.add(k.strip())
bibkeys = set(re.findall(r"\\bibitem\{([^}]+)\}", text))
print("cited not in bib:", sorted(cited - bibkeys))
print("bib not cited:", sorted(bibkeys - cited))
print("len", orig_len, "->", len(text))

path.write_text(text, encoding="utf-8")
print("WROTE", path)
