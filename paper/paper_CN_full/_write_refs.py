# -*- coding: utf-8 -*-
"""Write IEEE-style bibitem suggestions for the paper."""
from pathlib import Path

out = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\refs_suggestions.tex")
text = r"""% 参考文献补充建议（按推荐插入位置分组）
% 用法：将对应 \bibitem 粘贴到 paper_CN.tex 的 thebibliography 中，
% 并把编号改成当前文献表末尾顺延；正文用 \cite{...} 引用。
% 说明：作者按“姓, 名首字母”IEEE 格式；期刊缩写已尽量对齐文中现有风格。

%% =========================================================
%% A. 综述类（引言第一段“广泛应用”/相关工作开头，或综述旁证）
%% =========================================================

\bibitem{refA1}
K.~Karur, N.~Sharma, C.~Dharmatti, and J.~E. Siegel,
``A survey of path planning algorithms for mobile robots,''
\textit{Vehicles}, vol.~3, no.~3, pp.~448--468, 2021,
doi: 10.3390/vehicles3030027.

\bibitem{refA2}
J.~A. Abdulsaheb and D.~J. Kadhim,
``Classical and heuristic approaches for mobile robot path planning: A survey,''
\textit{Robotics}, vol.~12, no.~4, p.~93, 2023,
doi: 10.3390/robotics12040093.

\bibitem{refA3}
K.~Katona, H.~A. Neamah, and P.~K\"or\"ondi,
``Obstacle avoidance and path planning methods for autonomous navigation of mobile robot,''
\textit{Sensors}, vol.~24, no.~11, p.~3573, 2024,
doi: 10.3390/s24113573.

\bibitem{refA4}
A.~Bektemessov,
``A survey of path planning and obstacle avoidance techniques in mobile robotics,''
\textit{Eng. Technol. Appl. Sci. Res.}, vol.~15, no.~6, pp.~29632--29640, 2025,
doi: 10.48084/etasr.13885.

\bibitem{refA5}
X.~Xiao, B.~Liu, G.~Warnell, and P.~Stone,
``Motion planning and control for mobile robot navigation using machine learning: A survey,''
\textit{Auton. Robot.}, vol.~46, no.~5, pp.~569--597, 2022,
doi: 10.1007/s10514-022-10039-8.

\bibitem{refA6}
P.~C. Pop, O.~Cosma, C.~Sabo, and C.~Pop Sitar,
``A comprehensive survey on the generalized traveling salesman problem,''
\textit{Eur. J. Oper. Res.}, vol.~314, no.~3, pp.~819--835, 2023,
doi: 10.1016/j.ejor.2023.07.022.

\bibitem{refA7}
A.~Mezzina and M.~F. Pavone,
``A comprehensive review of metaheuristics for the modern traveling salesman problem and drone-assisted delivery,''
\textit{Algorithms}, vol.~19, no.~4, p.~278, 2026,
doi: 10.3390/a19040278.

\bibitem{refA8}
S.~El Kafhali, M.~Abid, and M.~Hanini,
``Advanced optimization methods for the knapsack, traveling salesman, and close-enough traveling salesman problems: A survey and case studies,''
\textit{Math. Comput. Appl.}, vol.~31, no.~5, p.~180, 2026,
doi: 10.3390/mca31050180.

\bibitem{refA9}
E.~Alanzi and M.~E. Mena\"i,
``Solving the traveling salesman problem with machine learning: A review of recent advances and challenges,''
\textit{Artif. Intell. Rev.}, vol.~58, no.~9, 2025,
doi: 10.1007/s10462-025-11267-x.

\bibitem{refA10}
J.~Sui, S.~Ding, X.~Huang, Y.~Yu, R.~Liu, B.~Xia, Z.~Ding, L.~Xu, H.~Zhang, C.~Yu, and D.~Bu,
``A survey on deep learning-based algorithms for the traveling salesman problem,''
\textit{Front. Comput. Sci.}, vol.~19, no.~6, 2024,
doi: 10.1007/s11704-024-40490-y.

%% =========================================================
%% B. 引言应用 / 问题动机（接驳、物流、巡检等）
%% =========================================================

\bibitem{refB1}
J.~Wang and M.~Q.-H. Meng,
``Real-time decision making and path planning for robotic autonomous luggage trolley collection at airports,''
\textit{IEEE Trans. Syst., Man, Cybern.: Syst.}, vol.~52, no.~4, pp.~2174--2183, 2022,
doi: 10.1109/TSMC.2020.3048984.

\bibitem{refB2}
A.~Bolu and \"O.~Kor\c{c}ak,
``Adaptive task planning for multi-robot smart warehouse,''
\textit{IEEE Access}, vol.~9, pp.~27346--27358, 2021,
doi: 10.1109/ACCESS.2021.3058190.

\bibitem{refB3}
F.~Nekov\'a\v{r}, J.~Faigl, and M.~Saska,
``Multi-tour set traveling salesman problem in planning power transmission line inspection,''
\textit{IEEE Robot. Autom. Lett.}, vol.~6, no.~4, pp.~6196--6203, 2021,
doi: 10.1109/LRA.2021.3091695.

\bibitem{refB4}
M.~D. Phung, C.~H. Quach, T.~H. Dinh, and Q.~P. Ha,
``Enhanced discrete particle swarm optimization path planning for UAV vision-based surface inspection,''
\textit{Autom. Constr.}, vol.~81, pp.~25--33, 2017,
doi: 10.1016/j.autcon.2017.04.013.

%% =========================================================
%% C. 多目标遍历 / 两类子问题耦合（相关工作第3小节、引言）
%% =========================================================

\bibitem{refC1}
C.~Chen, J.~Frey, P.~Arm, and M.~Hutter,
``SMUG planner: A safe multi-goal planner for mobile robots in challenging environments,''
\textit{IEEE Robot. Autom. Lett.}, vol.~8, no.~11, pp.~7170--7177, 2023,
doi: 10.1109/LRA.2023.3311207.
% 注：请修正现有 ref1 的作者（原写 S. B. Liu et al. 有误）

\bibitem{refC2}
D.~Lee, I.~M. A. Nahrendra, M.~Oh, B.~Yu, and H.~Myung,
``TRG-planner: Traversal risk graph-based path planning in unstructured environments for safe and efficient navigation,''
\textit{IEEE Robot. Autom. Lett.}, vol.~10, no.~2, pp.~1736--1743, 2025,
doi: 10.1109/LRA.2024.3524912.

\bibitem{refC3}
S.~Stavrinidis, P.~Th. Zacharia, and E.~K. Xidias,
``A fuzzy control strategy for multi-goal autonomous robot navigation,''
\textit{Sensors}, vol.~25, no.~2, p.~446, 2025,
doi: 10.3390/s25020446.

\bibitem{refC4}
T.~T.~C. Giang, H.~T.~T. Binh, H.~V.~D. Luong, N.~H. Hoang, and D.~Q. Huy,
``Fast marching firework method for multi-goal mobile robot path planning in complex obstacle maps,''
\textit{Intell. Service Robot.}, vol.~18, no.~6, pp.~1467--1484, 2025,
doi: 10.1007/s11370-025-00654-6.
% 若 ref2 已引用同一文，请合并作者信息

\bibitem{refC5}
J.~Faigl,
``On the performance of self-organizing maps for the non-Euclidean traveling salesman problem in the polygonal domain,''
\textit{Inf. Sci.}, vol.~181, no.~19, pp.~4214--4229, 2011,
doi: 10.1016/j.ins.2011.05.019.
% 很适合支撑“障碍环境下代价非欧氏”的动机

\bibitem{refC6}
K.~Vicencio, B.~Davis, and I.~Gentilini,
``Multi-goal path planning based on the generalized traveling salesman problem with neighborhoods,''
in \textit{Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)}, 2014, pp.~2985--2990,
doi: 10.1109/IROS.2014.6942974.

\bibitem{refC7}
J.~Deckerov\'a, P.~V\'a\v{n}a, and J.~Faigl,
``Combinatorial lower bounds for the generalized traveling salesman problem with neighborhoods,''
\textit{Expert Syst. Appl.}, vol.~258, p.~125185, 2024,
doi: 10.1016/j.eswa.2024.125185.

%% =========================================================
%% D. A* / JPS / 栅格寻径（相关工作第1小节、方法 3.1）
%% =========================================================

\bibitem{refD1}
D.~Harabor and A.~Grastien,
``The JPS pathfinding system,''
in \textit{Proc. Int. Symp. Comb. Search (SoCS)}, vol.~3, no.~1, pp.~207--208, 2021,
doi: 10.1609/socs.v3i1.18254.

\bibitem{refD2}
Y.~Hu, D.~Harabor, L.~Qin, and Q.~Yin,
``Regarding goal bounding and jump point search,''
\textit{J. Artif. Intell. Res.}, vol.~70, pp.~631--681, 2021,
doi: 10.1613/jair.1.12255.

\bibitem{refD3}
M.~Carlson, S.~Kardani Moghadam, D.~Harabor, P.~J. Stuckey, and M.~Ebrahimi,
``Optimal pathfinding on weighted grid maps,''
in \textit{Proc. AAAI Conf. Artif. Intell.}, vol.~37, no.~10, pp.~12373--12380, 2023,
doi: 10.1609/aaai.v37i10.26458.

% arXiv 近期 JPS 变体（可选，若需更多新文献）
% J. Baum, ``Jump point search pathfinding in 4-connected grids,'' arXiv:2501.14816, 2025.
% S. Zhao, D. Harabor, and P. J. Stuckey, ``Reducing redundant work in jump point search,'' arXiv:2306.15928, 2023.

%% =========================================================
%% E. 路径简化 / 样条平滑（相关工作第1小节末、方法 3.2）
%% =========================================================

\bibitem{refE1}
F.~Huo, S.~Zhu, H.~Dong, and W.~Ren,
``A new approach to smooth path planning of Ackerman mobile robot based on improved ACO algorithm and B-spline curve,''
\textit{Robot. Auton. Syst.}, vol.~175, p.~104655, 2024,
doi: 10.1016/j.robot.2024.104655.

\bibitem{refE2}
M.~N. Zafar and J.~C. Mohanta,
``Methodology for path planning and optimization of mobile robots: A review,''
\textit{Procedia Comput. Sci.}, vol.~133, pp.~141--152, 2018,
doi: 10.1016/j.procs.2018.07.018.

%% =========================================================
%% F. 开放/邻域/贴近遍历 TSP 变体（相关工作 2.3、问题形式化）
%% =========================================================

\bibitem{refF1}
B.~Behdani and J.~C. Smith,
``An integer-programming-based approach to the close-enough traveling salesman problem,''
\textit{INFORMS J. Comput.}, vol.~26, no.~3, pp.~415--432, 2014,
doi: 10.1287/ijoc.2013.0574.

\bibitem{refF2}
X.~Wang, B.~Golden, and E.~Wasil,
``A Steiner zone variable neighborhood search heuristic for the close-enough traveling salesman problem,''
\textit{Comput. Oper. Res.}, vol.~101, pp.~200--219, 2018,
doi: 10.1016/j.cor.2018.07.023.

\bibitem{refF3}
A.~Di Placido, C.~Archetti, C.~Cerrone, and B.~Golden,
``The generalized close enough traveling salesman problem,''
\textit{Eur. J. Oper. Res.}, vol.~310, no.~3, pp.~974--991, 2023,
doi: 10.1016/j.ejor.2023.04.010.

%% =========================================================
%% G. 蚁群 / 混合局部搜索（相关工作 2.2、方法 3.3）
%% =========================================================

\bibitem{refG1}
Y.~Wang and Z.~Han,
``Ant colony optimization for traveling salesman problem based on parameters optimization,''
\textit{Appl. Soft Comput.}, vol.~107, p.~107439, 2021,
doi: 10.1016/j.asoc.2021.107439.

\bibitem{refG2}
X.~Zhou, H.~Ma, J.~Gu, H.~Chen, and W.~Deng,
``Parameter adaptation-based ant colony optimization with dynamic hybrid mechanism,''
\textit{Eng. Appl. Artif. Intell.}, vol.~114, p.~105139, 2022,
doi: 10.1016/j.engappai.2022.105139.

\bibitem{refG3}
P.~Stodola, P.~Ot\v{r}\'isal, and K.~Hasilov\'a,
``Adaptive ant colony optimization with node clustering applied to the travelling salesman problem,''
\textit{Swarm Evol. Comput.}, vol.~70, p.~101056, 2022,
doi: 10.1016/j.swevo.2022.101056.

\bibitem{refG4}
K.~Tang, X.~Wei, Y.-H. Jiang, Z.~Chen, and L.~Yang,
``An adaptive ant colony optimization for solving large-scale traveling salesman problem,''
\textit{Mathematics}, vol.~11, no.~21, p.~4439, 2023,
doi: 10.3390/math11214439.

\bibitem{refG5}
M.~Scianna,
``The AddACO: A bio-inspired modified version of the ant colony optimization algorithm to solve travel salesman problems,''
\textit{Math. Comput. Simul.}, vol.~218, pp.~357--382, 2023,
doi: 10.1016/j.matcom.2023.12.003.

% 经典（若尚未收录）：MMAS+局部搜索
% T. St\"utzle and H. H. Hoos, ``MAX--MIN ant system and local search for the traveling salesman problem,''
% in Proc. IEEE Int. Conf. Evol. Comput. (ICEC), 1997, pp. 309--314.

"""
out.write_text(text, encoding="utf-8")
print("WROTE", out)
