# -*- coding: utf-8 -*-
from pathlib import Path

tex_path = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
text = tex_path.read_text(encoding="utf-8")

replacements = [
(
"""% TODO: 插入图——VND邻域搜索序列流程图
% \\begin{figure}[!t]
% \\centering
% \\includegraphics[width=2.5in]{fig_vnd_flow}
% \\caption{VND邻域搜索序列流程图：2-opt $\\to$ 重定位 $\\to$ 交换 $\\to$ 循环，以及各算子的候选列表剪枝。}
% \\label{fig:vnd_flow}
% \\end{figure}""",
r"""\begin{figure}[!t]
\centering
\includegraphics[width=2.2in]{figures/fig03_vnd_flow}
\caption{VND邻域搜索序列流程图。}
\label{fig:vnd_flow}
\end{figure}"""
),
(
"""% TODO: 插入图——S形optRatio(t)曲线
% \\begin{figure}[!t]
% \\centering
% \\includegraphics[width=2.5in]{fig_sigmoid}
% \\caption{S形$\\mathrm{optRatio}(t)$关于迭代次数的曲线图：迭代0之前平坦于0，迭代0至30的sigmoid过渡，迭代30之后平坦于0.4。}
% \\label{fig:sigmoid}
% \\end{figure}""",
r"""\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{figures/fig04_sigmoid}
\caption{S形$\mathrm{optRatio}(t)$关于迭代次数的曲线图。}
\label{fig:sigmoid}
\end{figure}"""
),
(
"""% TODO: 插入图——改进蚁群优化总体流程图
% \\begin{figure*}[!t]
% \\centering
% \\includegraphics[width=0.9\\textwidth]{fig_aco_flow}
% \\caption{改进蚁群优化总体流程图。}
% \\label{fig:aco_flow}
% \\end{figure*}""",
r"""\begin{figure*}[!t]
\centering
\includegraphics[width=0.75\textwidth]{figures/fig05_aco_flow}
\caption{改进蚁群优化总体流程图。}
\label{fig:aco_flow}
\end{figure*}"""
),
(
"""% TODO: 插入图——系统架构总图
% \\begin{figure*}[!t]
% \\centering
% \\includegraphics[width=0.95\\textwidth]{fig_system_arch}
% \\caption{系统架构总图：任务规格$\\to$第一层（A* + SimplifyPath$\\to$带路径缓存的代价矩阵）$\\to$第二层（蚁群优化$\\to$访问顺序）$\\to$路径后处理（SimplifyPath + SmoothPath）$\\to$连续参考轨迹。}
% \\label{fig:system}
% \\end{figure*}""",
r"""\begin{figure*}[!t]
\centering
\includegraphics[width=0.85\textwidth]{figures/fig06_system}
\caption{系统架构总图，展示完整管线。}
\label{fig:system}
\end{figure*}"""
),
("% TODO: 图1--图3：路径可视化、路径长度对比折线图、计算时间对比折线图",
r"""\begin{figure*}[!t]
\centering
\subfloat[本文A*（JPS）规划器]{\includegraphics[width=0.22\textwidth]{figures/fig07a_jps_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07a_jps_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07a_jps_m3}}\\
\subfloat[传统A*]{\includegraphics[width=0.22\textwidth]{figures/fig07b_astar_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07b_astar_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07b_astar_m3}}\\
\subfloat[Dijkstra]{\includegraphics[width=0.22\textwidth]{figures/fig07c_dijk_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07c_dijk_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07c_dijk_m3}}\\
\subfloat[RRT]{\includegraphics[width=0.22\textwidth]{figures/fig07d_rrt_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07d_rrt_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig07d_rrt_m3}}
\caption{四种算法在三张地图上的路径规划结果。每行从左至右依次为Map~1、Map~2、Map~3。}
\label{fig:plan_paths}
\end{figure*}

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{figures/fig08_pathlen}
\caption{四个算法在Map~1、2、3上的路径长度对比（折线图）。}
\label{fig:plan_len}
\end{figure}

\begin{figure}[!t]
\centering
\includegraphics[width=2.5in]{figures/fig09_time}
\caption{四个算法在Map~1、2、3上的计算时间对比（折线图）。}
\label{fig:plan_time}
\end{figure}"""
),
("% TODO: 图4--图6：参数敏感性代价/时间对比、默认参数路径图",
r"""\begin{figure*}[!t]
\centering
\includegraphics[width=0.32\textwidth]{figures/fig10_cost_nants}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig10_cost_q0}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig10_cost_ymax}\\
\includegraphics[width=0.32\textwidth]{figures/fig10_cost_relite}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig10_cost_k}
\caption{各参数水平在Map~1、Map~2、kroB100上的代价对比（折线图）。}
\label{fig:sens_cost}
\end{figure*}

\begin{figure*}[!t]
\centering
\includegraphics[width=0.32\textwidth]{figures/fig11_time_nants}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig11_time_q0}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig11_time_ymax}\\
\includegraphics[width=0.32\textwidth]{figures/fig11_time_relite}\hfil
\includegraphics[width=0.32\textwidth]{figures/fig11_time_k}
\caption{各参数水平在Map~1、Map~2、kroB100上的计算时间对比（折线图）。}
\label{fig:sens_time}
\end{figure*}

\begin{figure}[!t]
\centering
\includegraphics[width=1.05in]{figures/fig12_default_m1}\hfil
\includegraphics[width=1.05in]{figures/fig12_default_m2}\hfil
\includegraphics[width=1.05in]{figures/fig12_default_kro}
\caption{默认参数设置下算法在Map~1、Map~2、kroB100上规划的路径图。}
\label{fig:sens_paths}
\end{figure}"""
),
("% TODO: 图7--图10：消融收敛曲线、代价分布箱线图、平均代价/时间柱状图",
r"""\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig13a_conv_m1}\label{fig:abl_conv_a}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig13b_conv_m2}\label{fig:abl_conv_b}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig13c_conv_kro}\label{fig:abl_conv_c}}
\caption{六个消融组在三张地图上40次运行的平均迭代次数-代价收敛曲线。}
\label{fig:abl_conv}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig14a_box_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig14b_box_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig14c_box_kro}}
\caption{六个消融组在三张地图上40次运行的代价分布（箱线图）。}
\label{fig:abl_box}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig15a_costbar_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig15b_costbar_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig15c_costbar_kro}}
\caption{六个消融组在三张地图上的平均代价对比（柱状图）。}
\label{fig:abl_cost}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig16a_timebar_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig16b_timebar_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig16c_timebar_kro}}
\caption{六个消融组在三张地图上的平均计算时间对比（柱状图）。}
\label{fig:abl_time}
\end{figure*}"""
),
("% TODO: 图11--图15：TSP求解器收敛曲线、代价分布、平均代价/时间对比、路径图",
r"""\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig17a_tspconv_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig17b_tspconv_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig17c_tspconv_m3}}
\caption{四个旅行商求解器在三张地图上40次运行的平均时间-代价收敛曲线。}
\label{fig:tsp_conv}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig18a_tspbox_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig18b_tspbox_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig18c_tspbox_m3}}
\caption{四个旅行商求解器在三张地图上40次运行的代价分布（箱线图）。}
\label{fig:tsp_box}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig19a_tspcost_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig19b_tspcost_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig19c_tspcost_m3}}
\caption{四个旅行商求解器在三张地图上的平均代价对比（柱状图）。}
\label{fig:tsp_cost}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[Map~1]{\includegraphics[width=0.3\textwidth]{figures/fig20a_tsptime_m1}}\hfil
\subfloat[Map~2]{\includegraphics[width=0.3\textwidth]{figures/fig20b_tsptime_m2}}\hfil
\subfloat[kroB100]{\includegraphics[width=0.3\textwidth]{figures/fig20c_tsptime_m3}}
\caption{四个旅行商求解器在三张地图上的平均计算时间对比（柱状图）。}
\label{fig:tsp_time}
\end{figure*}

\begin{figure*}[!t]
\centering
\subfloat[MMAS-VND-CL（本文）]{\includegraphics[width=0.22\textwidth]{figures/fig12_default_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig12_default_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig12_default_kro}}\\
\subfloat[ACO]{\includegraphics[width=0.22\textwidth]{figures/fig21b_aco_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21b_aco_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21b_aco_m3}}\\
\subfloat[GA]{\includegraphics[width=0.22\textwidth]{figures/fig21c_ga_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21c_ga_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21c_ga_m3}}\\
\subfloat[SA]{\includegraphics[width=0.22\textwidth]{figures/fig21d_sa_m1}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21d_sa_m2}\hfil
\includegraphics[width=0.22\textwidth]{figures/fig21d_sa_m3}}
\caption{每个旅行商求解器在三张地图上规划的最优代价路径图。每行从左至右依次为Map~1、Map~2、Map~3。}
\label{fig:tsp_paths}
\end{figure*}"""
),
]

for old, new in replacements:
    if old not in text:
        print("NOT FOUND:", old[:70].replace("\n", " "))
    else:
        text = text.replace(old, new)
        print("OK replace")

anchor = "\\subsection{改进的开放旅行商蚁群优化}"
fig2 = r"""\begin{figure}[!t]
\centering
\subfloat[原始路径与简化后路径]{\includegraphics[width=1.55in]{figures/fig02a_simplify_raw}%
\label{fig:pipeline_a}}%
\hfil
\subfloat[简化后路径与平滑后路径]{\includegraphics[width=1.55in]{figures/fig02b_smooth_raw}%
\label{fig:pipeline_b}}
\caption{路径通过管线各阶段的对比。}
\label{fig:pipeline}
\end{figure}

"""
if anchor in text and "\\label{fig:pipeline}" not in text:
    text = text.replace(anchor, fig2 + anchor, 1)
    print("OK fig pipeline")

# Full sensitivity table from Word
old_tab = r"""\begin{table*}[!t]
\caption{代表性参数敏感性结果（选定配置）}
\label{tab:sens}
\centering
\begin{tabular}{llccccc}
\toprule
参数水平 & 地图 & 最优代价 & 平均收敛代价 & 成功率(\%) & 平均时间(s) & 标准差 \\
\midrule
\multirow{3}{*}{默认（均为水平3）}
& Map 1 & 351.4406 & 351.4406 & 100 & 0.0272 & 0 \\
& Map 2 & 620.1509 & 620.1509 & 100 & 0.2415 & 0 \\
& kroB100 & 22141.0000 & 22142.4500 & 97.5 & 0.3479 & 9.1706 \\
\midrule
$n_{\mathrm{ants}}=10$（水平1） & kroB100 & 22141.0000 & 22145.0250 & 92.5 & 0.2505 & 14.6629 \\
\midrule
\multirow{2}{*}{$q_0=0.90$（水平5）}
& Map 2 & 620.1509 & 620.7505 & 62.5 & 0.2321 & 1.1727 \\
& kroB100 & 22141.0000 & 22180.5500 & 50 & 0.3517 & 42.0579 \\
\midrule
\multirow{2}{*}{$y_{\max}=0.10$（水平1）}
& Map 2 & 620.1509 & 620.7129 & 65 & 0.2171 & 1.0482 \\
& kroB100 & 22141.0000 & 22157.0500 & 80 & 0.3654 & 35.3944 \\
\midrule
$r_{\mathrm{elite}}=0.1$（水平1） & kroB100 & 22141.0000 & 22145.3500 & 92.5 & 0.3420 & 15.4713 \\
\midrule
$k=3$（水平1） & kroB100 & 22141.0000 & 22193.3000 & 30 & 0.7868 & 37.4544 \\
\bottomrule
\end{tabular}
\end{table*}"""

# Build full table rows from Word structure
rows_src = """1/3/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 9.1 | 0
1/3/3/4/2 | Map2 | 620.15 | 620.34 | 80 | 148.0 | 0.44
1/3/3/4/2 | kroB100 | 22141.00 | 22145.03 | 92.5 | 250.5 | 14.66
2/3/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 15.1 | 0
2/3/3/4/2 | Map2 | 620.15 | 620.27 | 85 | 204.7 | 0.29
2/3/3/4/2 | kroB100 | 22141.00 | 22143.90 | 95 | 254.8 | 12.80
3/3/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 27.2 | 0
3/3/3/4/2 | Map2 | 620.15 | 620.15 | 100 | 241.5 | 0
3/3/3/4/2 | kroB100 | 22141.00 | 22142.45 | 97.5 | 347.9 | 9.17
4/3/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 40.4 | 0
4/3/3/4/2 | Map2 | 620.15 | 620.15 | 100 | 371.1 | 0
4/3/3/4/2 | kroB100 | 22141.00 | 22141.00 | 100 | 480.5 | 0
5/3/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 52.7 | 0
5/3/3/4/2 | Map2 | 620.15 | 620.15 | 100 | 336.3 | 0
5/3/3/4/2 | kroB100 | 22141.00 | 22142.45 | 97.5 | 628.7 | 9.17
3/1/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 37.8 | 0
3/1/3/4/2 | Map2 | 620.15 | 620.17 | 97.5 | 426.5 | 0.13
3/1/3/4/2 | kroB100 | 22141.00 | 22141.00 | 100 | 548.4 | 0
3/2/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 27.2 | 0
3/2/3/4/2 | Map2 | 620.15 | 620.15 | 100 | 311.8 | 0
3/2/3/4/2 | kroB100 | 22141.00 | 22141.00 | 100 | 475.7 | 0
3/4/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 30.5 | 0
3/4/3/4/2 | Map2 | 620.15 | 620.19 | 95 | 257.0 | 0.18
3/4/3/4/2 | kroB100 | 22141.00 | 22143.90 | 95 | 296.7 | 12.80
3/5/3/4/2 | Map1 | 351.44 | 351.44 | 100 | 38.5 | 0
3/5/3/4/2 | Map2 | 620.15 | 620.75 | 62.5 | 232.1 | 1.17
3/5/3/4/2 | kroB100 | 22141.00 | 22180.55 | 50 | 351.7 | 42.06
3/3/1/4/2 | Map1 | 351.44 | 351.44 | 100 | 44.4 | 0
3/3/1/4/2 | Map2 | 620.15 | 620.71 | 65 | 217.1 | 1.05
3/3/1/4/2 | kroB100 | 22141.00 | 22157.05 | 80 | 365.4 | 35.39
3/3/2/4/2 | Map1 | 351.44 | 351.44 | 100 | 32.2 | 0
3/3/2/4/2 | Map2 | 620.15 | 620.35 | 75 | 247.2 | 0.35
3/3/2/4/2 | kroB100 | 22141.00 | 22143.40 | 95 | 294.5 | 10.83
3/3/4/4/2 | Map1 | 351.44 | 351.44 | 100 | 24.9 | 0
3/3/4/4/2 | Map2 | 620.15 | 620.15 | 100 | 309.0 | 0
3/3/4/4/2 | kroB100 | 22141.00 | 22142.45 | 97.5 | 350.1 | 9.17
3/3/5/4/2 | Map1 | 351.44 | 351.44 | 100 | 28.1 | 0
3/3/5/4/2 | Map2 | 620.15 | 620.15 | 100 | 356.3 | 0
3/3/5/4/2 | kroB100 | 22141.00 | 22141.00 | 100 | 407.1 | 0
3/3/3/1/2 | Map1 | 351.44 | 351.44 | 100 | 32.1 | 0
3/3/3/1/2 | Map2 | 620.15 | 620.15 | 100 | 355.5 | 0
3/3/3/1/2 | kroB100 | 22141.00 | 22141.00 | 100 | 474.8 | 0
3/3/3/2/2 | Map1 | 351.44 | 351.44 | 100 | 40.0 | 0
3/3/3/2/2 | Map2 | 620.15 | 620.17 | 97.5 | 344.4 | 0.13
3/3/3/2/2 | kroB100 | 22141.00 | 22141.00 | 100 | 421.5 | 0
3/3/3/3/2 | Map1 | 351.44 | 351.44 | 100 | 27.5 | 0
3/3/3/3/2 | Map2 | 620.15 | 620.15 | 100 | 272.8 | 0
3/3/3/3/2 | kroB100 | 22141.00 | 22142.45 | 97.5 | 357.8 | 9.17
3/3/3/5/2 | Map1 | 351.4406 | 351.44 | 100 | 21.7 | 0.02
3/3/3/5/2 | Map2 | 620.15 | 620.21 | 92.5 | 211.6 | 0.22
3/3/3/5/2 | kroB100 | 22141.00 | 22145.35 | 92.5 | 342.0 | 15.47
3/3/3/4/1 | Map1 | 351.44 | 351.44 | 100 | 37.8 | 0
3/3/3/4/1 | Map2 | 620.15 | 620.17 | 97.5 | 217.8 | 0.13
3/3/3/4/1 | kroB100 | 22141.00 | 22193.30 | 30 | 786.8 | 37.45
3/3/3/4/3 | Map1 | 351.44 | 351.44 | 100 | 33.8 | 0
3/3/3/4/3 | Map2 | 620.15 | 620.15 | 100 | 243.2 | 0
3/3/3/4/3 | kroB100 | 22141.00 | 22143.90 | 95 | 404.3 | 12.80
3/3/3/4/4 | Map1 | 351.44 | 351.44 | 100 | 33.5 | 0
3/3/3/4/4 | Map2 | 620.15 | 620.17 | 97.5 | 286.0 | 0.13
3/3/3/4/4 | kroB100 | 22141.00 | 22142.45 | 97.5 | 390.8 | 9.17
3/3/3/4/5 | Map1 | 351.44 | 351.44 | 100 | 28.3 | 0
3/3/3/4/5 | Map2 | 620.15 | 620.19 | 95 | 287.7 | 0.18
3/3/3/4/5 | kroB100 | 22141.00 | 22142.45 | 97.5 | 487.7 | 9.17"""

mmap = {"Map1": "Map~1", "Map2": "Map~2", "kroB100": "kroB100"}
row_lines = []
for line in rows_src.strip().splitlines():
    parts = [p.strip() for p in line.split("|")]
    cfg, mp, best, avg, succ, tms, std = parts
    # mark default
    prefix = ""
    if cfg == "3/3/3/4/2":
        prefix = "\\textbf{"
        suffix = "}"
    else:
        suffix = ""
    # only bold first row of default block via multirow is hard; leave plain
    row_lines.append(f"{cfg} & {mmap.get(mp, mp)} & {best} & {avg} & {succ} & {tms} & {std} \\\\")

body = "\n".join(row_lines)
# highlight default config rows
body = body.replace(
    "3/3/3/4/2 & Map~1 & 351.44 & 351.44 & 100 & 27.2 & 0 \\\\",
    "3/3/3/4/2$^{*}$ & Map~1 & 351.44 & 351.44 & 100 & 27.2 & 0 \\\\",
)

new_tab = r"""\begin{table*}[!t]
\caption{代表性参数敏感性结果（选定配置）。参数水平依次为$n_{\mathrm{ants}}/q_0/y_{\max}/r_{\mathrm{elite}}/k$，$^{*}$表示默认配置。}
\label{tab:sens}
\centering
\scriptsize
\begin{tabular}{llccccc}
\toprule
取值等级 & 地图 & 最优代价 & 平均收敛代价 & 成功率(\%) & 平均时间(ms) & 标准差 \\
\midrule
""" + body + r"""
\bottomrule
\end{tabular}
\end{table*}"""

if old_tab in text:
    text = text.replace(old_tab, new_tab)
    print("OK full sens table")
else:
    print("SENS TABLE NOT FOUND")

# Update ablation table times s -> ms (from Word)
ablate_time_map = {
    "0.0272": "27.2",
    "0.2415": "241.5",
    "0.3479": "347.9",
    "0.0324": "32.4",
    "0.4157": "415.7",
    "0.7313": "731.3",
    "0.1339": "133.9",
    "0.5695": "569.5",
    "1.4739": "1473.9",
    "0.0175": "17.5",
    "0.3494": "349.4",
    "0.6570": "657.0",
    "0.0325": "32.5",
    "0.3325": "332.5",
    "0.4260": "426.0",
    "0.0218": "21.8",
    "0.0854": "85.4",
    "0.2484": "248.4",
}
# only replace inside ablation table caption area - safer: change header unit
text = text.replace(
    "组别 & 地图 & 最优代价 & 中位代价 & 平均代价 & 成功率(\\%) & 平均时间(s) & 代价极差 & 标准差 \\\\",
    "组别 & 地图 & 最优代价 & 中位代价 & 平均代价 & 成功率(\\%) & 平均时间(ms) & 代价极差 & 标准差 \\\\",
)

# Update TSP params table to Word full form
old_p = r"""\begin{table}[!t]
\caption{对比旅行商求解器的参数设置}
\label{tab:tsp_params}
\centering
\footnotesize
\begin{tabular}{lll}
\toprule
算法 & 参数 & 值 \\
\midrule
MMAS-VND-CL & $m,\alpha,\beta,\rho,Q,\tau_0$等 & 见正文 \\
ACO & $m,\alpha,\beta,\rho,Q,\tau_0$,iter & 40,1,2,0.25,225,0.1,100 \\
GA & pop,iter,$P_c,P_m$,elite & 100,1500,0.8,0.25,10 \\
SA & $T_0,\alpha,L,T_{\mathrm{end}}$ & 150,0.996,10,$10^{-8}$ \\
\bottomrule
\end{tabular}
\end{table}"""
new_p = r"""\begin{table}[!t]
\caption{对比旅行商求解器的参数设置}
\label{tab:tsp_params}
\centering
\scriptsize
\begin{tabular}{p{0.22\columnwidth}p{0.68\columnwidth}}
\toprule
算法 & 参数 \\
\midrule
MMAS-VND-CL & 蚂蚁数量$m=40$；信息素启发因子$\alpha=1$；期望启发因子$\beta=2$；信息素挥发系数$\rho=0.25$；信息素强度$Q=225$；初始信息素$\tau_0=0.1$；迭代次数$\mathrm{Iter}=200$；贪心选择概率$=0.45$；局部搜索比例$=0.4$；局部搜索过渡区间$=[0,30]$；局部搜索精英蚂蚁占比$=0.7$；候选邻居数$=9$ \\
ACO & 蚂蚁数量$m=40$；信息素启发因子$\alpha=1$；期望启发因子$\beta=2$；信息素挥发系数$\rho=0.25$；信息素强度$Q=225$；初始信息素$\tau_0=0.1$；迭代次数$\mathrm{Iter}=100$ \\
GA & 种群规模$\mathrm{Pop}=100$；迭代次数$\mathrm{Iter}=1500$；交叉概率$P_c=0.8$；变异概率$P_m=0.25$；精英保留数量$=10$ \\
SA & 初始温度$T_0=150$；降温系数$\alpha=0.996$；每个温度下的迭代次数$L=10$；终止温度$T_{\mathrm{end}}=10^{-8}$ \\
\bottomrule
\end{tabular}
\end{table}"""
if old_p in text:
    text = text.replace(old_p, new_p)
    print("OK tsp params table")
else:
    print("TSP PARAMS NOT FOUND")

# Update conclusion C2 continuous removal (Word dropped C2)
text = text.replace(
    "弧长参数化三次样条平滑将简化折线转化为适合物理机器人执行的$C^2$连续轨迹。",
    "弧长参数化三次样条平滑将简化折线转化为适合物理机器人执行的连续轨迹。",
)

# References -> English (from Word)
refs_start = text.find("\\begin{thebibliography}{32}")
refs_end = text.find("\\end{thebibliography}")
if refs_start < 0 or refs_end < 0:
    print("BIB NOT FOUND")
else:
    refs_end += len("\\end{thebibliography}")
    new_refs = r"""\begin{thebibliography}{32}

\bibitem{ref1}
S.~B. Liu et al., ``SMUG planner: A safe multi-goal planner for mobile robots in challenging environments,'' \textit{IEEE Robot. Autom. Lett.}, vol.~8, no.~9, pp.~5504--5511, 2023.

\bibitem{ref2}
H.~T.~T. Binh, ``Fast marching firework method for multi-goal mobile robot path planning in complex obstacle maps,'' \textit{Intell. Service Robot.}, vol.~18, 2025, doi: 10.1007/s11370-025-00654-6.

\bibitem{ref3}
C.-G. Wu, Y.-C. Liang, H.-P. Lee, C. Lu, and C. Lin, ``Solving constrained traveling salesman problems by genetic algorithms,'' \textit{Prog. Nat. Sci.}, vol.~14, no.~7, pp.~631--637, 2004.

\bibitem{ref4}
``A survey on approximability of traveling salesman problems using the TSP-T3CO definition scheme,'' \textit{Ann. Oper. Res.}, 2025, doi: 10.1007/s10479-025-06641-5.

\bibitem{ref5}
P.~E. Hart, N.~J. Nilsson, and B. Raphael, ``A formal basis for the heuristic determination of minimum cost paths,'' \textit{IEEE Trans. Syst. Sci. Cybern.}, vol.~4, no.~2, pp.~100--107, 1968.

\bibitem{ref6}
E.~W. Dijkstra, ``A note on two problems in connexion with graphs,'' \textit{Numer. Math.}, vol.~1, no.~1, pp.~269--271, 1959.

\bibitem{ref7}
S.~M. LaValle, ``Rapidly-exploring random trees: A new tool for path planning,'' Dept. Comput. Sci., Iowa State Univ., Ames, IA, USA, Tech. Rep. 98-11, 1998.

\bibitem{ref8}
``A comprehensive review of improved A* path planning algorithms and their hybrid integrations,'' \textit{Automation}, vol.~6, p.~52, 2025.

\bibitem{ref9}
L.~H. O. Rios and L. Chaimowicz, ``A survey and classification of A* based best-first heuristic search algorithms,'' in \textit{Proc. 20th Brazilian Symp. Artif. Intell. (SBIA)}, 2010, pp.~253--262.

\bibitem{ref10}
M. Likhachev, G. Gordon, and S. Thrun, ``ARA*: Anytime A* with provable bounds on sub-optimality,'' in \textit{Adv. Neural Inf. Process. Syst. (NIPS)}, 2003, pp.~767--774.

\bibitem{ref11}
D. Harabor and A. Grastien, ``Online graph pruning for pathfinding on grid maps,'' in \textit{Proc. 25th AAAI Conf. Artif. Intell. (AAAI-11)}, 2011, pp.~1114--1119.

\bibitem{ref12}
D. Harabor and A. Grastien, ``Improving jump point search,'' in \textit{Proc. 24th Int. Conf. Automated Planning Scheduling (ICAPS-14)}, 2014, pp.~128--135.

\bibitem{ref13}
``Improved JPS and DWA path planning algorithm for mobile robots,'' \textit{Expert Syst. Appl.}, 2026.

\bibitem{ref14}
``Research on mobile robot path planning based on an improved bidirectional jump point search algorithm,'' \textit{Electronics}, vol.~14, no.~8, p.~1669, 2025, doi: 10.3390/electronics14081669.

\bibitem{ref15}
``Adaptive weight optimization based jump point Theta* algorithm in mobile robot path planning for intricate environments,'' \textit{Discover Appl. Sci.}, vol.~7, p.~11, 2025, doi: 10.1007/s42452-025-07932-z.

\bibitem{ref16}
``UGV path optimization in UAV-assisted environments using visibility-aware path simplification,'' \textit{J. Sens. Actuator Netw.}, vol.~15, no.~3, p.~41, 2025.

\bibitem{ref17}
``Enhancing the safety and smoothness of path planning through an integration of Dijkstra's algorithm and piecewise cubic Bezier optimization,'' \textit{Expert Syst. Appl.}, 2025.

\bibitem{ref18}
``Adaptive A*/NSGA-II framework for multi-goal navigation of mobile robots,'' \textit{Meas. Sci. Technol.}, 2025, doi: 10.1088/1361-6501/ae9209.

\bibitem{ref19}
J.~H. Holland, \textit{Adaptation in Natural and Artificial Systems}. Ann Arbor, MI, USA: Univ. Michigan Press, 1975.

\bibitem{ref20}
S. Kirkpatrick, C.~D. Gelatt, and M.~P. Vecchi, ``Optimization by simulated annealing,'' \textit{Science}, vol.~220, no.~4598, pp.~671--680, 1983.

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
``An improvement to the 2-opt heuristic algorithm for approximation of optimal TSP tour,'' \textit{Appl. Sci.}, vol.~13, no.~12, p.~7339, 2023, doi: 10.3390/app13127339.

\bibitem{ref27}
``Matrix-based ant colony system for traveling salesman problem,'' in \textit{Proc. IEEE Int. Conf. Syst., Man, Cybern. (SMC)}, 2024.

\bibitem{ref28}
``nLKH-ACS: A niching Lin-Kernighan--Helsgaun-based ant colony system for multisolution traveling salesman problems,'' \textit{IEEE Trans. Evol. Comput.}, 2024.

\bibitem{ref29}
``CAACS: A carbon aware ant colony system,'' \textit{Sci. Rep.}, 2025.

\bibitem{ref30}
``Ant colony optimization for multiple traveling salesmen problem with revisitable cities,'' in \textit{Proc. IEEE}, 2024.

\bibitem{ref31}
B. Yao, Q. Hu, L. Zhang, and J. Tian, ``Improved ant colony optimization for seafood product delivery routing problem,'' \textit{Promet--Traffic Transp.}, vol.~26, no.~1, 2014.

\bibitem{ref32}
B. Yao, C. Chen, X. Song, and X. Yang, ``Fresh seafood delivery routing problem using an improved ant colony optimization,'' \textit{Ann. Oper. Res.}, vol.~273, no.~1--2, pp.~163--186, 2019, doi: 10.1007/s10479-017-2531-2.

\end{thebibliography}"""
    text = text[:refs_start] + new_refs + text[refs_end:]
    print("OK english refs")

tex_path.write_text(text, encoding="utf-8")
print("DONE remaining TODO", text.count("TODO"), "len", len(text))
