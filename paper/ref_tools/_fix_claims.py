# -*- coding: utf-8 -*-
from pathlib import Path

p = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
t = p.read_text(encoding="utf-8")

fixes = [
    (
        r"旅行商问题是组合优化领域中研究最为深入的问题之一\textcolor{blue}{\cite{ref36,ref34}}，精确算法仅适用于小规模实例（通常$K \leq 20$）",
        r"旅行商问题是组合优化领域中研究最为深入的问题之一\textcolor{blue}{\cite{ref4,ref36}}，精确算法通常仅适用于小规模实例",
    ),
    (
        r"结合路径规划器与旅行商求解器的两层架构已有若干先例\textcolor{blue}{\cite{ref1,ref2}}。例如，概率路线图被用于构建连通图，并在其上以采样构型间的最短路径距离求解旅行商问题。然而，这些方法通常采用现成的规划器和求解器，未针对多目标点遍历上下文优化任一层。",
        r"结合路径规划器与旅行商求解器的两层架构已有若干先例\textcolor{blue}{\cite{ref1,ref2}}。然而，这些方法通常采用现成的规划器和求解器，未针对多目标点遍历上下文优化任一层。",
    ),
]

for old, new in fixes:
    if old not in t:
        print("MISSING:", old[:100])
    else:
        t = t.replace(old, new, 1)
        print("OK:", old[:60])

p.write_text(t, encoding="utf-8")
print("written")
