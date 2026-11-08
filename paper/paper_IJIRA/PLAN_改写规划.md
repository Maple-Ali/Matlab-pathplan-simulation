# IJIRA 投稿格式调整规划

**目标期刊**：International Journal of Intelligent Robotics and Applications (Springer, ISSN 2366-5971)
**原稿**：`paper/paper_EN/paper_EN.tex`（IEEEtran, `[lettersize,journal]`, 双栏, 19 页）
**新目录**：`paper/paper_IJIRA/`
**规划日期**：2026-11-07

---

## 0. 结论速览

| 项目 | 结论 |
|---|---|
| 整体可行性 | **可行**，无需重构论文骨架；章节层级（3 级）刚好满足期刊"不超过 3 级"要求 |
| 纯格式调整 | 约 **95%** 可由脚本机械完成（换文档类、前页、算法环境、文献、图表命名） |
| 必须动文字的部分 | **仅摘要**（318 词 → 期刊要求 150–250 词） |
| 需要你提供的信息 | 作者姓名/单位/通讯邮箱/ORCID、声明部分措辞、ref15 缺失的作者 |
| 需要你取舍的格式冲突 | **4 项**（见第 4 节），其中子图处理方式影响工作量最大 |
| 预计页数 | 单栏 A4、31pc 文本宽 → 约 **32–40 页**（原 19 页双栏，属正常膨胀） |

---

## 1. 依据

1. **IJIRA "Submission guidelines"**（Springer 官方 Instructions for Authors，2026-11-07 抓取）
2. **Springer Nature LaTeX 模板用户手册 v3.1（2024-12）** + `sn-jnl.cls` + `sn-article.tex` 样例
   （均在你提供的 `paper/sn-article-template/` 中）
3. 已本地验证：`sn-jnl.cls` 在本机 **TeX Live 2026 / pdfTeX** 上可正常编译（模板样例 11 页，0 error）

---

## 2. 期刊硬性要求 ⇄ 本稿现状 对照表

| # | IJIRA / SN 要求 | 本稿现状 | 处理方式 | 需你拍板 |
|---|---|---|---|---|
| 1 | 使用 Springer Nature LaTeX 模板 `sn-jnl.cls` | `IEEEtran` `[lettersize,journal]` 双栏 letter 纸 | 换文档类，A4 单栏 | — |
| 2 | 摘要 **150–250 词** | **318 词** | 压缩重写（第 3.1 节给出候选稿） | ✅ 审阅措辞 |
| 3 | 关键词 **4–6 个** | 4 个 | 保留，或补 1–2 个（如 jump point search、variable neighborhood descent） | 可选 |
| 4 | 正文引用为 **方括号数字**，文末按引用顺序编号 | IEEE 数字引用 `\cite{ref1}` | 已天然匹配；`sn-basic` + `Numbered` 选项 | — |
| 5 | 参考文献表用 Springer Basic 格式 | 手写 IEEE 格式 40 条 | 转 `references.bib` + BibTeX（第 3.4 节） | ✅ 认可 |
| 6 | 标题层级 **十进制、≤3 级** | 6 章 / 15 节 / 12 小节 = 3 级 | 满足，不改 | — |
| 7 | 标题页含：题目、作者、单位（院系/机构/城市/省/国家）、通讯作者有效邮箱、ORCID | 仅占位符 `Author Name, Member, IEEE` | 需真实信息 | ✅ **必须提供** |
| 8 | **"Statements and Declarations"** 段（Competing Interests 等） | 无 | 新建（第 3.6 节） | ✅ 确认措辞 |
| 9 | **Data Availability Statement**（原创研究必须有） | 无 | 新建 | ✅ 确认数据/代码共享方式 |
| 10 | 图件：矢量优先，文件名 `Fig1`、`Fig2`… | `figures/` 子目录，42 个文件，含 `&` 字符文件名 | 平铺到根目录并重命名 | — |
| 11 | 图题由模板自动生成 "Fig. 1" 加粗前标 | 无手写前标 | 无需改 | — |
| 12 | 表用 `\caption` + `booktabs` | 已用 `booktabs` + `\bottomrule` | `\bottomrule` → `\botrule`（SN 风格） | — |
| 13 | 算法用 `algorithmicx` + `algpseudocode` | 用旧 `algorithmic`（`\REQUIRE/\STATE/\WHILE`） | 改语法（第 3.5 节） | — |
| 14 | 单 `.tex` 文件提交，不要 `\input` | 已是单文件 | 满足 | — |
| 15 | 彩字在全文 XML 中不被支持 | **92 处 `\textcolor{blue}{}`** 修订高亮 | 全部去除（保留内部内容） | — |
| 16 | Regular paper 录用后需**作者简介 + 照片** | — | 录用后补，本次不做 | 知悉 |
| 17 | 稿件可用 Word 或 LaTeX 提交 | LaTeX | 保持 LaTeX | — |

---

## 3. 需要你确认 / 提供的信息

### 3.1 摘要压缩（318 → ≤250 词）——需要你审阅

期刊上限 250 词，现稿 318 词，**必须删改**。这是我拟的候选稿（228 词，美式拼写，与正文一致）：

> Multi-goal traversal path planning is fundamental to autonomous shuttle services and logistics
> delivery. Grid-based map modeling simplifies planning in complex environments containing many
> intermediate nodes. This paper proposes a two-layer collaborative framework that couples an improved
> A* planner with a Max-Min Ant System-Variable Neighborhood Descent-Candidate List (MMAS-VND-CL)
> solver for multi-goal traversal in cluttered grid environments. The geometric layer combines Jump
> Point Search (JPS), tie-breaking, safety-aware path simplification, and arc-length-parameterized
> cubic spline smoothing to generate collision-free continuous paths between point pairs; the arc
> lengths of these trajectories form the obstacle-aware cost matrix. The ordering layer solves the
> open traveling salesman problem with fixed start and end points through virtual-node encoding. In
> MMAS-VND-CL, a candidate-list-accelerated Variable Neighborhood Descent (VND) confines neighborhood
> evaluation to the nearest neighbors, so tours remain efficiently refinable even at large target
> scales, while MMAS pheromone bounds preserve diversity, an Ant Colony System-style pseudo-random
> transition balances exploration and exploitation, and an S-shaped schedule controls when
> local-search effort is spent. Experiments on three grid maps show that the improved A* reduces path
> length on Map 3 by 5.3% and planning time by 75.9% relative to conventional A*. In solver
> comparisons, MMAS-VND-CL reduces the mean path length on Map 1 by 10.0% and the running time by
> 57.4% relative to standard ant colony optimization, and it attains the published optimum of the
> kroB100 benchmark with a 97.5% hit rate.

删去的内容：开篇两句的空泛背景、对框架的第二次重复描述、"performance-tested on three grid maps with
different obstacle morphologies" 的冗长表述。**数值全部保留**。

> 若你希望保留更多细节，我也可以压到 245–250 词；若希望更短（如 200 词）也可以继续删。

### 3.2 作者信息（必须提供）

现在只有占位符。需要：

```
作者 1：姓 / 名 / 是否有 \equalcont / 单位编号
作者 2：…
通讯作者：用 \author* 标记 + \email{}
单位 1：\orgdiv{院系}, \orgname{学校}, \orgaddress{\street{路}, \city{城市}, \postcode{邮编}, \state{省}, \country{China}}
ORCID（可选，Springer 鼓励）
```

### 3.3 子图处理方式（工作量大头，见 4.2）

4 个图用了 `\subfloat` 多面板：`fig:pipeline`(2)、`fig:tsp_conv`(3)、`fig:tsp_box`(3)、`fig:googlemap`(3)。三种方案：

- **方案 A（推荐，改动最小）**：保留面板结构，`subfig` → `subcaption`，`\subfloat[x]{...}` → `\subfigure[x]{...}`。
  与 IJIRA "Figure parts should be denoted by lowercase letters (a, b, c)" 完全一致。轻微偏离 SN 手册"避免子图"的**建议**。
- **方案 B（最符合 SN 手册字面）**：把每组面板合成为**一张 PDF**，把 (a)(b)(c) 标签烧进图里，正文只 `\includegraphics` 一次。
  需要我用 LaTeX 重新排版合成图，视觉细节可能与你原图有细微差异。
- **方案 C**：拆成独立图（Fig.7a、Fig.7b 各自成图），图号会变多，正文 `\ref` 需改指代。

**默认按 A 执行**，你若要 B/C 请告知。

### 3.4 参考文献（建议转 BibTeX）

- 现有：手写 `thebibliography`，40 条，IEEE 风格（`vol.~8, no.~11, pp.~7170--7177`）。
- 计划：转成 `references.bib`（40 条，`@article` / `@inproceedings` / `@book`），配合 `sn-basic.bst` + `Numbered`，
  由 BibTeX 自动排出 Springer 格式：
  `Chen, C., Frey, J., Arm, P., Hutter, M.: SMUG planner: a safe multi-goal planner... IEEE Robot. Autom. Lett. 8(11), 7170–7177 (2023). https://doi.org/10.1109/LRA.2023.3311207`
- **优点**：格式 100% 符合 .bst，避免手改 40 条出错；Springer 审稿/生产阶段偏好 .bib。
- **需要你补的一条**：`ref15`（"Adaptive weight optimization based jump point Theta* algorithm…", *Discover Appl. Sci.*, 2025）
  **缺作者名**。请补作者，或确认从参考文献中删除。

### 3.5 声明部分（Statements and Declarations）

IJIRA 要求投稿时在系统中填写 Author Contribution 与 Competing Interest，同时文内需有相应段落。
我拟加入以下内容（`\section*{Declarations}` 或 SN 的 `\bmhead`，按模板样例用 `\section*`）：

```
Funding            —— 需你确认（无资助写 "The authors received no specific funding for this work."）
Competing interests—— 需你确认（通常 "The authors declare no competing interests."）
Ethics approval    —— Not applicable.
Consent to publish —— Not applicable.
Data availability  —— 需你确认，见下
Code availability  —— 需你确认（是否公开 MATLAB 代码仓库/GitHub 链接）
Author contributions—— 需你确认（CRediT 或自由文本）
```

**Data availability 必须写**。若暂不公开，可用标准句式：
"The datasets generated and analysed during the current study are available from the corresponding author on reasonable request."
若你愿意开源（IJIRA 强烈鼓励），提供 GitHub / Zenodo DOI 更好。

---

## 4. 难以完全满足 / 存在取舍的要求（重点告知）

### 4.1 摘要长度 —— **硬性冲突，必须改文字**
318 词 vs 150–250 词。这不是排版能解决的，必须删句。已在 3.1 给出候选稿。

### 4.2 子图 —— **SN 手册"建议避免"，IJIRA 又要求 (a)(b)(c) 分图标号**
SN 用户手册 §7.3 原文："Also make sure that each figure is from a single input image file. Avoid using subfigures."
而 IJIRA 图件指南："Figure parts should be denoted by lowercase letters (a, b, c, etc.)."
两者并不真正矛盾（IJIRA 指的是**一张图里的分块**），但字面上要求"单文件"。
→ 我建议按方案 A 保留分面板，这是审稿阶段最常见做法，生产阶段也能处理；若你追求零风险则选方案 B。

### 4.3 图件格式：EPS vs PDF —— **建议保留 PDF**
- IJIRA 图件指南："For vector graphics, the preferred format is EPS"。
- 但 SN 的 LaTeX 管线（`sn-jnl.cls` 默认 `pdflatex` 模式）接受 PDF/PNG/JPG；现稿 42 个图**全部是矢量 PDF**。
- 转 EPS 会经过一次栅格化或字体替换，**画质反而可能下降**。
- → 建议保留 PDF。若期刊返修时坚持要 EPS，我再批量转（`pdftops -eps`）。

### 4.4 图题末尾不加标点 —— **与多句图题冲突**
IJIRA 图件指南："No punctuation … is to be placed at the end of the caption."
本稿 18 个图题中多个是**多句图题**（如 `fig:env_maps`、`fig:googlemap`），末句去掉句号会显得不自然。
→ 建议：**单句图题去掉末尾句号**；多句图题保留（Springer 自身大量论文也如此）。需要你同意这个折中。

### 4.5 中文附录不能直接用
`paper_EN/exp_data/exp_data_appendix.tex` 是**中文**文档（`ctex` + `IEEEtran`），IJIRA 是英文期刊。
→ 三种选择：(a) 不提交；(b) 译成英文作为 Supplementary Information（Online Resource）；(c) 并入正文附录。
**本次默认不处理**，如需要请说明。

### 4.6 页数与版式观感会明显变化
- letter 双栏 19 页 → A4 单栏约 32–40 页；通栏图（7 个 `figure*`）在单栏版式中与普通图无异。
- `sn-jnl.cls` 用 `\LoadClass[twoside,fleqn]{article}`：**公式左对齐**（SN 风格），与 IEEE 居中公式不同，这是正常的、无需修正。
- IEEE 的 `\thanks{Manuscript received...}`、`\markboth`、`\IEEEmembership` 全部删除。

### 4.7 其它小项
- `\multirow{1}{*}{ACS}` 这种 `multirow` 包 1 行的写法可简化为普通单元格（不影响结果）。
- 文件名含 `&`（`Ablation_Cost&Time` 等 7 个）：LaTeX 能编译，但生产环境/上传系统可能出错，统一改名去掉 `&`。
- 关键：`\cite` 与 `natbib` 冲突 → 必须卸载 `cite` 宏包（`sn-jnl.cls` 已内置 `natbib` `[numbers,sort&compress]`）。

---

## 5. 逐项转换清单（我将执行的改动）

### 5.1 导言区

| 原 | 新 |
|---|---|
| `\documentclass[lettersize,journal]{IEEEtran}` | `\documentclass[pdflatex,sn-basic,Numbered]{sn-jnl}`（投稿可选加 `referee,lineno`） |
| `amsmath,amsfonts,amssymb` | 保留 |
| `algorithmic` + `algorithm` | 移除；改用 `algorithm`（类文件已备）+ `algorithmicx` + `algpseudocode` |
| `subfig` | 移除 → `subcaption`（方案 A）或不用 |
| `cite` | **移除**（与 natbib 冲突） |
| `color` | 移除 → `xcolor`（SN 样例用） |
| `hyperref` | 移除（类文件已 `\RequirePackage{hyperref}`） |
| `stfloats` / `flafter` / `float` / `verbatim` / `setspace` | 移除（不需要） |
| `textcomp` / `url` / `booktabs` / `multirow` / `graphicx` / `array` / `bm` / `makecell` | 保留（按 SN 样例顺序重排） |
| `\renewcommand{\algorithmicrequire}` … | 移除（改用 `\Require` / `\Ensure`） |
| `\hyphenation{...}` | 移除或保留（无害） |

### 5.2 前页（front matter）

```latex
\title[Multi-Goal Traversal Planning via Improved A* and MMAS-VND-CL]{<全文标题>}
\author*[1]{\fnm{名} \sur{姓}}\email{通讯邮箱}
\author[1]{\fnm{名} \sur{姓}}
\affil[1]{\orgdiv{...}, \orgname{...}, \orgaddress{\street{...}, \city{...}, \postcode{...}, \state{...}, \country{China}}}
\abstract{<3.1 的 228 词版本>}
\keywords{multi-goal traversal, path planning, ant colony optimization, open traveling salesman problem}
\maketitle
```

### 5.3 正文

| 改动 | 规模 |
|---|---|
| 删除 `\textcolor{blue}{...}` 外壳（保留内部 `\cite`/`\ref`/`\eqref`） | **92 处** |
| `\begin{IEEEkeywords}` → `\keywords{}` | 1 处 |
| 算法 3 个：`\REQUIRE→\Require`、`\ENSURE→\Ensure`、`\STATE→\State`、`\WHILE…\ENDWHILE→\While…\EndWhile`、`\IF…\ENDIF→\If…\EndIf`、`\REPEAT…\UNTIL→\Repeat…\Until`、`\RETURN→\Return` | 3 处 / 约 60 行 |
| 删除 `\begin{spacing}{0.9}…\end{spacing}` | 3 处 |
| `\begin{figure*}` → `\begin{figure}`（单栏版式下通栏无意义） | 7 处 |
| `\bottomrule` → `\botrule` | 7 处 |
| `\subfloat[x]{...}` → `\subfigure[x]{...}`（方案 A） | 11 处 |
| 图片路径 `figures/xxx` → `FigN`（平铺 + 重命名） | 42 处 |
| 图题末尾句号（单句图题去掉，多句保留） | 视确认 |

### 5.4 后置部分（back matter）

```latex
\backmatter
\bmhead{Acknowledgements}        % 可选
\section*{Declarations}
  Funding / Competing interests / Ethics approval / Consent to publish /
  Data availability / Code availability / Author contributions
\bibliography{references}
\end{document}
```

---

## 6. 目录结构与文件规划

```
paper/
├── paper_EN/                 ← 原稿，只读，绝不修改
├── sn-article-template/      ← 期刊模板原件，只读
└── paper_IJIRA/              ← 本次新建
    ├── sn-jnl.cls            ✅ 已复制
    ├── sn-basic.bst          ✅ 已复制（类文件自动调用 sn-basic）
    ├── bst/                  ✅ 已复制（全套，备切换引用风格）
    ├── paper_IJIRA.tex       ← 待生成（单文件正文）
    ├── references.bib        ← 待生成（40 条）
    ├── Fig1.pdf … Fig18.pdf  ← 待生成（平铺，Springer 命名）
    ├── PLAN_改写规划.md       ← 本文件
    └── build/                ← 编译中间产物（.aux/.log/.bbl），保持根目录干净
```

---

## 7. 执行步骤

| 步骤 | 内容 | 产物 |
|---|---|---|
| S0 | 目录与模板素材 | ✅ 已完成 |
| S1 | 写脚本做机械转换：导言区 + 前页 + 92 处颜色 + 算法 + 图表环境 | `paper_IJIRA.tex` 雏形 |
| S2 | 图片平铺重命名（`&` 改名，`figures/x`→`FigN`），回填 `\includegraphics` | `Fig*.pdf` |
| S3 | 40 条参考文献 → `references.bib` | `references.bib` |
| S4 | 摘要替换、声明段、关键词 | — |
| S5 | 全文编译（`latexmk -pdf` + `bibtex`），修 overfull / 未定义引用 | `paper_IJIRA.pdf` |
| S6 | 逐页目检（图是否越界、表格是否超宽、算法是否溢出） | 校核清单 |
| S7 | 输出差异说明 + 待你补的空位清单 | 交付 |

**验证命令**（我会执行）：

```bash
cd paper_IJIRA
latexmk -pdf -interaction=nonstopmode paper_IJIRA.tex
# 检查：0 error、0 undefined reference、无 Overfull \hbox > 5pt
```

---

## 8. 验收标准

- [ ] `pdflatex` + `bibtex` 两轮后 **0 error**，log 中无 `Undefined control sequence`
- [ ] 所有 46 个 `\ref` / 4 个 `\eqref` / 40 个 citation 全部有定义（脚本已校验，改后需复验）
- [ ] 18 图 / 7 表 / 3 算法 / 40 文献编号连续、正文引用顺序正确
- [ ] 摘要 ≤250 词，关键词 4–6 个
- [ ] 首页含完整作者、单位、通讯邮箱
- [ ] 文末含 Declarations（含 Data availability）
- [ ] 全文无 `\textcolor`、无 `\IEEEmembership`、无 `\markboth`、无 `\thanks`
- [ ] 无 Overfull \hbox 超过 5pt 的图表

---

## 9. 改动量统计（供你评估工作量）

| 项目 | 数量 |
|---|---|
| 正文行数 | 1130 |
| 一级/二级/三级标题 | 6 / 15 / 12 |
| `\textcolor{blue}{}` | 92 |
| `\cite` 命令 / 被引文献 | 36 / 40 |
| `\ref` / `\eqref` | 52 / 4 |
| 图（普通 / 通栏） | 11 / 7 |
| 表 | 7 |
| 算法 | 3 |
| 独立图形文件 | 42（58 个 PDF 中的使用子集） |
| 公式环境（equation / align） | 10 / 1 |
| 参考文献条目 | 40 |
| 摘要词数 | 318 → ≤250 |

---

## 10. 现在需要你回复的 5 件事

1. **作者信息**：姓名、单位、通讯作者邮箱、ORCID（可后补 ORCID）。
2. **摘要候选稿**：3.1 的版本可以吗？还是要我调长/调短？
3. **子图方案**：A（保留分面板，推荐）/ B（合成单图）/ C（拆图）。
4. **声明部分**：Funding、Competing interests、Data availability、Code availability、Author contributions 的措辞（或授权我用通用句式先占位）。
5. **`ref15` 缺失作者**：补作者还是删除该条？

其余（文档类、图表、算法、文献格式、颜色高亮、命名）我按本规划直接执行，无需你逐项确认。
