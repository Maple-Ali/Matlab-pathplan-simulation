# -*- coding: utf-8 -*-
from pathlib import Path
import re
t = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex").read_text(encoding="utf-8")
print("bibitems", len(re.findall(r"\\bibitem\{", t)))
print("模拟退火", t.count("模拟退火"))
print("概率路线图", t.count("概率路线图"))
for k in ["ref3", "ref9", "ref20", "ref31"]:
    # exact key usage in cite/bibitem
    cites = re.findall(r"\\cite\{[^}]*\}", t)
    used = any(re.search(rf"\b{k}\b", c) for c in cites)
    bib = bool(re.search(rf"\\bibitem\{{{k}\}}", t))
    print(k, "cited", used, "in bib", bib)
print("--- sentences ---")
for m in re.finditer(r".{0,20}研究最为深入.{0,80}", t):
    print(m.group(0).replace("\n"," "))
for m in re.finditer(r".{0,10}两层架构已有若干先例.{0,100}", t):
    print(m.group(0).replace("\n"," "))
