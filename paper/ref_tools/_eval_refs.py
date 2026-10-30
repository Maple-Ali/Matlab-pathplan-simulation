# -*- coding: utf-8 -*-
"""Extract cite contexts from intro/related work and bibliography for evaluation."""
import re
from pathlib import Path

p = Path(r"D:\Claude-File\shiyan1\paper\paper_CN_full\paper_CN.tex")
text = p.read_text(encoding="utf-8")

# Extract front matter: abstract through end of related work
start = text.find("\\section{引言}")
form = text.find("\\section{问题形式化}")
front = text[start:form]
print("=== FRONT LENGTH", len(front), "===")

# Find all cite keys with surrounding context
print("\n=== CITE CONTEXTS IN FRONT MATTER ===")
for m in re.finditer(r"\\textcolor\{blue\}\{\\cite\{([^}]+)\}\}|\\cite\{([^}]+)\}", front):
    keys = m.group(1) or m.group(2)
    s = max(0, m.start() - 120)
    e = min(len(front), m.end() + 80)
    ctx = front[s:e].replace("\n", " ")
    print(f"\nCITE [{keys}]")
    print("  ...", ctx, "...")

print("\n=== ALL CITE OCCURRENCES IN FULL TEXT (by key) ===")
cite_map = {}
for m in re.finditer(r"\\cite\{([^}]+)\}", text):
    keys = [k.strip() for k in m.group(1).split(",")]
    s = max(0, m.start() - 80)
    e = min(len(text), m.end() + 60)
    ctx = text[s:e].replace("\n", " ")
    for k in keys:
        cite_map.setdefault(k, []).append(ctx)

for k in sorted(cite_map, key=lambda x: (len(x), x)):
    if not k.startswith("ref"):
        continue
    print(f"\n--- {k} ({len(cite_map[k])} times) ---")
    for i, c in enumerate(cite_map[k][:3]):
        print(f"  [{i}]", c[:180])

print("\n=== BIB ENTRIES ===")
bib_start = text.find("\\begin{thebibliography}")
bib = text[bib_start:]
for m in re.finditer(r"\\bibitem\{([^}]+)\}\s*(.*?)(?=\\bibitem|\\end\{thebibliography\})", bib, re.S):
    key, body = m.group(1), m.group(2).strip().replace("\n", " ")
    print(f"{key}: {body[:220]}")
